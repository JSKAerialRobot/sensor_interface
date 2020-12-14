#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include <lepton_thermal_sensor/sensor_handler.h>
#include <lepton_thermal_sensor/LeptonCameraConfig.h>

#include <boost/thread.hpp>
#include <fstream>
#include <mutex>

namespace lepton_camera_driver
{
  class LeptonThermalSensorNodelet: public nodelet::Nodelet
  {
  public:
    LeptonThermalSensorNodelet() {}

    ~LeptonThermalSensorNodelet()
    {
      if(pubThread_)
        {
          pubThread_->interrupt();
          pubThread_->join();

          try
            {
              NODELET_DEBUG("Disconnecting from camera.");
              sh_->disconnect();
            }
          catch(std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }
        }
    }

  private:
    void onInit()
    {
      // Get nodeHandles
      ros::NodeHandle &nh = getMTNodeHandle();
      ros::NodeHandle &pnh = getMTPrivateNodeHandle();

      // Get the sensor polling configuration and create the sensor handler
      std::string spi_dev, i2c_dev;
      int spi_speed;
      pnh.param("spi_dev", spi_dev, std::string("/dev/spidev0.1"));
      pnh.param("spi_speed", spi_speed, 1000000);
      pnh.param("i2c_dev", i2c_dev, std::string("/dev/i2c-0"));
      int packets_per_frame, packet_size, head_bytes, reboot_max_cnt;
      pnh.param("packets_per_frame", packets_per_frame, 60);
      pnh.param("packet_size", packet_size, 164);
      pnh.param("head_bytes", head_bytes, 4);
      pnh.param("reboot_max_cnt", reboot_max_cnt, 750);
      sh_.reset(new SensorHandler(i2c_dev, spi_dev, spi_speed, packets_per_frame, packet_size, head_bytes, reboot_max_cnt));

      // Get the location of our camera config yaml
      std::string camera_info_url, camera_model, camera_frame;
      pnh.param("camera_info_url", camera_info_url, std::string(""));
      pnh.param("camera", camera_model, std::string("camera"));
      pnh.param("camera_frame", camera_frame, std::string("camera"));

      // Start the camera info manager and attempt to load any configurations
      cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, camera_model, camera_info_url));
      ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
      ci_->header.frame_id = camera_frame + std::string("_optical");

      // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
      it_.reset(new image_transport::ImageTransport(nh));
      image_transport::SubscriberStatusCallback cb = boost::bind(&lepton_camera_driver::LeptonThermalSensorNodelet::connectCb, this);
      it_pub_color_ = it_->advertiseCamera("color/image", 5, cb, cb);
      it_pub_temp_ = it_->advertiseCamera("temperature/image", 5, cb, cb);
      it_pub_thresh_ = it_->advertiseCamera("threshold/image", 5, cb, cb);

      // Dynamic reconfigure server
      reconf_server_.reset(new dynamic_reconfigure::Server<lepton_thermal_sensor::LeptonCameraConfig>(pnh));
      reconf_func_ = boost::bind(&lepton_camera_driver::LeptonThermalSensorNodelet::cfgCb, this, _1, _2);
      reconf_server_->setCallback(reconf_func_);
    }

    void connectCb()
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);

      NODELET_DEBUG("Connect callback!");
      // Check if we should disconnect (there are 0 subscribers to our data)
      if(it_pub_color_.getNumSubscribers() + it_pub_temp_.getNumSubscribers() + it_pub_thresh_.getNumSubscribers() == 0)
        {
          if (pubThread_)
            {
              NODELET_DEBUG("Disconnecting.");
              pubThread_->interrupt();
              pubThread_->join();
              pubThread_.reset();

              sh_->disconnect();
            }
        }
      else if(!pubThread_)  // We need to connect
        {
          // Start the thread to loop through and publish messages
          pubThread_.reset(new boost::thread(boost::bind(&LeptonThermalSensorNodelet::devicePoll, this)));
        }
      else
        {
          NODELET_DEBUG("Do nothing in callback.");
        }
    }

    void devicePoll()
    {
      enum State
      {
        NONE
        , ERROR
        , DISCONNECTED
        , CONNECTED
      };

      State state = DISCONNECTED;
      State previous_state = NONE;

      double prev_temp_timestamp = 0;
      double get_temp_du;
      double ref_temp;
      double temp_convert_a, temp_convert_b;
      ros::NodeHandle &pnh = getMTPrivateNodeHandle();
      pnh.param("get_temp_du", get_temp_du, 1.0);
      pnh.param("temp_convert_a", temp_convert_a, 1.0);
      pnh.param("temp_convert_b", temp_convert_b, 0.0);
      pnh.param("temp_thresh", temp_thresh_, 50.0);

      cv::Mat raw_map(sh_->getImageSize(), CV_16UC1); // 16bit raw data
      cv::Mat thermal_map(sh_->getImageSize(), CV_8UC3); // color data for visualization
      cv::Mat temp_map(sh_->getImageSize(), CV_16SC1); // temperature (scale: x100 C)
      cv::Mat thresh_map(sh_->getImageSize(), CV_8UC1); // temperature (scale: x100 C).

      while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
        {
          bool state_changed = state != previous_state;

          previous_state = state;

          switch(state)
            {
            case ERROR:
            case DISCONNECTED:
              // Try connecting to the camera
              try
                {
                  NODELET_DEBUG("Connecting to thermal sensor.");
                  sh_->connect();
                  NODELET_INFO("Connected to thermal sensor.");

                  state = CONNECTED;
                }
              catch(std::runtime_error& e)
                {
                  NODELET_ERROR_COND(state_changed,
                                     "Failed to connect with error: %s", e.what());
                  ros::Duration(1.0).sleep(); // sleep for one second each time
                }

              break;
            case CONNECTED:
              try
                {
                  double timestamp;
                  NODELET_DEBUG("Starting a new grab from camera.");
                  if(!sh_->grabImage(raw_map, timestamp))  return;

                  // Publish the message using standard image transport
                  if(it_pub_color_.getNumSubscribers() > 0)
                    {
                      double min_val, max_val;
                      cv::minMaxLoc(raw_map, &min_val, &max_val);

                      thermal_map.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int position[2]) -> void
                        {
                          uint8_t normalized_val = (raw_map.ptr<uint16_t>(position[0])[ position[1] ] - min_val ) / (max_val - min_val) * 255.0;
                          //NODELET_INFO("[%d, %d]: %d", position[1], position[0], );
                          p[0] = Lepton::colorMap(3 * normalized_val);
                          p[1] = Lepton::colorMap(3 * normalized_val + 1);
                          p[2] = Lepton::colorMap(3 * normalized_val + 2);
                        });

                      ci_->header.stamp.fromSec(timestamp);
                      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(ci_->header, sensor_msgs::image_encodings::RGB8, thermal_map).toImageMsg();
                      it_pub_color_.publish(img_msg, ci_);
                    }

                  if(it_pub_temp_.getNumSubscribers() > 0 || it_pub_thresh_.getNumSubscribers() > 0)
                    {
                      if(ros::Time::now().toSec() - prev_temp_timestamp > get_temp_du)
                        {
                          /* get temperature in Kelvin */
                          ref_temp = sh_->getSensorTemperature();
                        }
                      temp_map.forEach<int16_t>([&](int16_t &p, const int position[2]) -> void
                        {
                          // TODO: the raw_value-temperature model should be more accurate
                          // based on http://takesan.hatenablog.com/entry/2016/02/18/194252
                          p = (temp_convert_a * raw_map.ptr<uint16_t>(position[0])[ position[1] ] + ref_temp + temp_convert_b);
                        });

                      if(it_pub_temp_.getNumSubscribers() > 0)
                        {
                          //double min_val, max_val;
                          //cv::minMaxLoc(temp_map, &min_val, &max_val);
                          //NODELET_INFO("max temp: %f, min temp: %f", max_val, min_val);
                          sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(ci_->header, sensor_msgs::image_encodings::TYPE_16SC1, temp_map).toImageMsg();
                          it_pub_temp_.publish(img_msg, ci_);
                        }

                      if(it_pub_thresh_.getNumSubscribers() > 0)
                        {
                          // cv::threshold() can only be applied for 8UC1
                          thresh_map.forEach<uint8_t>([&](uint8_t &p, const int position[2]) -> void
                            {
                              if(temp_map.ptr<int16_t>(position[0])[ position[1] ] >= temp_thresh_)
                                p = 255;
                              else
                                p = 0;
                            });

                          sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(ci_->header, sensor_msgs::image_encodings::MONO8, thresh_map).toImageMsg();
                          it_pub_thresh_.publish(img_msg, ci_);
                        }
                    }
                }
              catch(std::runtime_error& e)
                {
                  NODELET_ERROR("%s", e.what());

                  state = ERROR;
                }
              break;
            default:
              NODELET_ERROR("Unknown camera state %d!", state);
            }
        }
      NODELET_DEBUG("Leaving thread.");
    }

    void cfgCb(lepton_thermal_sensor::LeptonCameraConfig &config, uint32_t level)
    {
      switch(level)
        {
        case 0:
          temp_thresh_ = config.temperature_threshold;
          NODELET_INFO("change the temperature threshould to %f", temp_thresh_);
          break;
        default :
          break;
        }
    }

    std::mutex connect_mutex_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    image_transport::CameraPublisher it_pub_color_, it_pub_temp_, it_pub_thresh_;

    boost::shared_ptr<dynamic_reconfigure::Server<lepton_thermal_sensor::LeptonCameraConfig>> reconf_server_;
    dynamic_reconfigure::Server<lepton_thermal_sensor::LeptonCameraConfig>::CallbackType reconf_func_;

    boost::shared_ptr<SensorHandler> sh_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
    sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
    boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.
    double temp_thresh_;
  };

  PLUGINLIB_EXPORT_CLASS(lepton_camera_driver::LeptonThermalSensorNodelet, nodelet::Nodelet)
}


