#include <takasako_sps/scpi_tcp_client.h>

TakasokoSps::TakasokoSps(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{
  nhp_.param("tm_loop_rate", tm_loop_rate_, 10.0);
  nhp_.param("port", port_, 50001);
  nhp_.param("ip_address", ip_address_, std::string("192.168.0.1"));

  //socket

  if((sd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    ROS_ERROR("can not open socket");
    ros::shutdown();
    return;
  }
 
  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(port_);
  addr_.sin_addr.s_addr = inet_addr(ip_address_.c_str());
  connect(sd_, (struct sockaddr *)&addr_, sizeof(struct sockaddr_in));

  //initialize
  if  (write(sd_, "ADDR 1;:ALM:CLEar\n",18)==-1 )
    {
      ROS_ERROR("can not send socket");
      ros::shutdown();
      return;
    }
  memset(read_buf_, 0, sizeof(read_buf_));
  read(sd_, read_buf_, sizeof(read_buf_));
  printf("%s", read_buf_);
  if(!strstr(read_buf_, "OK"))
    {
      ROS_ERROR("can not initialize");
      ros::shutdown();
      return;
    }

  if  (write(sd_, "*IDN?\n",6)==-1 )
    {
      ROS_ERROR("can not send socket");
      ros::shutdown();
      return;
    }
  memset(read_buf_, 0, sizeof(read_buf_));
  read(sd_, read_buf_, sizeof(read_buf_));
  ROS_INFO("initialized: %s", read_buf_);

  power_info_pub_ = nh_.advertise<takasako_sps::PowerInfo>("power_info", 5);
  power_on_sub_ = nh_.subscribe<std_msgs::Empty>("power_on_cmd", 1, &TakasokoSps::powerOnCallback, this, ros::TransportHints().tcpNoDelay());
  power_off_sub_ = nh_.subscribe<std_msgs::Empty>("power_off_cmd", 1, &TakasokoSps::powerOffCallback, this, ros::TransportHints().tcpNoDelay());
  check_sub_ = nh_.subscribe<std_msgs::Bool>("check_vol_and_power", 1, &TakasokoSps::checkVolCallback, this, ros::TransportHints().tcpNoDelay());
  timer_ = nhp_.createTimer(ros::Duration(1.0 / tm_loop_rate_), &TakasokoSps::powerInfoFunction,this);

}


void TakasokoSps::powerOnCallback(const std_msgs::EmptyConstPtr &msg)
{
  if  (write(sd_, "OUTP ON\n" ,8)==-1 )
    {
      ROS_ERROR("can not send socket");
      return;
    }

  memset(read_buf_, 0, sizeof(read_buf_));
  read(sd_, read_buf_, sizeof(read_buf_));

  //bad
  ROS_ERROR("power on!");
  power_flag_ = true;

  /*
  if(strstr(read_buf_, "OK"))
    {
      ROS_ERROR("power on!");
      power_flag_ = true;
    }
  else
    {
      ROS_ERROR("%s", read_buf_);
    }
  */
}

void TakasokoSps::powerOffCallback(const std_msgs::EmptyConstPtr &msg)
{
  if  (write(sd_, "OUTP OFF\n" ,9)==-1 )
    {
      ROS_ERROR("can not send socket");
      return;
    }

  memset(read_buf_, 0, sizeof(read_buf_));
  read(sd_, read_buf_, sizeof(read_buf_));

  //bad
  ROS_ERROR("power off!");
  power_flag_ = false;

  /*
  if(strstr(read_buf_, "OK"))
    {
      ROS_ERROR("power off!");
      power_flag_ = false;
    }
  else
    {
      ROS_ERROR("%s", read_buf_);
    }
  */

}

void TakasokoSps::checkVolCallback(const std_msgs::BoolConstPtr &msg)
{
  check_flag_ = msg->data;
}

void TakasokoSps::powerInfoFunction(const ros::TimerEvent & e)
{
  if(!power_flag_) return;

  takasako_sps::PowerInfo power_info;
  std::stringstream ss_curr,ss_volt;
  float voltage = 0.0f, currency = 0.0f;

  if (check_flag_) {
    // voltage
    if (write(sd_, "MEAS:VOLT?\n", strlen("MEAS:VOLT?\n")) == -1){
      ROS_ERROR("can not send socket");
      return;
    }
    memset(read_buf_, 0, sizeof(read_buf_));
    int n = read(sd_, read_buf_, sizeof(read_buf_) - 1);
    if (n <= 0) { ROS_ERROR("read voltage failed"); return; }
    read_buf_[n] = '\0';

    {
      std::stringstream ss;
      ss.str(std::string(read_buf_));
      ss >> voltage;
      if (ss.fail()) { ROS_ERROR("parse voltage failed: %s", read_buf_); return; }
    }
    power_info.voltage = voltage;
  }

  if (write(sd_, "MEAS:CURR?\n",std::strlen("MEAS:CURR?\n"))==-1 )
    {
      ROS_ERROR("can not send socket");
      return;
    }

  // current
  memset(read_buf_, 0, sizeof(read_buf_));
  int n = read(sd_, read_buf_, sizeof(read_buf_) - 1);
  if (n <= 0) { ROS_ERROR("read current failed"); return; }
  read_buf_[n] = '\0';

  {
    std::stringstream ss;
    ss.str(std::string(read_buf_));
    ss >> currency;
    if (ss.fail()) { ROS_ERROR("parse current failed: %s", read_buf_); return; }
  }
  power_info.currency = currency;

  if(check_flag_){
    power_info.power = voltage * currency;
  }

  power_info_pub_.publish(power_info);

}
