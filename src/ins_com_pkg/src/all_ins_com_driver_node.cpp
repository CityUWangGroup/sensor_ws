#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_driver_msgs/msg/ins_byins.hpp"
#include "sensor_driver_msgs/msg/gnss_gpdop.hpp"
#include "sensor_driver_msgs/msg/gnss_gpfpd.hpp"
#include "sensor_driver_msgs/msg/gnss_gpgga.hpp"
#include "sensor_driver_msgs/msg/gnss_gpgst.hpp"
#include "sensor_driver_msgs/msg/gnss_gphdt.hpp"
#include "sensor_driver_msgs/msg/gnss_gpzda.hpp"
#include "sensor_driver_msgs/msg/gnss_pashr.hpp"
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <chrono>

/*驱动一台*/
class ins_com : public rclcpp::Node{
 public:
  // 构造函数,有一个参数为节点名称
  explicit ins_com(std::string name) : Node(name){
    // 创建发布
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>("/Sensor_msgs/INS/rawData", 10);
    Byins_publisher_ = this->create_publisher<sensor_driver_msgs::msg::InsByins>("/Sensor_msgs/INS/BYINS", 10);
    ByinsIns_NavSatFix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/Sensor_msgs/INS/NavSatFix_BYINS_ins", 10);
    ByinsGnss_NavSatFix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/Sensor_msgs/INS/NavSatFix_BYINS_gnss", 10);
    ByinsRaw_Imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/Sensor_msgs/INS/Imu_BYINS_raw", 10);
    ByinsJudged_Imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/Sensor_msgs/INS/Imu_BYINS_judged", 10);
    Gpdop_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssGpdop>("/Sensor_msgs/INS/GPDOP", 10);
    Gpfpd_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssGpfpd>("/Sensor_msgs/INS/GPFPD", 10);
    Gpfpd_NavSatFix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/Sensor_msgs/INS/NavSatFix_GPFPD", 10);
    Gpgga_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssGpgga>("/Sensor_msgs/INS/GPGGA", 10);
    Gpgga_NavSatFix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/Sensor_msgs/INS/NavSatFix_GPGGA", 10);
    Gpgst_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssGpgst>("/Sensor_msgs/INS/GPGST", 10);
    Gphdt_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssGphdt>("/Sensor_msgs/INS/GPHDT", 10);
    Gpzda_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssGpzda>("/Sensor_msgs/INS/GPZDA", 10);
    Pashr_publisher_ = this->create_publisher<sensor_driver_msgs::msg::GnssPashr>("/Sensor_msgs/INS/PASHR", 10);
    
    /*声明参数*/
    this->declare_parameter("baudrate_INS", 921600);
    this->declare_parameter("INS_ComName", "/dev/insCOM0"); 
    this->declare_parameter("ins_frame_id", "/ins"); 
    
    /*获取参数*/
    this->get_parameter("baudrate_INS", baudrate_INS);
    this->get_parameter("INS_ComName", ComName);
    this->get_parameter("ins_frame_id", ins_frame_id);

    Byins_data.header.frame_id = ins_frame_id;
    ByinsIns_NavSatFix_data.header.frame_id = ins_frame_id;
    ByinsGnss_NavSatFix_data.header.frame_id = ins_frame_id;
    ByinsRaw_Imu_data.header.frame_id = ins_frame_id;
    ByinsJudged_Imu_data.header.frame_id = ins_frame_id;
    Gpdop_data.header.frame_id = ins_frame_id;
    Gpfpd_data.header.frame_id = ins_frame_id;
    Gpfpd_NavSatFix_data.header.frame_id = ins_frame_id;
    Gpgga_data.header.frame_id = ins_frame_id;
    Gpgga_NavSatFix_data.header.frame_id = ins_frame_id;
    Gpgst_data.header.frame_id = ins_frame_id;
    Gphdt_data.header.frame_id = ins_frame_id;
    Gpzda_data.header.frame_id = ins_frame_id;
    Pashr_data.header.frame_id = ins_frame_id;

    std::stringstream ss;
    ss.str("");
    ss << "\nbaudrate_INS = " << baudrate_INS;
    ss << "\nINS_ComName = " << ComName;
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());
    
    /*创建timeout*/
    serial::Timeout ComSerialTimeOut = serial::Timeout::simpleTimeout(100);
    
    /*设置需要打开的串口名字*/
    ComSerial.setPort(ComName);

    /*设置串口通信的波特率*/
    ComSerial.setBaudrate(baudrate_INS);
    
    /*串口设置timeout*/
    ComSerial.setTimeout(ComSerialTimeOut);

    /*打开串口*/
    try{ComSerial.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open INS port");}
      if(ComSerial.isOpen()){
        RCLCPP_INFO(this->get_logger(), "INS Port is opened. Serial name : %s", ComName.c_str());
      }else{
        RCLCPP_ERROR(this->get_logger(), "Open INS Port failed! Serial name : %s", ComName.c_str());}

    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        10ms, std::bind(&ins_com::read_serial, this));
  }

 private:
  /*设置串口基本参数*/
  int baudrate_INS,int_i,int_n,int_bridge;
  /*创建serial类*/
  serial::Serial ComSerial;
  /*设置计数器*/
  int intCount = 0;
  bool message_flag = false;
  // uint8_t buffer_uint8_t[14];
  std::string ins_frame_id;
  std::string str_buffer;
  std::string ComName;
  std::string ReadData;
  std::string start_split_str = "$";
  int start_split_str_len = start_split_str.length();
  std::string end_split_str = "\r\n";
  int end_split_str_len = end_split_str.length();
  std::string::size_type size_type_i, start, end;
  bool start_split_str_found;
  tf2::Quaternion orientation_tf;

  /*ROS2发布者、订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::InsByins>::SharedPtr Byins_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ByinsIns_NavSatFix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr ByinsGnss_NavSatFix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ByinsRaw_Imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ByinsJudged_Imu_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssGpdop>::SharedPtr Gpdop_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssGpfpd>::SharedPtr Gpfpd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr Gpfpd_NavSatFix_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssGpgga>::SharedPtr Gpgga_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr Gpgga_NavSatFix_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssGpgst>::SharedPtr Gpgst_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssGphdt>::SharedPtr Gphdt_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssGpzda>::SharedPtr Gpzda_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::GnssPashr>::SharedPtr Pashr_publisher_;
  std_msgs::msg::String raw_data;
  sensor_driver_msgs::msg::InsByins Byins_data;
  sensor_msgs::msg::NavSatFix ByinsIns_NavSatFix_data;
  sensor_msgs::msg::NavSatFix ByinsGnss_NavSatFix_data;
  sensor_msgs::msg::Imu ByinsRaw_Imu_data;
  sensor_msgs::msg::Imu ByinsJudged_Imu_data;
  sensor_driver_msgs::msg::GnssGpdop Gpdop_data;
  sensor_driver_msgs::msg::GnssGpfpd Gpfpd_data;
  sensor_msgs::msg::NavSatFix Gpfpd_NavSatFix_data;
  sensor_driver_msgs::msg::GnssGpgga Gpgga_data;
  sensor_msgs::msg::NavSatFix Gpgga_NavSatFix_data;
  sensor_driver_msgs::msg::GnssGpgst Gpgst_data;
  sensor_driver_msgs::msg::GnssGphdt Gphdt_data;
  sensor_driver_msgs::msg::GnssGpzda Gpzda_data;
  sensor_driver_msgs::msg::GnssPashr Pashr_data;

  void String_process(std::string s){
    raw_data.data = s;
    rawData_publisher_->publish(raw_data);
    RCLCPP_INFO(this->get_logger(),"origin data is : %s",s.c_str());

    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(",");
    pos1 = 0;
    while (pos2 != std::string::npos)
    {
      v.push_back(s.substr(pos1, pos2 - pos1));
      pos1 = pos2 + 1;
      pos2 = s.find(",",pos1);   
    }
    pos2 = s.find("*",pos1);
    v.push_back(s.substr(pos1, pos2 - pos1));
    pos1 = pos2 + 1;
    v.push_back(s.substr(pos1));

    /*定位BYINS消息类型，赋值发送相关信息*/
    if (v[0] == "BYINS")
    {
      RCLCPP_INFO(this->get_logger(), "Have received BYINS msg!");
      if (v.size() != 41)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Byins_data.data_id = v[0].c_str();
      Byins_data.serial_number = v[1].c_str();
      Byins_data.utc_time = atof(v[2].c_str());
      Byins_data.gps_week_sec = atof(v[3].c_str());
      Byins_data.ins_latitude = atof(v[4].c_str());
      Byins_data.ins_longitude = atof(v[5].c_str());
      Byins_data.ins_altitude = atof(v[6].c_str());
      Byins_data.yaw = atof(v[7].c_str());
      Byins_data.pitch = atof(v[8].c_str());
      Byins_data.roll = atof(v[9].c_str());
      Byins_data.velocity_front = atof(v[10].c_str());
      Byins_data.velocity_right = atof(v[11].c_str());
      Byins_data.velocity_up = atof(v[12].c_str());
      Byins_data.accelerate_right = atof(v[13].c_str());
      Byins_data.accelerate_front = atof(v[14].c_str());
      Byins_data.accelerate_up = atof(v[15].c_str());
      Byins_data.angular_velocity_right_origin = atof(v[16].c_str());
      Byins_data.angular_velocity_front_origin = atof(v[17].c_str());
      Byins_data.angular_velocity_up_origin = atof(v[18].c_str());
      Byins_data.angular_velocity_right_judged = atof(v[19].c_str());
      Byins_data.angular_velocity_front_judged = atof(v[20].c_str());
      Byins_data.angular_velocity_up_judged = atof(v[21].c_str());
      Byins_data.ins_state = atoi(v[22].c_str());
      Byins_data.gnss_vector_state = atoi(v[23].c_str());
      Byins_data.satellite_num = atoi(v[24].c_str());
      Byins_data.rtk_delay = atof(v[25].c_str());
      Byins_data.bk_1 = atof(v[26].c_str());
      Byins_data.bk_2 = atof(v[27].c_str());
      Byins_data.bk_3 = atof(v[28].c_str());
      Byins_data.accelerate_north = atof(v[29].c_str());
      Byins_data.accelerate_east = atof(v[30].c_str());
      Byins_data.accelerate_ground = atof(v[31].c_str());
      Byins_data.gnss_latitude = atof(v[32].c_str());
      Byins_data.gnss_longitude = atof(v[33].c_str());
      Byins_data.gnss_altitude = atof(v[34].c_str());
      Byins_data.gnss_location_state = atoi(v[35].c_str());
      Byins_data.error = atoi(v[36].c_str());
      Byins_data.velocity_east = atof(v[37].c_str());
      Byins_data.velocity_north = atof(v[38].c_str());
      Byins_data.velocity_sky = atof(v[39].c_str());
      Byins_data.cs = v[40].c_str();

      Byins_data.header.stamp = this->get_clock()->now();
      Byins_publisher_->publish(Byins_data);

      orientation_tf.setRPY(Byins_data.roll,Byins_data.pitch,Byins_data.yaw);

      ByinsRaw_Imu_data.orientation.x = orientation_tf.x();
      ByinsRaw_Imu_data.orientation.y = orientation_tf.y();
      ByinsRaw_Imu_data.orientation.z = orientation_tf.z();
      ByinsRaw_Imu_data.orientation.w = orientation_tf.w();
      ByinsRaw_Imu_data.orientation_covariance = {-1,0,0,0,0,0,0,0,0};
      ByinsRaw_Imu_data.angular_velocity.x = Byins_data.angular_velocity_front_origin;
      ByinsRaw_Imu_data.angular_velocity.y = Byins_data.angular_velocity_right_origin;
      ByinsRaw_Imu_data.angular_velocity.z = Byins_data.angular_velocity_up_origin;
      ByinsRaw_Imu_data.angular_velocity_covariance = {-1,0,0,0,0,0,0,0,0};
      ByinsRaw_Imu_data.linear_acceleration.x = Byins_data.accelerate_front;
      ByinsRaw_Imu_data.linear_acceleration.y = Byins_data.accelerate_right;
      ByinsRaw_Imu_data.linear_acceleration.z = Byins_data.accelerate_up;
      ByinsRaw_Imu_data.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

      ByinsRaw_Imu_data.header.stamp = this->get_clock()->now();
      ByinsRaw_Imu_publisher_->publish(ByinsRaw_Imu_data);

      ByinsJudged_Imu_data.orientation.x = orientation_tf.x();
      ByinsJudged_Imu_data.orientation.y = orientation_tf.y();
      ByinsJudged_Imu_data.orientation.z = orientation_tf.z();
      ByinsJudged_Imu_data.orientation.w = orientation_tf.w();
      ByinsJudged_Imu_data.orientation_covariance = {-1,0,0,0,0,0,0,0,0};
      ByinsJudged_Imu_data.angular_velocity.x = Byins_data.angular_velocity_front_judged;
      ByinsJudged_Imu_data.angular_velocity.y = Byins_data.angular_velocity_right_judged;
      ByinsJudged_Imu_data.angular_velocity.z = Byins_data.angular_velocity_up_judged;
      ByinsJudged_Imu_data.angular_velocity_covariance = {-1,0,0,0,0,0,0,0,0};
      ByinsJudged_Imu_data.linear_acceleration.x = Byins_data.accelerate_front;
      ByinsJudged_Imu_data.linear_acceleration.y = Byins_data.accelerate_right;
      ByinsJudged_Imu_data.linear_acceleration.z = Byins_data.accelerate_up;
      ByinsJudged_Imu_data.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

      ByinsJudged_Imu_data.header.stamp = this->get_clock()->now();
      ByinsJudged_Imu_publisher_->publish(ByinsJudged_Imu_data);

      if(Byins_data.ins_state == 0){
        ByinsIns_NavSatFix_data.status.status = -1;
      }else if(Byins_data.ins_state == 1){
        ByinsIns_NavSatFix_data.status.status = 0;
      }else if(Byins_data.ins_state == 2){
        ByinsIns_NavSatFix_data.status.status = 1;
      }else if(Byins_data.ins_state == 4){
        ByinsIns_NavSatFix_data.status.status = 1;
      }else if(Byins_data.ins_state == 5){
        ByinsIns_NavSatFix_data.status.status = 2;
      }else if(Byins_data.ins_state == 6){
        ByinsIns_NavSatFix_data.status.status = 2;
      }
      ByinsIns_NavSatFix_data.status.service = 4;
      ByinsIns_NavSatFix_data.latitude = Byins_data.ins_latitude;
      ByinsIns_NavSatFix_data.longitude = Byins_data.ins_longitude;
      ByinsIns_NavSatFix_data.altitude = Byins_data.ins_altitude;
      ByinsIns_NavSatFix_data.position_covariance_type = 0;
      
      ByinsIns_NavSatFix_data.header.stamp = this->get_clock()->now();
      ByinsIns_NavSatFix_publisher_->publish(ByinsIns_NavSatFix_data);

      if(Byins_data.gnss_location_state == 0){
        ByinsGnss_NavSatFix_data.status.status = -1;
      }else if(Byins_data.gnss_location_state == 1){
        ByinsGnss_NavSatFix_data.status.status = 0;
      }else if(Byins_data.gnss_location_state == 2){
        ByinsGnss_NavSatFix_data.status.status = 1;
      }else if(Byins_data.gnss_location_state == 4){
        ByinsGnss_NavSatFix_data.status.status = 1;
      }else if(Byins_data.gnss_location_state == 5){
        ByinsGnss_NavSatFix_data.status.status = 2;
      }
      ByinsGnss_NavSatFix_data.status.service = 4;
      ByinsGnss_NavSatFix_data.latitude = Byins_data.gnss_latitude;
      ByinsGnss_NavSatFix_data.longitude = Byins_data.gnss_longitude;
      ByinsGnss_NavSatFix_data.altitude = Byins_data.gnss_altitude;
      ByinsGnss_NavSatFix_data.position_covariance_type = 0;

      ByinsGnss_NavSatFix_data.header.stamp = this->get_clock()->now();
      ByinsGnss_NavSatFix_publisher_->publish(ByinsGnss_NavSatFix_data);

    }else
    /*定位GPDOP消息类型，赋值发送相关信息*/
    if (v[0] == "GPDOP"){
      RCLCPP_INFO(this->get_logger(), "Have received GPDOP msg!");
      if (v.size() != 8)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Gpdop_data.data_id = v[0].c_str();
      Gpdop_data.utc_time = atof(v[1].c_str());
      Gpdop_data.pdop = atof(v[2].c_str());
      Gpdop_data.hdop = atof(v[3].c_str());
      Gpdop_data.vdop = atof(v[4].c_str());
      Gpdop_data.tdop = atof(v[5].c_str());
      Gpdop_data.gdop = atof(v[6].c_str());
      Gpdop_data.cs = v[7].c_str();

      Gpdop_data.header.stamp = this->get_clock()->now();
      Gpdop_publisher_->publish(Gpdop_data);
    }else
    /*定位GPFPD消息类型，赋值发送相关信息*/
    if (v[0] == "GPFPD"){
      RCLCPP_INFO(this->get_logger(), "Have received GPFPD msg!");
      if (v.size() != 17)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Gpfpd_data.data_id = v[0].c_str();
      Gpfpd_data.gps_week = atoi(v[1].c_str());
      Gpfpd_data.gps_time = atof(v[2].c_str());
      Gpfpd_data.yaw = atof(v[3].c_str());
      Gpfpd_data.pitch = atof(v[4].c_str());
      Gpfpd_data.roll = atof(v[5].c_str());
      Gpfpd_data.latitude = atof(v[6].c_str());
      Gpfpd_data.longitude = atof(v[7].c_str());
      Gpfpd_data.altitude = atof(v[8].c_str());
      Gpfpd_data.velocity_east = atof(v[9].c_str());
      Gpfpd_data.velocity_north = atof(v[10].c_str());
      Gpfpd_data.velocity_sky = atof(v[11].c_str());
      Gpfpd_data.base_line = atof(v[12].c_str());
      Gpfpd_data.satellite_num_1 = atoi(v[13].c_str());
      Gpfpd_data.satellite_num_2 = atoi(v[14].c_str());
      Gpfpd_data.state = atoi(v[15].c_str());
      Gpfpd_data.cs = v[16].c_str();

      Gpfpd_data.header.stamp = this->get_clock()->now();
      Gpfpd_publisher_->publish(Gpfpd_data);

      if(Gpfpd_data.state == 0){
        Gpfpd_NavSatFix_data.status.status = -1;
      }else if(Gpfpd_data.state == 1){
        Gpfpd_NavSatFix_data.status.status = 1;
      }else if(Gpfpd_data.state == 2){
        Gpfpd_NavSatFix_data.status.status = 0;
      }else if(Gpfpd_data.state == 3){
        Gpfpd_NavSatFix_data.status.status = -1;
      }else if(Gpfpd_data.state == 11){
        Gpfpd_NavSatFix_data.status.status = 2;
      }else if(Gpfpd_data.state == 12){
        Gpfpd_NavSatFix_data.status.status = 2;
      }
      Gpfpd_NavSatFix_data.status.service = 4;
      Gpfpd_NavSatFix_data.latitude = Gpfpd_data.latitude;
      Gpfpd_NavSatFix_data.longitude = Gpfpd_data.longitude;
      Gpfpd_NavSatFix_data.altitude = Gpfpd_data.altitude;
      Gpfpd_NavSatFix_data.position_covariance = {-1,0,0,0,0,0,0,0,0};
      Gpfpd_NavSatFix_data.position_covariance_type = 0;

      Gpfpd_NavSatFix_data.header.stamp = this->get_clock()->now();
      Gpfpd_NavSatFix_publisher_->publish(Gpfpd_NavSatFix_data);
    }else
    /*定位GPGGA消息类型，赋值发送相关信息*/
    if (v[0] == "GPGGA"){
      RCLCPP_INFO(this->get_logger(), "Have received GPGGA msg!");
      if (v.size() != 16)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Gpgga_data.data_id = v[0].c_str();
      Gpgga_data.utc_time = atof(v[1].c_str());
      Gpgga_data.latitude = atof(v[2].c_str());
      Gpgga_data.latitude_sign = v[3].c_str();
      Gpgga_data.longitude = atof(v[4].c_str());
      Gpgga_data.longitude_sign = v[5].c_str();
      Gpgga_data.state = atoi(v[6].c_str());
      Gpgga_data.satellite_num = atoi(v[7].c_str());
      Gpgga_data.hdop = atof(v[8].c_str());
      Gpgga_data.altitude = atof(v[9].c_str());
      Gpgga_data.altitude_sign = v[10].c_str();
      Gpgga_data.altitude_diff = atof(v[11].c_str());
      Gpgga_data.altitude_diff_sign = v[12].c_str();
      Gpgga_data.rtk_delay = atof(v[13].c_str());
      Gpgga_data.rtk_id = atoi(v[14].c_str());
      Gpgga_data.cs = v[15].c_str();

      Gpgga_data.header.stamp = this->get_clock()->now();
      Gpgga_publisher_->publish(Gpgga_data);

      if(Gpgga_data.state == 0){
        Gpgga_NavSatFix_data.status.status = -1;
      }else if(Gpgga_data.state == 1){
        Gpgga_NavSatFix_data.status.status = 0;
      }else if(Gpgga_data.state == 2){
        Gpgga_NavSatFix_data.status.status = 1;
      }else if(Gpgga_data.state == 4){
        Gpgga_NavSatFix_data.status.status = 1;
      }else if(Gpgga_data.state == 5){
        Gpgga_NavSatFix_data.status.status = 2;
      }
      Gpgga_NavSatFix_data.status.service = 4;
      if(Gpgga_data.latitude_sign=="N"){
        Gpgga_NavSatFix_data.latitude = Gpgga_data.latitude;
      }else if(Gpgga_data.latitude_sign=="S"){
        Gpgga_NavSatFix_data.latitude = -1*Gpgga_data.latitude;
      }
      if(Gpgga_data.longitude_sign=="E"){
        Gpgga_NavSatFix_data.longitude = Gpgga_data.longitude;
      }else if(Gpgga_data.longitude_sign=="W"){
        Gpgga_NavSatFix_data.longitude = -1*Gpgga_data.longitude;
      }
      Gpgga_NavSatFix_data.altitude = Gpgga_data.altitude;
      Gpgga_NavSatFix_data.position_covariance = {-1,0,0,0,0,0,0,0,0};
      Gpgga_NavSatFix_data.position_covariance_type = 0;

      Gpgga_NavSatFix_data.header.stamp = this->get_clock()->now();
      Gpgga_NavSatFix_publisher_->publish(Gpgga_NavSatFix_data);
    }else
    /*定位GPGST消息类型，赋值发送相关信息*/
    if (v[0] == "GPGST"){
      RCLCPP_INFO(this->get_logger(), "Have received GPGST msg!");
      if (v.size() != 10)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Gpgst_data.data_id = v[0].c_str();
      Gpgst_data.utc_time = atof(v[1].c_str());
      Gpgst_data.rms_std = atof(v[2].c_str());
      Gpgst_data.smjr_std = atof(v[3].c_str());
      Gpgst_data.smnr_std = atof(v[4].c_str());
      Gpgst_data.orient = atof(v[5].c_str());
      Gpgst_data.lat_std = atof(v[6].c_str());
      Gpgst_data.lon_std = atof(v[7].c_str());
      Gpgst_data.alt_std = atof(v[8].c_str());
      Gpgst_data.cs = v[9].c_str();

      Gpgst_data.header.stamp = this->get_clock()->now();
      Gpgst_publisher_->publish(Gpgst_data);
    }else
    /*定位GPHDT消息类型，赋值发送相关信息*/
    if (v[0] == "GPHDT"){
      RCLCPP_INFO(this->get_logger(), "Have received GPHDT msg!");
      if (v.size() != 4)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Gphdt_data.data_id = v[0].c_str();
      Gphdt_data.yaw = atof(v[1].c_str());
      Gphdt_data.t_sign = v[2].c_str();
      Gphdt_data.cs = v[3].c_str();

      Gphdt_data.header.stamp = this->get_clock()->now();
      Gphdt_publisher_->publish(Gphdt_data);
    }else
    /*定位GPZDA消息类型，赋值发送相关信息*/
    if (v[0] == "GPZDA"){
      RCLCPP_INFO(this->get_logger(), "Have received GPZDA msg!");
      if (v.size() != 8)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Gpzda_data.data_id = v[0].c_str();
      Gpzda_data.utc_time = atof(v[1].c_str());
      Gpzda_data.time_day = atoi(v[2].c_str());
      Gpzda_data.time_month = atoi(v[3].c_str());
      Gpzda_data.time_year = atoi(v[4].c_str());
      Gpzda_data.time_zone = atoi(v[5].c_str());
      Gpzda_data.time_diff = atoi(v[6].c_str());
      Gpzda_data.cs = v[7].c_str();

      Gpzda_data.header.stamp = this->get_clock()->now();
      Gpzda_publisher_->publish(Gpzda_data);
    }else
    /*定位PASHR消息类型，赋值发送相关信息*/
    if (v[0] == "PASHR"){
      RCLCPP_INFO(this->get_logger(), "Have received PASHR msg!");
      if (v.size() != 12)
      {
        RCLCPP_ERROR(this->get_logger(), "Split ERROR!");
        return;
      }
      Pashr_data.data_id = v[0].c_str();
      Pashr_data.utc_time = atof(v[1].c_str());
      Pashr_data.yaw = atof(v[2].c_str());
      Pashr_data.t_sign = v[3].c_str();
      Pashr_data.roll = atof(v[4].c_str());
      Pashr_data.pitch = atof(v[5].c_str());
      Pashr_data.height_error = atof(v[6].c_str());
      Pashr_data.roll_std = atof(v[7].c_str());
      Pashr_data.pitch_std = atof(v[8].c_str());
      Pashr_data.yaw_std = atof(v[9].c_str());
      Pashr_data.state = atoi(v[10].c_str());
      Pashr_data.cs = v[11].c_str();

      Pashr_data.header.stamp = this->get_clock()->now();
      Pashr_publisher_->publish(Pashr_data);
    }
  }

  void read_serial() {
    /*检查是否有IMU数据*/
    if(ComSerial.available()){
      /*打印收到的数据*/
      std::cout << ComSerial.available();
      str_buffer += ComSerial.read(ComSerial.available());
      //std::cout <<"str_buffer:" << str_buffer << "\n" ;
    }
    // RCLCPP_INFO(this->get_logger(),"The str buffer length is : %d",str_buffer.length());
    //2.截取数据、解析数据：
    if (start_split_str_found == false)
    { 
      start = str_buffer.find(start_split_str);
      if (start == std::string::npos)
      {
          // RCLCPP_INFO(this->get_logger(), "Haven't find start_split_str");
          start_split_str_found = false;
          return;
      }
      else
      {
          // RCLCPP_INFO(this->get_logger(), "Have found start_split_str, Search for the end_split_str");
          start_split_str_found = true;
      }
    }
    
    size_type_i = start + start_split_str_len;
    while (size_type_i < str_buffer.length())
    {
      //找终止标志
      end = str_buffer.find(end_split_str, size_type_i);
      //如果没找到，把起始标志开始的数据留下，前面的数据丢弃，然后跳出循环
      if (end == std::string::npos)
      {
          // RCLCPP_INFO(this->get_logger(), "Haven't find end_split_str, Save data from start_split_str");
          str_buffer = str_buffer.substr(start);
          start = 0;
          return;
      }
      //如果找到了终止标志，把这段有效的数据剪切给解析的函数，剩下的继续开始寻找
      else
      {
        // RCLCPP_INFO(this->get_logger(), "Have found end_split_str, Assign end to start, continue to search for end_split_str");
        //把有效的数据给解析的函数以获取经纬度
        ins_com::String_process(str_buffer.substr(size_type_i, end - size_type_i));
        start = end + end_split_str_len;
        str_buffer = str_buffer.substr(start);
        start_split_str_found = false;
        size_type_i = start + start_split_str_len;
        // RCLCPP_INFO(this->get_logger(), "size_type_i = %d , length of str _buffer = %d",size_type_i,str_buffer.length());
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ins_com>("ins_com_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
