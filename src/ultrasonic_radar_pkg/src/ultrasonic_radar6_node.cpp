#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_driver_msgs/msg/ultrasonic_radar20.hpp"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <chrono>

/*驱动六台UART自动发布的超声波雷达中转机*/
class Ultrasonic_radar6 : public rclcpp::Node{
 public:
  // 构造函数,有一个参数为节点名称
  explicit Ultrasonic_radar6(std::string name) : Node(name){
    // 创建发布
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>("/Sensor_msgs/UltrasonicRadar/rawData", 10);
    ultrasonicRadarData20_publisher_ = this->create_publisher<sensor_driver_msgs::msg::UltrasonicRadar20>("/Sensor_msgs/UltrasonicRadar/UltrasonicRadar20", 10);
    
    /*声明参数*/
    this->declare_parameter("baudrate_UltrasonicRadar", 9600);
    this->declare_parameter("UltrasonicRadar1_ComName", "/dev/ultraCOM0"); 
    this->declare_parameter("UltrasonicRadar2_ComName", "/dev/ultraCOM1"); 
    this->declare_parameter("UltrasonicRadar3_ComName", "/dev/ultraCOM2"); 
    this->declare_parameter("UltrasonicRadar4_ComName", "/dev/ultraCOM3"); 
    this->declare_parameter("UltrasonicRadar5_ComName", "/dev/ultraCOM4"); 
    this->declare_parameter("UltrasonicRadar6_ComName", "/dev/ultraCOM5"); 
    
    /*获取参数*/
    this->get_parameter("baudrate_UltrasonicRadar", baudrate_UltrasonicRadar);
    this->get_parameter("UltrasonicRadar1_ComName", ComName1);
    this->get_parameter("UltrasonicRadar2_ComName", ComName2);
    this->get_parameter("UltrasonicRadar3_ComName", ComName3);
    this->get_parameter("UltrasonicRadar4_ComName", ComName4);
    this->get_parameter("UltrasonicRadar5_ComName", ComName5);
    this->get_parameter("UltrasonicRadar6_ComName", ComName6);
    std::stringstream ss;
    ss.str("");
    ss << "\nbaudrate_UltrasonicRadar = " << baudrate_UltrasonicRadar;
    ss << "\nComName1 = " << ComName1;
    ss << "\nComName2 = " << ComName2;
    ss << "\nComName3 = " << ComName3;
    ss << "\nComName4 = " << ComName4;
    ss << "\nComName5 = " << ComName5;
    ss << "\nComName6 = " << ComName6;
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());
    
    /*创建timeout*/
    serial::Timeout ComSerialTimeOut = serial::Timeout::simpleTimeout(100);
    
    /*设置需要打开的串口名字*/
    ComSerial1.setPort(ComName1);
    ComSerial2.setPort(ComName2);
    ComSerial3.setPort(ComName3);
    ComSerial4.setPort(ComName4);
    ComSerial5.setPort(ComName5);
    ComSerial6.setPort(ComName6);

    /*设置串口通信的波特率*/
    ComSerial1.setBaudrate(baudrate_UltrasonicRadar);
    ComSerial2.setBaudrate(baudrate_UltrasonicRadar);
    ComSerial3.setBaudrate(baudrate_UltrasonicRadar);
    ComSerial4.setBaudrate(baudrate_UltrasonicRadar);
    ComSerial5.setBaudrate(baudrate_UltrasonicRadar);
    ComSerial6.setBaudrate(baudrate_UltrasonicRadar);
    
    /*串口设置timeout*/
    ComSerial1.setTimeout(ComSerialTimeOut);
    ComSerial2.setTimeout(ComSerialTimeOut);
    ComSerial3.setTimeout(ComSerialTimeOut);
    ComSerial4.setTimeout(ComSerialTimeOut);
    ComSerial5.setTimeout(ComSerialTimeOut);
    ComSerial6.setTimeout(ComSerialTimeOut);
    // bool start_figure = true ;

    /*打开串口*/
    try{ComSerial1.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 1");}
    if(ComSerial1.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 1 is opened. Serial name : %s", ComName1.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 1 failed! Serial name : %s", ComName1.c_str());}
    
    try{ComSerial2.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 2");}
    if(ComSerial2.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 2 is opened. Serial name : %s", ComName2.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 2 failed! Serial name : %s", ComName2.c_str());}

    try{ComSerial3.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 3");}
    if(ComSerial3.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 3 is opened. Serial name : %s", ComName3.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 3 failed! Serial name : %s", ComName3.c_str());}

    try{ComSerial4.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 4");}
    if(ComSerial4.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 4 is opened. Serial name : %s", ComName4.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 4 failed! Serial name : %s", ComName4.c_str());}

    try{ComSerial5.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 5");}
    if(ComSerial5.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 5 is opened. Serial name : %s", ComName5.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 5 failed! Serial name : %s", ComName5.c_str());}

    try{ComSerial6.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 6");}
    if(ComSerial6.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 6 is opened. Serial name : %s", ComName6.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 6 failed! Serial name : %s", ComName6.c_str());}

    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        5ms, std::bind(&Ultrasonic_radar6::read_serial, this));
  }

 private:
  /*设置串口基本参数*/
  int baudrate_UltrasonicRadar,int_i,int_n,int_bridge;
  /*创建serial类*/
  serial::Serial ComSerial1;
  serial::Serial ComSerial2;
  serial::Serial ComSerial3;
  serial::Serial ComSerial4;
  serial::Serial ComSerial5;
  serial::Serial ComSerial6;
  /*设置计数器*/
  int intCount = 0;
  bool message_flag = false;
  uint8_t buffer_uint8_t[14];
  uint16_t temper_data_record[20];
  std::string ComName1,ComName2,ComName3,ComName4,ComName5,ComName6;

  /*ROS2发布者、订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::UltrasonicRadar20>::SharedPtr ultrasonicRadarData20_publisher_;
  std_msgs::msg::String origin_data;
  sensor_driver_msgs::msg::UltrasonicRadar20 ultrasonicRadarData20;

  void read_serial() {
    size_t n1 = ComSerial1.available();
    size_t n2 = ComSerial2.available();
    size_t n3 = ComSerial3.available();
    size_t n4 = ComSerial4.available();
    size_t n5 = ComSerial5.available();
    size_t n6 = ComSerial6.available();
    /*第一个中转盒*/
    if(n1!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n1 = ComSerial1.read(buffer_uint8_t, n1);
      int_n = int(n1);
      intCount = 0;
      origin_data.data = "n1: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<3;int_i++){
          temper_data_record[int_i+0] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*第二个中转盒*/
    if(n2!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n2 = ComSerial1.read(buffer_uint8_t, n2);
      int_n = int(n2);
      intCount = 0;
      origin_data.data = "n2: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<3;int_i++){
          temper_data_record[int_i+3] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*第三个中转盒*/
    if(n3!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n3 = ComSerial1.read(buffer_uint8_t, n3);
      int_n = int(n3);
      intCount = 0;
      origin_data.data = "n3: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<4;int_i++){
          temper_data_record[int_i+6] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*第四个中转盒*/
    if(n4!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n4 = ComSerial1.read(buffer_uint8_t, n4);
      int_n = int(n4);
      intCount = 0;
      origin_data.data = "n4: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<3;int_i++){
          temper_data_record[int_i+10] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*第五个中转盒*/
    if(n5!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n5 = ComSerial1.read(buffer_uint8_t, n5);
      int_n = int(n5);
      intCount = 0;
      origin_data.data = "n5: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<3;int_i++){
          temper_data_record[int_i+13] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*第六个中转盒*/
    if(n6!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n6 = ComSerial1.read(buffer_uint8_t, n6);
      int_n = int(n6);
      intCount = 0;
      origin_data.data = "n6: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<4;int_i++){
          temper_data_record[int_i+16] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*检测是否有新数据，并发布相关话题*/
    if(message_flag){
      ultrasonicRadarData20.ultrasonic_01 = temper_data_record[0];
      ultrasonicRadarData20.ultrasonic_02 = temper_data_record[1];
      ultrasonicRadarData20.ultrasonic_03 = temper_data_record[2];
      ultrasonicRadarData20.ultrasonic_04 = temper_data_record[3];
      ultrasonicRadarData20.ultrasonic_05 = temper_data_record[4];
      ultrasonicRadarData20.ultrasonic_06 = temper_data_record[5];
      ultrasonicRadarData20.ultrasonic_07 = temper_data_record[6];
      ultrasonicRadarData20.ultrasonic_08 = temper_data_record[7];
      ultrasonicRadarData20.ultrasonic_09 = temper_data_record[8];
      ultrasonicRadarData20.ultrasonic_10 = temper_data_record[9];
      ultrasonicRadarData20.ultrasonic_11 = temper_data_record[10];
      ultrasonicRadarData20.ultrasonic_12 = temper_data_record[11];
      ultrasonicRadarData20.ultrasonic_13 = temper_data_record[12];
      ultrasonicRadarData20.ultrasonic_14 = temper_data_record[13];
      ultrasonicRadarData20.ultrasonic_15 = temper_data_record[14];
      ultrasonicRadarData20.ultrasonic_16 = temper_data_record[15];
      ultrasonicRadarData20.ultrasonic_17 = temper_data_record[16];
      ultrasonicRadarData20.ultrasonic_18 = temper_data_record[17];
      ultrasonicRadarData20.ultrasonic_19 = temper_data_record[18];
      ultrasonicRadarData20.ultrasonic_20 = temper_data_record[19];

      ultrasonicRadarData20.header.stamp = this->get_clock()->now();
      ultrasonicRadarData20_publisher_->publish(ultrasonicRadarData20);
      message_flag = false;
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<Ultrasonic_radar6>("ultrasonicRadarAll_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
