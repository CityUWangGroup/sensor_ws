#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <chrono>

/*驱动六台UART自动发布的超声波雷达中转机*/
class Ultrasonic_radar_com : public rclcpp::Node{
 public:
  // 构造函数,有一个参数为节点名称
  explicit Ultrasonic_radar_com(std::string name) : Node(name){
    
    /*声明参数*/
    this->declare_parameter("baudrate_UltrasonicRadar", 9600);
    this->declare_parameter("UltrasonicRadar3_ComName", "/dev/ultraCOM5"); 
    this->declare_parameter("Publish_topic_17", "/Sensor_msgs/UltrasonicRadar/COM17"); 
    this->declare_parameter("Publish_topic_18", "/Sensor_msgs/UltrasonicRadar/COM18"); 
    this->declare_parameter("Publish_topic_19", "/Sensor_msgs/UltrasonicRadar/COM19"); 
    this->declare_parameter("Publish_topic_20", "/Sensor_msgs/UltrasonicRadar/COM20"); 

    /*获取参数*/
    this->get_parameter("baudrate_UltrasonicRadar", baudrate_UltrasonicRadar);
    this->get_parameter("UltrasonicRadar3_ComName", ComName);
    this->get_parameter("Publish_topic_17", Publish_topic_17);
    this->get_parameter("Publish_topic_18", Publish_topic_18);
    this->get_parameter("Publish_topic_19", Publish_topic_19);
    this->get_parameter("Publish_topic_20", Publish_topic_20);
    
    // 创建发布
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>("/Sensor_msgs/UltrasonicRadar/COM5_rawData", 10);
    ultrasonicRadarCOM_a_publisher_ = this->create_publisher<std_msgs::msg::Int16>(Publish_topic_17, 10);
    ultrasonicRadarCOM_b_publisher_ = this->create_publisher<std_msgs::msg::Int16>(Publish_topic_18, 10);
    ultrasonicRadarCOM_c_publisher_ = this->create_publisher<std_msgs::msg::Int16>(Publish_topic_19, 10);
    ultrasonicRadarCOM_d_publisher_ = this->create_publisher<std_msgs::msg::Int16>(Publish_topic_20, 10);
    
    std::stringstream ss;
    ss.str("");
    ss << "\nnode_name = " << name;
    ss << "\nbaudrate_UltrasonicRadar = " << baudrate_UltrasonicRadar;
    ss << "\nComName3 = " << ComName;
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());
    
    /*创建timeout*/
    serial::Timeout ComSerialTimeOut = serial::Timeout::simpleTimeout(100);
    
    /*设置需要打开的串口名字*/
    ComSerial.setPort(ComName);

    /*设置串口通信的波特率*/
    ComSerial.setBaudrate(baudrate_UltrasonicRadar);
    
    /*串口设置timeout*/
    ComSerial.setTimeout(ComSerialTimeOut);
    // bool start_figure = true ;

    /*打开串口*/
    try{ComSerial.open();}
    catch(serial::IOException& e){
      RCLCPP_ERROR(this->get_logger(), "Unable to open port 6");}
    if(ComSerial.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Port 6 is opened. Serial name : %s", ComName.c_str());
    }else{
      RCLCPP_ERROR(this->get_logger(), "Open Port 6 failed! Serial name : %s", ComName.c_str());}
    
    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        5ms, std::bind(&Ultrasonic_radar_com::read_serial, this));
  }

 private:
  /*设置串口基本参数*/
  int baudrate_UltrasonicRadar,int_i,int_n,int_bridge;
  /*创建serial类*/
  serial::Serial ComSerial;
  /*设置计数器*/
  int intCount = 0;
  bool message_flag = false;
  uint8_t buffer_uint8_t[14];
  uint16_t temper_data_record[4];
  std::string ComName;
  std::string Publish_topic_17,Publish_topic_18,Publish_topic_19,Publish_topic_20;

  /*ROS2发布者、订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ultrasonicRadarCOM_a_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ultrasonicRadarCOM_b_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ultrasonicRadarCOM_c_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ultrasonicRadarCOM_d_publisher_;
  std_msgs::msg::String origin_data;
  std_msgs::msg::Int16 data_a,data_b,data_c,data_d;

  void read_serial() {
    size_t n6 = ComSerial.available();
    /*第一个中转盒*/
    if(n6!=0){
      for(int_i=0;int_i<14;int_i++){
        buffer_uint8_t[int_i] = 0;
      }
      n6 = ComSerial.read(buffer_uint8_t, n6);
      int_n = int(n6);
      intCount = 0;
      origin_data.data = "n6: " ;
      for(int_i = 0; int_i < int_n; int_i++){
        int_bridge = buffer_uint8_t[int_i];
        origin_data.data = origin_data.data + std::to_string(int_bridge) + " ";
        intCount = intCount + 1;
      }
      // RCLCPP_INFO(this->get_logger(), origin_data.data.c_str());
      rawData_publisher_->publish(origin_data);
      if(intCount==10){
        for(int_i=0;int_i<4;int_i++){
          temper_data_record[int_i] = buffer_uint8_t[int_i*2+1]*0x100+buffer_uint8_t[int_i*2+2];
        }
        message_flag = true;
      }
    }
    /*检测是否有新数据，并发布相关话题*/
    if(message_flag){
      data_a.data = temper_data_record[0];
      data_b.data = temper_data_record[1];
      data_c.data = temper_data_record[2];
      data_d.data = temper_data_record[3];

      ultrasonicRadarCOM_a_publisher_->publish(data_a);
      ultrasonicRadarCOM_b_publisher_->publish(data_b);
      ultrasonicRadarCOM_c_publisher_->publish(data_c);
      ultrasonicRadarCOM_d_publisher_->publish(data_d);
      message_flag = false;
    }
  }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<Ultrasonic_radar_com>("ultrasonicRadarCom6_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
