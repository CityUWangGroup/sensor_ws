#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cstdio>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/*驱动底盘CAN总线协议反馈端（含有反馈信息发布和订阅控制指令功能）*/
class radar_can : public rclcpp::Node{
 public:

  // 构造函数,有一个参数为节点名称
  explicit radar_can(std::string name) : Node(name){

    /*创建发布函数*/
    rawdata_publisher_ = this->create_publisher<std_msgs::msg::String>("/Sensor_msgs/radar_can/rawdata", 10);

    /*声明参数*/
    this->declare_parameter("radar_can_name", "can1");
    
    /*获取参数*/
    this->get_parameter("radar_can_name", CAN_name);

    /*初始化CAN通道*/
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    strcpy(ifr.ifr_name, CAN_name.c_str() );
    ioctl(s, SIOCGIFINDEX, &ifr); //指定 can1 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can1 绑定

    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        1ms, std::bind(&radar_can::data_callback, this));
  }

 private:
  /*声明CAN相关变量*/
  int s, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame;
  std::string CAN_name;
  std::vector<can_frame> can_vec;
  
  /*声明时间循环、发布者和订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawdata_publisher_;
  std_msgs::msg::String rawdata;

  /*CAN报文信息原始值*/
    /*控制部分*/

    /*反馈部分*/

  void data_callback() {
    nbytes = read(s, &frame, sizeof(frame)); //接收报文
    //报文解析
    if(nbytes > 0){
      if(frame.can_id == 0x352){
      }
      else if(frame.can_id == 0x354){
        /*发布车辆运行速度信息*/
        rawdata.data = "s";
        rawdata_publisher_->publish(rawdata);
      }
      else if(frame.can_id == 0x356){
      }
      else if(frame.can_id == 0x358){
      }
      else if(frame.can_id == 0x360){
      }
      else if(frame.can_id == 0x362){
      }
      else if(frame.can_id == 0x364){
      }
      RCLCPP_INFO(this->get_logger(), "0");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<radar_can>("radar_can_driver_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}