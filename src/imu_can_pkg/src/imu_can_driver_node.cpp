#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_driver_msgs/msg/imu_hwt9073can.hpp"
#include <tf2/LinearMath/Quaternion.h>
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

/*驱动一台维特智能HWT9073-485惯性测量元件*/
class imu_can : public rclcpp::Node{
 public:

  // 构造函数,有一个参数为节点名称
  explicit imu_can(std::string name) : Node(name){

    /*创建发布函数*/
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>("/Sensor_msgs/IMU_Lidar/rawData", 10);
    ImuData_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/Sensor_msgs/IMU_Lidar/Imu", 10);
    ImuHwtData_publisher_ = this->create_publisher<sensor_driver_msgs::msg::ImuHwt9073can>("/Sensor_msgs/IMU_Lidar/ImuHwt9073can", 10);
    
    /*声明参数*/
    this->declare_parameter("imu_can_name", "can0"); 
    this->declare_parameter("gravity_acc", 9.8); 
    this->declare_parameter("imu_frame_id", "/lidar_front/imu"); 
    
    /*获取参数*/
    this->get_parameter("imu_can_name", CAN_name);
    this->get_parameter("gravity_acc", gravity_acc);
    this->get_parameter("imu_frame_id", imu_frame_id);
    Imu_data.header.frame_id = imu_frame_id;
    ImuHwt_data.header.frame_id = imu_frame_id;
    std::stringstream ss;
    ss.str("");
    ss << "\nimu_can_name = " << CAN_name;
    ss << "\ngravity_acc = " << gravity_acc << " m/s^2";
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());

    /*初始化CAN通道*/
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    strcpy(ifr.ifr_name, CAN_name.c_str() );
    ioctl(s, SIOCGIFINDEX, &ifr); //指定 can0 设备
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can0 绑定

    /*启动成功*/
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        0.1ms, std::bind(&imu_can::data_callback, this));
  }

 private:
  /*声明CAN相关变量*/
  int s, nbytes;
  float gravity_acc;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame;
  std::string CAN_name;
  std::string imu_frame_id;
  std::vector<can_frame> can_vec;
  bool flag_50 = false;
  bool flag_51 = false;
  bool flag_52 = false;
  bool flag_53_1 = false;
  bool flag_53_2 = false;
  bool flag_53_3 = false;
  bool flag_54 = false;
  tf2::Quaternion orientation_tf;
  
  /*声明时间循环、发布者和订阅者*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ImuData_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::ImuHwt9073can>::SharedPtr ImuHwtData_publisher_;
  std_msgs::msg::String origin_data;
  sensor_msgs::msg::Imu Imu_data;
  sensor_driver_msgs::msg::ImuHwt9073can ImuHwt_data;

  /*CAN报文信息原始值*/
    /*反馈部分*/
    unsigned int ImuHwt_time_year;
    unsigned int ImuHwt_time_month;
    unsigned int ImuHwt_time_day;
    unsigned int ImuHwt_time_hour;
    unsigned int ImuHwt_time_min;
    unsigned int ImuHwt_time_sec;
    signed int ImuHwt_acc_x_raw;
    signed int ImuHwt_acc_y_raw;
    signed int ImuHwt_acc_z_raw;
    signed int ImuHwt_a_vel_x_raw;
    signed int ImuHwt_a_vel_y_raw;
    signed int ImuHwt_a_vel_z_raw;
    signed int ImuHwt_angle_x_raw;
    signed int ImuHwt_angle_y_raw;
    signed int ImuHwt_angle_z_raw;
    signed int ImuHwt_mag_x;
    signed int ImuHwt_mag_y;
    signed int ImuHwt_mag_z;
    double ImuHwt_acc_x;
    double ImuHwt_acc_y;
    double ImuHwt_acc_z;
    double ImuHwt_a_vel_x;
    double ImuHwt_a_vel_y;
    double ImuHwt_a_vel_z;
    double ImuHwt_angle_x;
    double ImuHwt_angle_y;
    double ImuHwt_angle_z;

  void data_callback() {
    nbytes = read(s, &frame, sizeof(frame)); //接收报文
    //报文解析
    if(nbytes > 0){
    // origin_data.data = "test time";
    // rawData_publisher_->publish(origin_data);
      if(frame.can_id == 0x50){
        if(frame.data[0] == 0x55){
            if(frame.data[1] == 0x50){
                /*进入时间数据处理*/
                flag_50 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x50.");
                ImuHwt_data.time_year = ImuHwt_time_year = frame.data[2];
                ImuHwt_data.time_month = ImuHwt_time_month = frame.data[3];
                ImuHwt_data.time_day = ImuHwt_time_day = frame.data[4];
                ImuHwt_data.time_hour = ImuHwt_time_hour = frame.data[5];
                ImuHwt_data.time_min = ImuHwt_time_min = frame.data[6];
                ImuHwt_data.time_sec = ImuHwt_time_sec = frame.data[7];
            }else if(frame.data[1] == 0x51){
                /*进入加速度数据处理*/
                flag_51 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x51.");
                ImuHwt_acc_x_raw = frame.data[3];
                ImuHwt_acc_x_raw = ImuHwt_acc_x_raw*0x100 + frame.data[2];
                ImuHwt_data.acc_x_raw = ImuHwt_acc_x_raw;
                ImuHwt_acc_x = (double)ImuHwt_acc_x_raw;
                if(ImuHwt_acc_x>32768){
                    ImuHwt_acc_x = (double)ImuHwt_acc_x - 65536;
                }
                ImuHwt_acc_x = ImuHwt_acc_x*gravity_acc/2048;
                ImuHwt_data.acc_x = ImuHwt_acc_x;

                ImuHwt_acc_y_raw = frame.data[5];
                ImuHwt_acc_y_raw = ImuHwt_acc_y_raw*0x100 + frame.data[4];
                ImuHwt_data.acc_y_raw = ImuHwt_acc_y_raw;
                ImuHwt_acc_y = (double)ImuHwt_acc_y_raw;
                if(ImuHwt_acc_y>32768){
                    ImuHwt_acc_y = (double)ImuHwt_acc_y - 65536;
                }
                ImuHwt_acc_y = ImuHwt_acc_y*gravity_acc/2048;
                ImuHwt_data.acc_y = ImuHwt_acc_y;
                
                ImuHwt_acc_z_raw = frame.data[7];
                ImuHwt_acc_z_raw = ImuHwt_acc_z_raw*0x100 + frame.data[6];
                ImuHwt_data.acc_z_raw = ImuHwt_acc_z_raw;
                ImuHwt_acc_z = (double)ImuHwt_acc_z_raw;
                if(ImuHwt_acc_z>32768){
                    ImuHwt_acc_z = (double)ImuHwt_acc_z - 65536;
                }
                ImuHwt_acc_z = ImuHwt_acc_z*gravity_acc/2048;
                ImuHwt_data.acc_z = ImuHwt_acc_z;
            }else if(frame.data[1] == 0x52){
                /*进入角速度数据处理*/
                flag_52 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x52.");
                ImuHwt_a_vel_x_raw = frame.data[3];
                ImuHwt_a_vel_x_raw = ImuHwt_a_vel_x_raw*0x100 + frame.data[2];
                ImuHwt_data.a_vel_x_raw = ImuHwt_a_vel_x_raw;
                ImuHwt_a_vel_x = (double)ImuHwt_a_vel_x_raw;
                if(ImuHwt_a_vel_x>32768){
                    ImuHwt_a_vel_x = (double)ImuHwt_a_vel_x - 65536;
                }
                ImuHwt_a_vel_x = ImuHwt_a_vel_x*0.06103515625;
                ImuHwt_data.a_vel_x = ImuHwt_a_vel_x;
                
                ImuHwt_a_vel_y_raw = frame.data[5];
                ImuHwt_a_vel_y_raw = ImuHwt_a_vel_y_raw*0x100 + frame.data[4];
                ImuHwt_data.a_vel_y_raw = ImuHwt_a_vel_y_raw;
                ImuHwt_a_vel_y = (double)ImuHwt_a_vel_y_raw;
                if(ImuHwt_a_vel_y>32768){
                    ImuHwt_a_vel_y = (double)ImuHwt_a_vel_y - 65536;
                }
                ImuHwt_a_vel_y = ImuHwt_a_vel_y*0.06103515625;
                ImuHwt_data.a_vel_y = ImuHwt_a_vel_y;

                ImuHwt_a_vel_z_raw = frame.data[7];
                ImuHwt_a_vel_z_raw = ImuHwt_a_vel_z_raw*0x100 + frame.data[6];
                ImuHwt_data.a_vel_z_raw = ImuHwt_a_vel_z_raw;
                ImuHwt_a_vel_z = (double)ImuHwt_a_vel_z_raw;
                if(ImuHwt_a_vel_z>32768){
                    ImuHwt_a_vel_z = (double)ImuHwt_a_vel_z - 65536;
                }
                ImuHwt_a_vel_z = ImuHwt_a_vel_z*0.06103515625;
                ImuHwt_data.a_vel_z = ImuHwt_a_vel_z;
            }else if(frame.data[1] == 0x53){
                /*进入角度数据处理*/
                if(frame.data[2]==0x01){
                flag_53_1 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x53_1.");
                ImuHwt_angle_x_raw = frame.data[7];
                ImuHwt_angle_x_raw = ImuHwt_angle_x_raw*0x100 + frame.data[6];
                ImuHwt_angle_x_raw = ImuHwt_angle_x_raw*0x100 + frame.data[5];
                ImuHwt_angle_x_raw = ImuHwt_angle_x_raw*0x100 + frame.data[4];
                ImuHwt_data.angle_x_raw = ImuHwt_angle_x_raw;
                ImuHwt_angle_x = (double)ImuHwt_angle_x_raw;
                if(ImuHwt_angle_x>32768){
                    ImuHwt_angle_x = (double)ImuHwt_angle_x - 65536;
                }
                ImuHwt_angle_x = ImuHwt_angle_x / 1000;
                ImuHwt_data.angle_x = ImuHwt_angle_x;
                }else
                if(frame.data[2]==0x02){
                flag_53_2 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x53_2.");
                ImuHwt_angle_y_raw = frame.data[7];
                ImuHwt_angle_y_raw = ImuHwt_angle_y_raw*0x100 + frame.data[6];
                ImuHwt_angle_y_raw = ImuHwt_angle_y_raw*0x100 + frame.data[5];
                ImuHwt_angle_y_raw = ImuHwt_angle_y_raw*0x100 + frame.data[4];
                ImuHwt_data.angle_y_raw = ImuHwt_angle_y_raw;
                ImuHwt_angle_y = (double)ImuHwt_angle_y_raw;
                if(ImuHwt_angle_y>32768){
                    ImuHwt_angle_y = (double)ImuHwt_angle_y - 65536;
                }
                ImuHwt_angle_y = ImuHwt_angle_y / 1000;
                ImuHwt_data.angle_y = ImuHwt_angle_y;
                }else
                if(frame.data[2]==0x03){
                flag_53_3 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x53_3.");
                ImuHwt_angle_z_raw = frame.data[7];
                ImuHwt_angle_z_raw = ImuHwt_angle_z_raw*0x100 + frame.data[6];
                ImuHwt_angle_z_raw = ImuHwt_angle_z_raw*0x100 + frame.data[5];
                ImuHwt_angle_z_raw = ImuHwt_angle_z_raw*0x100 + frame.data[4];
                ImuHwt_data.angle_z_raw = ImuHwt_angle_z_raw;
                ImuHwt_angle_z = (double)ImuHwt_angle_z_raw;
                if(ImuHwt_angle_z>32768){
                    ImuHwt_angle_z = (double)ImuHwt_angle_z - 65536;
                }
                ImuHwt_angle_z = ImuHwt_angle_z / 1000;
                ImuHwt_data.angle_z = ImuHwt_angle_z;
                }
            }else if(frame.data[1] == 0x54){
                /*进入磁场数据处理*/
                flag_54 = true;
                RCLCPP_INFO(this->get_logger(), "Successful get 0x54.");
                ImuHwt_mag_x = frame.data[3];
                ImuHwt_mag_x = ImuHwt_mag_x*0x100 + frame.data[2];
                ImuHwt_mag_y = frame.data[5];
                ImuHwt_mag_y = ImuHwt_mag_x*0x100 + frame.data[4];
                ImuHwt_mag_z = frame.data[7];
                ImuHwt_mag_z = ImuHwt_mag_x*0x100 + frame.data[6];
                ImuHwt_data.mag_x = ImuHwt_mag_x;
                ImuHwt_data.mag_y = ImuHwt_mag_y;
                ImuHwt_data.mag_z = ImuHwt_mag_z;
            }else{
            RCLCPP_INFO(this->get_logger(), "Wrong data id, passed this section. data0=%x,data1=%x,data2=%x,data3=%x,data4=%x,data5=%x,data6=%x,data7=%x.",frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
            return;
            }
            if(flag_50&&flag_51&&flag_52&&flag_53_1&&flag_53_2&&flag_53_3&&flag_54){
            //if(flag_50&&flag_51&&flag_52&&flag_53_1&&flag_53_2&&flag_53_3){
                /*检测是否有新数据，并发布相关话题*/
                orientation_tf.setRPY(ImuHwt_angle_x,ImuHwt_angle_y,ImuHwt_angle_z);
                Imu_data.orientation.x = orientation_tf.x();
                Imu_data.orientation.y = orientation_tf.y();
                Imu_data.orientation.z = orientation_tf.z();
                Imu_data.orientation.w = orientation_tf.w();
                Imu_data.orientation_covariance = {-1,0,0,0,0,0,0,0,0};
                Imu_data.angular_velocity.x = ImuHwt_a_vel_x;
                Imu_data.angular_velocity.y = ImuHwt_a_vel_y;
                Imu_data.angular_velocity.z = ImuHwt_a_vel_z;
                Imu_data.angular_velocity_covariance = {-1,0,0,0,0,0,0,0,0};
                Imu_data.linear_acceleration.x = ImuHwt_acc_x;
                Imu_data.linear_acceleration.y = ImuHwt_acc_y;
                Imu_data.linear_acceleration.z = ImuHwt_acc_z;
                Imu_data.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

                Imu_data.header.stamp = this->get_clock()->now();
                ImuData_publisher_->publish(Imu_data);
                ImuHwt_data.header.stamp = this->get_clock()->now();
                ImuHwtData_publisher_->publish(ImuHwt_data);
                
                RCLCPP_INFO(this->get_logger(), "Successful pub imudata.");
                flag_50 = false;
                flag_51 = false;
                flag_52 = false;
                flag_53_1 = false;
                flag_53_2 = false;
                flag_53_3 = false;
                flag_54 = false;
            }
        }else{
        RCLCPP_INFO(this->get_logger(), "Wrong CAN id, passed this section. CAN id = %x . CAN data1 = %x . CAN data2 = %x . CAN data3 = %x . CAN data4 = %x . CAN data5 = %x . CAN data6 = %x . CAN data7 = %x . CAN data8 = %x .",frame.can_id,frame.data[0],frame.data[1],frame.data[2],frame.data[3],frame.data[4],frame.data[5],frame.data[6],frame.data[7]);
        return;
        }
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<imu_can>("imu_can_driver_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
