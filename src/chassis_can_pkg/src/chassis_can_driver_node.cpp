#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_driver_msgs/msg/chassis_can_recived.hpp"
#include "sensor_driver_msgs/msg/chassis_can_sended.hpp"
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
class chassis_can : public rclcpp::Node{
 public:

  // 构造函数,有一个参数为节点名称
  explicit chassis_can(std::string name) : Node(name){

    /*声明参数*/
    this->declare_parameter("chassis_can_name", "can1");
    this->declare_parameter("speed_coefficient", 10.0); 
    this->declare_parameter("torque_coefficient", 1.0); 
    this->declare_parameter("lr_signal", 1); 
    this->declare_parameter("control_methon_all_flag", false); 
    this->declare_parameter("control_methon_ssb_flag", true); 
    this->declare_parameter("speed_subscribe_topic", "/Control_msgs/speed"); 
    this->declare_parameter("steerAngle_subscribe_topic", "/Control_msgs/steerAngle"); 
    this->declare_parameter("brake_subscribe_topic", "/Control_msgs/brakeRate"); 
    this->declare_parameter("all_subscribe_topic", "/Control_msgs/chassisCan"); 
    this->declare_parameter("rawData_publisher_topic", "/Sensor_msgs/chassis_can/rawData"); 
    this->declare_parameter("speed_publisher_topic", "/Sensor_msgs/chassis_can/speed"); 
    this->declare_parameter("steerAngle_publisher_topic", "/Sensor_msgs/chassis_can/steerAngle"); 
    this->declare_parameter("brake_publisher_topic", "/Sensor_msgs/chassis_can/brakeRate"); 
    this->declare_parameter("soc_publisher_topic", "/Sensor_msgs/chassis_can/SOC"); 
    this->declare_parameter("all_publisher_topic", "/Sensor_msgs/chassis_can/chassisCan"); 

    /*获取参数*/
    this->get_parameter("chassis_can_name", CAN_name);
    this->get_parameter("speed_coefficient", speed_coefficient);
    this->get_parameter("torque_coefficient", torque_coefficient);
    this->get_parameter("lr_signal", lr_signal);
    this->get_parameter("control_methon_all_flag", control_methon_all_flag);
    if(control_methon_all_flag)control_methon_ssb_flag=false;
    this->get_parameter("control_methon_ssb_flag", control_methon_ssb_flag);
    if(control_methon_ssb_flag)control_methon_all_flag=false;
    this->get_parameter("speed_subscribe_topic", speed_subscribe_topic);
    this->get_parameter("steerAngle_subscribe_topic", steerAngle_subscribe_topic);
    this->get_parameter("brake_subscribe_topic", brake_subscribe_topic);
    this->get_parameter("all_subscribe_topic", all_subscribe_topic);
    this->get_parameter("rawData_publisher_topic", rawData_publisher_topic);
    this->get_parameter("speed_publisher_topic", speed_publisher_topic);
    this->get_parameter("steerAngle_publisher_topic", steerAngle_publisher_topic);
    this->get_parameter("brake_publisher_topic", brake_publisher_topic);
    this->get_parameter("soc_publisher_topic", soc_publisher_topic);
    this->get_parameter("all_publisher_topic", all_publisher_topic);

    std::stringstream ss;
    ss.str("");
    ss << "\nCAN_name = " << CAN_name;
    ss << "\nspeed_coefficient = " << speed_coefficient;
    ss << "\ntorque_coefficient = " << torque_coefficient;
    ss << "\nlr_signal = " << lr_signal;
    ss << "\ncontrol_methon_all_flag = " << control_methon_all_flag;
    ss << "\ncontrol_methon_ssb_flag = " << control_methon_ssb_flag;
    ss << "\nspeed_subscribe_topic = " << speed_subscribe_topic;
    ss << "\nsteerAngle_subscribe_topic = " << steerAngle_subscribe_topic;
    ss << "\nbrake_subscribe_topic = " << brake_subscribe_topic;
    ss << "\nall_subscribe_topic = " << all_subscribe_topic;
    ss << "\nrawData_publisher_topic = " << rawData_publisher_topic;
    ss << "\nspeed_publisher_topic = " << speed_publisher_topic;
    ss << "\nsteerAngle_publisher_topic = " << steerAngle_publisher_topic;
    ss << "\nbrake_publisher_topic = " << brake_publisher_topic;
    ss << "\nsoc_publisher_topic = " << soc_publisher_topic;
    ss << "\nall_publisher_topic = " << all_publisher_topic;
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());

    /*创建发布函数*/
    rawData_publisher_ = this->create_publisher<std_msgs::msg::String>(rawData_publisher_topic, 10);
    speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(speed_publisher_topic, 10);
    steerAngle_publisher_ = this->create_publisher<std_msgs::msg::Float32>(steerAngle_publisher_topic, 10);
    brake_publisher_ = this->create_publisher<std_msgs::msg::Float32>(brake_publisher_topic, 10);
    soc_publisher_ = this->create_publisher<std_msgs::msg::Int16>(soc_publisher_topic, 10);
    all_publisher_ = this->create_publisher<sensor_driver_msgs::msg::ChassisCanRecived>(all_publisher_topic, 10);

    /*创建订阅函数*/
    if(control_methon_ssb_flag){
      speed_subscribe_ = this->create_subscription<std_msgs::msg::Float32>(speed_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_speed, this, std::placeholders::_1));
      steerAngle_subscribe_ = this->create_subscription<std_msgs::msg::Float32>(steerAngle_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_steerAngle, this, std::placeholders::_1));
      brake_subscribe_ = this->create_subscription<std_msgs::msg::Float32>(brake_subscribe_topic, 10, std::bind(&chassis_can::command_callback_chassis_brake, this, std::placeholders::_1));
    }
    if(control_methon_all_flag){
      all_subscribe_ = this->create_subscription<sensor_driver_msgs::msg::ChassisCanSended>(all_subscribe_topic,10,std::bind(&chassis_can::command_callback_chassis_can_sended, this, std::placeholders::_1));
    }

    /*初始化CAN数组信号*/
    frame.can_dlc = 8;
    frame.can_id = 0x351;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x353;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x355;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x357;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x359;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x361;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);
    frame.can_id = 0x363;
    for(int i = 0; i < 8; i ++) frame.data[i] = 0x00;
    can_vec.push_back(frame);

    /*初始化计数器*/
    IDUVCU_Ctrl_RollCnt = 0;
    IDUVCU_Ctrl_WorkMod = 0x1; //进入自动驾驶模式
    IDUVCU_Ctrl_ESTOP = 0x0; //取消紧急制动状态
    IDUVCU_Ctrl_ReStop = 0x0; //停止重置制动状态
    IDUMCU_Ctrl_WorkMod = 0x1; //车速控制为速度模式
    IDUEPB_Ctrl_ReqPark = 0x2; //松开驻车制动
    IDUBCM_Ctrl_HeadLight = 0x1; //打开大灯
    IDUMCU_Ctrl_RND = 0x1; //挂前进档D

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
        1ms, std::bind(&chassis_can::data_callback, this));
    timerSend_ = this->create_wall_timer(
        10ms, std::bind(&chassis_can::data_sendCAN, this));
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
  rclcpp::TimerBase::SharedPtr timerSend_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rawData_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steerAngle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr soc_publisher_;
  rclcpp::Publisher<sensor_driver_msgs::msg::ChassisCanRecived>::SharedPtr all_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscribe_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steerAngle_subscribe_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brake_subscribe_;
  rclcpp::Subscription<sensor_driver_msgs::msg::ChassisCanSended>::SharedPtr all_subscribe_;

  /*CAN报文信息原始值*/
    /*控制部分*/
  unsigned int IDUVCU_Ctrl_WorkMod,IDUVCU_Ctrl_ESTOP,IDUVCU_Ctrl_ReStop,IDUVCU_Ctrl_RollCnt,IDUVCU_Ctrl_Checksum;//地址0x351
  unsigned int IDUMCU_Ctrl_WorkMod,IDUMCU_Ctrl_RND;//地址0x353
  int IDUMCU_Ctrl_TgtSped,IDUMCU_Ctrl_TgtAcc,IDUMCU_Ctrl_TgtTorq;//地址0x353
  unsigned int IDUEPB_Ctrl_ReqPark;//地址0x355
  int IDUEPS_Ctrl_TgtAngle;//地址0x357
  unsigned int IDUEHB_Ctrl_BrakMod,IDUEHB_Ctrl_TgtPressure;//地址0x359
  int IDUEHB_Ctrl_TgtDece;//地址0x359
  unsigned int IDUBMS_Ctrl_Power;//地址0x361
  unsigned int IDUBCM_Ctrl_HeadLight,IDUBCM_Ctrl_LeftFlash,IDUBCM_Ctrl_RightFlash,IDUBCM_Ctrl_BackLight,IDUBCM_Ctrl_BrakeLight,IDUBCM_Ctrl_Siren,IDUBCM_Ctrl_Voice,IDUBCM_Ctrl_DoubleFlash;//地址0x363

    /*反馈部分*/
  unsigned int VCUIDU_Fdbk_WorkMod,VCUIDU_Fdbk_ModelStop,VCUIDU_Fdbk_Power,VCUIDU_Fdbk_Erro,VCU_Fdbk_SoftVer,VCU_Fdbk_RollCnt,VCUIDU_Fdbk_Checksum; //地址0x352
  unsigned int MCUIDU_Fdbk_RND,MCUIDU_Fdbk_RealTorq,MCUIDU_Fdbk_RealRpm,MCUIDU_Fdbk_ErroLevel,MCUIDU_Fdbk_Fault;//地址0x354
  int MCUIDU_Fdbk_RealSped,MCUIDU_Fdbk_RealAcc;//地址0x354，带有正负
  unsigned int EPBIDU_Fdbk_ParkSta,EPBIDU_Fdbk_ErroLevel,EPBIDU_Fdbk_Fault;//地址0x356
  unsigned int EPSIDU_Fdbk_WorkSta,EPSIDU_Fdbk_ErroLevel,EPSIDU_Fdbk_Fault;//地址0x358
  int EPSIDU_Fdbk_RealAngle;//地址0x358，带有正负
  unsigned int EHBIDU_Fdbk_RealPressure,EHBIDU_Fdbk_ErroLevel,EHBIDU_Fdbk_Fault;//地址0x360
  int EHBIDU_Fdbk_RealDece;//地址0x360，带有正负
  unsigned int BMSIDU_Fdbk_CHCSta,BMSIDU_Fdbk_SOC,BMSIDU_Fdbk_BusV,BMSIDU_Fdbk_ErroLevel,BMSIDU_Fdbk_Fault;//地址0x362
  int BMSIDU_Fdbk_BusA;//地址0x362，带有正负
  unsigned int BCMIDU_Fdbk_HeadLight,BCMIDU_Fdbk_TurnFlash,BCMIDU_Fdbk_BackLight,BCMIDU_Fdbk_BrakeLight,BCMIDU_Fdbk_Siren,BCMIDU_Fdbk_Voice,BCMIDU_Fdbk_DoubleFlash;//地址0x364
  unsigned int check_sum_counter;

  /*底盘数据处理后值*/
  float speed_coefficient,torque_coefficient;
  int lr_signal,pn_signal;
  bool control_methon_all_flag,control_methon_ssb_flag;
  float speed_publisher_float = 0.0;
  float steerAngle_publisher_float = 0.0;
  std::string speed_subscribe_topic,steerAngle_subscribe_topic,brake_subscribe_topic,all_subscribe_topic;
  std::string rawData_publisher_topic,speed_publisher_topic,steerAngle_publisher_topic,brake_publisher_topic,soc_publisher_topic,all_publisher_topic;
  std_msgs::msg::String rawData_publisher_message;
  std_msgs::msg::Float32 speed_publisher_message;
  std_msgs::msg::Float32 steerAngle_publisher_message;
  std_msgs::msg::Float32 brake_publisher_message;
  std_msgs::msg::Int16 soc_publisher_message;
  sensor_driver_msgs::msg::ChassisCanRecived all_publisher_message;

  void data_callback() {
    nbytes = read(s, &frame, sizeof(frame)); //接收报文
    //报文解析
    if(nbytes > 0){
      if(frame.can_id == 0x352){
        /*校验和*/
        VCUIDU_Fdbk_Checksum = frame.data[7];
        check_sum_counter = frame.data[0]^frame.data[1]^frame.data[2]^frame.data[3]^frame.data[4]^frame.data[5]^frame.data[6];
        all_publisher_message.vcuidu_fdbk_checksum=VCUIDU_Fdbk_Checksum;
        if(check_sum_counter!=VCUIDU_Fdbk_Checksum){
          RCLCPP_ERROR(this->get_logger(),"VCUIDU(can_id 0x352) check sum ERROR!");
        }else{
          /*当前驾驶模式状态*/
          VCUIDU_Fdbk_WorkMod = frame.data[0]%0x04;
          all_publisher_message.vcuidu_fdbk_workmod=VCUIDU_Fdbk_WorkMod;
          /*当前停止状态*/
          VCUIDU_Fdbk_ModelStop = frame.data[0]%0x20/0x04;
          all_publisher_message.vcuidu_fdbk_modelstop=VCUIDU_Fdbk_ModelStop;
          /*当前点火状态*/
          VCUIDU_Fdbk_Power = frame.data[0]%0x40/0x20;
          all_publisher_message.vcuidu_fdbk_power=VCUIDU_Fdbk_Power;
          /*VCU故障信息*/
          VCUIDU_Fdbk_Erro = frame.data[1]%0x08;
          all_publisher_message.vcuidu_fdbk_erro=VCUIDU_Fdbk_Erro;
          /*VCU软件版本*/
          VCU_Fdbk_SoftVer = ((frame.data[5]%0x10*256+frame.data[4])*256+frame.data[3])*256+frame.data[2];
          all_publisher_message.vcu_fdbk_softver=VCU_Fdbk_SoftVer;
          /*循环计数*/
          VCU_Fdbk_RollCnt = frame.data[6]%0x10;
          all_publisher_message.vcu_fdbk_rollcnt=VCU_Fdbk_RollCnt;
          all_publisher_->publish(all_publisher_message);
        }
      }
      else if(frame.can_id == 0x354){
        /*当前档位*/
        MCUIDU_Fdbk_RND = frame.data[0]%0x04;
        all_publisher_message.mcuidu_fdbk_rnd=MCUIDU_Fdbk_RND;
        /*车辆实际速度*/
        if(frame.data[1]%0x40/0x20){
          MCUIDU_Fdbk_RealSped = ((frame.data[1]%0x40*0x100+frame.data[0])/0x04 - 0x0fff) * speed_coefficient;
        }else{
          MCUIDU_Fdbk_RealSped = (frame.data[1]%0x40*0x100+frame.data[0])/0x04 * speed_coefficient;
        }
        all_publisher_message.mcuidu_fdbk_realsped=MCUIDU_Fdbk_RealSped;
        /*发布车辆运行速度信息*/
        speed_publisher_float = MCUIDU_Fdbk_RealSped*0.01;
        speed_publisher_message.data = speed_publisher_float;
        speed_publisher_->publish(speed_publisher_message);
        // RCLCPP_INFO(this->get_logger(),"MCUIDU_Fdbk_RealSped raw data : %x %x, inter data : %d , real data : %f",frame.data[1],frame.data[0],MCUIDU_Fdbk_RealSped,speed_publisher_float);
        
        /*车辆实际加速度*/
        if(frame.data[3]%0x04/0x02){
          MCUIDU_Fdbk_RealAcc = (((frame.data[3]%0x04*0x100+frame.data[2])*0x100+frame.data[1])/0x40-0x0fff) * speed_coefficient;
        }else{
          MCUIDU_Fdbk_RealAcc = (((frame.data[3]%0x04*0x100+frame.data[2])*0x100+frame.data[1])/0x40) * speed_coefficient;
        }
        all_publisher_message.mcuidu_fdbk_realacc=MCUIDU_Fdbk_RealAcc;
        /*实际车辆输出扭矩*/
        MCUIDU_Fdbk_RealTorq = ((frame.data[3]%0x40*0x100+frame.data[4])/0x04) * torque_coefficient;
        all_publisher_message.mcuidu_fdbk_realtorq=MCUIDU_Fdbk_RealTorq;
        /*电机转速*/
        MCUIDU_Fdbk_RealRpm = ((frame.data[4]%0x40*0x100+frame.data[5])*0x100+frame.data[6])/0x20;
        all_publisher_message.mcuidu_fdbk_realrpm=MCUIDU_Fdbk_RealRpm;
        // RCLCPP_INFO(this->get_logger(),"MCUIDU_Fdbk_RealRpm real data : %d",MCUIDU_Fdbk_RealRpm);

        /*故障等级*/
        MCUIDU_Fdbk_ErroLevel = frame.data[6]/0x04;
        all_publisher_message.mcuidu_fdbk_errolevel=MCUIDU_Fdbk_ErroLevel;
        /*故障信息*/
        MCUIDU_Fdbk_Fault = frame.data[7];
        all_publisher_message.mcuidu_fdbk_fault=MCUIDU_Fdbk_Fault;
      }
      else if(frame.can_id == 0x356){
        /*驻车状态*/
        EPBIDU_Fdbk_ParkSta = frame.data[0]%0x08;
        all_publisher_message.epbidu_fdbk_parksta=EPBIDU_Fdbk_ParkSta;
        /*驻车系统的故障等级*/
        EPBIDU_Fdbk_ErroLevel = frame.data[1];
        all_publisher_message.epbidu_fdbk_errolevel=EPBIDU_Fdbk_ErroLevel;
        /*驻车故障状态*/
        EPBIDU_Fdbk_Fault = frame.data[2];
        all_publisher_message.epbidu_fdbk_fault=EPBIDU_Fdbk_Fault;
      }
      else if(frame.can_id == 0x358){
        /*前轮实际转角*/
        if(frame.data[1]/0x80){
          EPSIDU_Fdbk_RealAngle = (frame.data[1]%0x80*0x100+frame.data[0])-0x7fff;
        }else{
          EPSIDU_Fdbk_RealAngle = (frame.data[1]%0x80*0x100+frame.data[0]);
        }
        all_publisher_message.epsidu_fdbk_realangle=EPSIDU_Fdbk_RealAngle;
        // RCLCPP_INFO(this->get_logger(),"EPSIDU_Fdbk_RealAngle raw data : %x %x",frame.data[1],frame.data[0]);
        /*发布前轮转向角度信息*/
        steerAngle_publisher_float = EPSIDU_Fdbk_RealAngle*0.1;
        steerAngle_publisher_message.data = steerAngle_publisher_float;
        steerAngle_publisher_->publish(steerAngle_publisher_message);

        /*工作状态*/
        EPSIDU_Fdbk_WorkSta = frame.data[2]%0x10;
        all_publisher_message.epsidu_fdbk_worksta=EPSIDU_Fdbk_WorkSta;
        /*故障等级*/
        EPSIDU_Fdbk_ErroLevel = frame.data[2]/0x10;
        all_publisher_message.epsidu_fdbk_errolevel=EPSIDU_Fdbk_ErroLevel;
        /*故障信息*/
        EPSIDU_Fdbk_Fault = frame.data[3];
        all_publisher_message.epsidu_fdbk_fault=EPSIDU_Fdbk_Fault;
      }
      else if(frame.can_id == 0x360){
        /*车辆实际减速度*/
        EHBIDU_Fdbk_RealDece = (frame.data[1]%0x10*0x100+frame.data[0]);
        all_publisher_message.ehbidu_fdbk_realdece =EHBIDU_Fdbk_RealDece ;
        /*实际制动油压值*/
        EHBIDU_Fdbk_RealPressure = frame.data[2];
        all_publisher_message.ehbidu_fdbk_realpressure=EHBIDU_Fdbk_RealPressure;
        brake_publisher_message.data = EHBIDU_Fdbk_RealPressure;
        brake_publisher_->publish(brake_publisher_message);
        /*故障等级*/
        EHBIDU_Fdbk_ErroLevel = frame.data[3];
        all_publisher_message.ehbidu_fdbk_errolevel=EHBIDU_Fdbk_ErroLevel;
        /*故障信息*/
        EHBIDU_Fdbk_Fault = frame.data[4];
        all_publisher_message.ehbidu_fdbk_fault=EHBIDU_Fdbk_Fault;
      }
      else if(frame.can_id == 0x362){
        /*充电状态*/
        BMSIDU_Fdbk_CHCSta = frame.data[0]%0x02;
        all_publisher_message.bmsidu_fdbk_chcsta=BMSIDU_Fdbk_CHCSta;
        /*剩余电量(SOC)*/
        BMSIDU_Fdbk_SOC = (frame.data[1]%0x20*0x100+frame.data[0])/0x02;
        all_publisher_message.bmsidu_fdbk_soc=BMSIDU_Fdbk_SOC;
        soc_publisher_message.data=BMSIDU_Fdbk_SOC;
        soc_publisher_->publish(soc_publisher_message);
        /*母线电流*/
        if(frame.data[3]/0x80){
          BMSIDU_Fdbk_BusA = -(frame.data[3]%0x80*0x100 + frame.data[2] - 0xffff);
        }else{
          BMSIDU_Fdbk_BusA = frame.data[3]%0x80*0x100 + frame.data[2];
        }
        all_publisher_message.bmsidu_fdbk_busa=BMSIDU_Fdbk_BusA;
        /*母线电压*/
        BMSIDU_Fdbk_BusV = frame.data[5]*0x100+frame.data[4];
        all_publisher_message.bmsidu_fdbk_busv=BMSIDU_Fdbk_BusV;
        /*故障等级*/
        BMSIDU_Fdbk_ErroLevel = frame.data[6];
        all_publisher_message.bmsidu_fdbk_errolevel=BMSIDU_Fdbk_ErroLevel;
        /*故障信息*/
        BMSIDU_Fdbk_Fault = frame.data[1]/0x20;
        all_publisher_message.bmsidu_fdbk_fault=BMSIDU_Fdbk_Fault;
      }
      else if(frame.can_id == 0x364){
        /*大灯状态反馈*/
        BCMIDU_Fdbk_HeadLight = frame.data[0]%0x04;
        all_publisher_message.bcmidu_fdbk_headlight=BCMIDU_Fdbk_HeadLight;
        /*转向状态反馈*/
        BCMIDU_Fdbk_TurnFlash = frame.data[0]%0x10/0x04;
        all_publisher_message.bcmidu_fdbk_turnflash=BCMIDU_Fdbk_TurnFlash;
        /*倒车灯状态反馈*/
        BCMIDU_Fdbk_BackLight = frame.data[0]%0x40/0x10;
        all_publisher_message.bcmidu_fdbk_backlight=BCMIDU_Fdbk_BackLight;
        /*刹车灯状态反馈*/
        BCMIDU_Fdbk_BrakeLight = frame.data[0]/0x40;
        all_publisher_message.bcmidu_fdbk_brakelight=BCMIDU_Fdbk_BrakeLight;
        /*喇叭控制*/
        BCMIDU_Fdbk_Siren = frame.data[1]%0x04;
        all_publisher_message.bcmidu_fdbk_siren=BCMIDU_Fdbk_Siren;
        /*行车对外语音播报*/
        BCMIDU_Fdbk_Voice = frame.data[1]%0x10/0x04;
        all_publisher_message.bcmidu_fdbk_voice=BCMIDU_Fdbk_Voice;
        /*双闪反馈*/
        BCMIDU_Fdbk_DoubleFlash = frame.data[1]%0x40/0x10;
        all_publisher_message.bcmidu_fdbk_doubleflash=BCMIDU_Fdbk_DoubleFlash;
      }
      
      std::stringstream ss;
      ss << VCUIDU_Fdbk_WorkMod << ","  << VCUIDU_Fdbk_ModelStop << ","  << VCUIDU_Fdbk_Power << ","  << VCUIDU_Fdbk_Erro << ","  << VCU_Fdbk_SoftVer << ","  << VCU_Fdbk_RollCnt << ","  << VCUIDU_Fdbk_Checksum << ";"  << MCUIDU_Fdbk_RND << ","  << MCUIDU_Fdbk_RealSped << ","  << MCUIDU_Fdbk_RealAcc << ","  << MCUIDU_Fdbk_RealTorq << ","  << MCUIDU_Fdbk_RealRpm << ","  << MCUIDU_Fdbk_ErroLevel << ","  << MCUIDU_Fdbk_Fault << ";"  << EPBIDU_Fdbk_ParkSta << ","  << EPBIDU_Fdbk_ErroLevel << ","  << EPBIDU_Fdbk_Fault << ";"  << EPSIDU_Fdbk_RealAngle << ","  << EPSIDU_Fdbk_WorkSta << ","  << EPSIDU_Fdbk_ErroLevel << ","  << EPSIDU_Fdbk_Fault << ";"  << EHBIDU_Fdbk_RealDece << ","  << EHBIDU_Fdbk_RealPressure << ","  << EHBIDU_Fdbk_ErroLevel << ","  << EHBIDU_Fdbk_Fault << ";"  << BMSIDU_Fdbk_CHCSta << ","  << BMSIDU_Fdbk_SOC << ","  << BMSIDU_Fdbk_BusA << ","  << BMSIDU_Fdbk_BusV << ","  << BMSIDU_Fdbk_ErroLevel << ","  << BMSIDU_Fdbk_Fault << ";"  << BCMIDU_Fdbk_HeadLight << ","  << BCMIDU_Fdbk_TurnFlash << ","  << BCMIDU_Fdbk_BackLight << ","  << BCMIDU_Fdbk_BrakeLight << ","  << BCMIDU_Fdbk_Siren << ","  << BCMIDU_Fdbk_Voice << ","  << BCMIDU_Fdbk_DoubleFlash ;
      ss >> rawData_publisher_message.data;
      rawData_publisher_->publish(rawData_publisher_message);
      // RCLCPP_INFO(this->get_logger(), "\n%d\t%d\t%d\t%d\t%d\t%d\t%d\n%d\t%d\t%d\t%d\t%d\t%d\t%d\n%d\t%d\t%d\n%d\t%d\t%d\t%d\n%d\t%d\t%d\t%d\n%d\t%d\t%d\t%d\t%d\t%d\n%d\t%d\t%d\t%d\t%d\t%d\t%d\n"
      //             ,VCUIDU_Fdbk_WorkMod,VCUIDU_Fdbk_ModelStop,VCUIDU_Fdbk_Power,VCUIDU_Fdbk_Erro,VCU_Fdbk_SoftVer,VCU_Fdbk_RollCnt,VCUIDU_Fdbk_Checksum
      //             ,MCUIDU_Fdbk_RND,MCUIDU_Fdbk_RealSped,MCUIDU_Fdbk_RealAcc,MCUIDU_Fdbk_RealTorq,MCUIDU_Fdbk_RealRpm,MCUIDU_Fdbk_ErroLevel,MCUIDU_Fdbk_Fault
      //             ,EPBIDU_Fdbk_ParkSta,EPBIDU_Fdbk_ErroLevel,EPBIDU_Fdbk_Fault
      //             ,EPSIDU_Fdbk_RealAngle,EPSIDU_Fdbk_WorkSta,EPSIDU_Fdbk_ErroLevel,EPSIDU_Fdbk_Fault
      //             ,EHBIDU_Fdbk_RealDece,EHBIDU_Fdbk_RealPressure,EHBIDU_Fdbk_ErroLevel,EHBIDU_Fdbk_Fault
      //             ,BMSIDU_Fdbk_CHCSta,BMSIDU_Fdbk_SOC,BMSIDU_Fdbk_BusA,BMSIDU_Fdbk_BusV,BMSIDU_Fdbk_ErroLevel,BMSIDU_Fdbk_Fault
      //             ,BCMIDU_Fdbk_HeadLight,BCMIDU_Fdbk_TurnFlash,BCMIDU_Fdbk_BackLight,BCMIDU_Fdbk_BrakeLight,BCMIDU_Fdbk_Siren,BCMIDU_Fdbk_Voice,BCMIDU_Fdbk_DoubleFlash);
    }
  }

  /*订阅控制速度指令回调函数*/
  void command_callback_chassis_speed(const std_msgs::msg::Float32::SharedPtr msg_chassis_speed){
    if(IDUMCU_Ctrl_TgtSped>0.1){
      IDUMCU_Ctrl_RND = 0x1; //挂前进档D
    }else if(IDUMCU_Ctrl_TgtSped<-0.1){
      IDUMCU_Ctrl_RND = 0x2; //挂倒车档R
      IDUMCU_Ctrl_TgtSped = -IDUMCU_Ctrl_TgtSped;
    }else{
      IDUMCU_Ctrl_RND = 0x0; //挂空档N
    }
    IDUMCU_Ctrl_TgtSped = int(msg_chassis_speed->data*100/speed_coefficient);
  }

  /*订阅控制转向角度指令回调函数*/
  void command_callback_chassis_steerAngle(const std_msgs::msg::Float32::SharedPtr msg_chassis_steerAngle){
    IDUEPS_Ctrl_TgtAngle = int(msg_chassis_steerAngle->data*10*lr_signal);
    // RCLCPP_INFO(this->get_logger(),"the control target angle is %d",IDUEPS_Ctrl_TgtAngle);
  }

  /*订阅控制制动压力指令回调函数*/
  void command_callback_chassis_brake(const std_msgs::msg::Float32::SharedPtr msg_chassis_brake){
    IDUEHB_Ctrl_BrakMod = 0x2; //控制制动油压模式
    IDUEHB_Ctrl_TgtPressure = int(msg_chassis_brake->data);
  }

  /**/
  void command_callback_chassis_can_sended(const sensor_driver_msgs::msg::ChassisCanSended::SharedPtr msg){
    IDUVCU_Ctrl_WorkMod=msg->iduvcu_ctrl_workmod;
    IDUVCU_Ctrl_ESTOP=msg->iduvcu_ctrl_estop;
    IDUVCU_Ctrl_ReStop=msg->iduvcu_ctrl_restop;
    IDUVCU_Ctrl_RollCnt=msg->iduvcu_ctrl_rollcnt;
    IDUVCU_Ctrl_Checksum=msg->iduvcu_ctrl_checksum;
    IDUMCU_Ctrl_WorkMod=msg->idumcu_ctrl_workmod;
    IDUMCU_Ctrl_RND=msg->idumcu_ctrl_rnd;
    IDUMCU_Ctrl_TgtSped=msg->idumcu_ctrl_tgtsped;
    IDUMCU_Ctrl_TgtAcc=msg->idumcu_ctrl_tgtacc;
    IDUMCU_Ctrl_TgtTorq=msg->idumcu_ctrl_tgttorq;
    IDUEPB_Ctrl_ReqPark=msg->iduepb_ctrl_reqpark;
    IDUEPS_Ctrl_TgtAngle=msg->idueps_ctrl_tgtangle;
    IDUEHB_Ctrl_BrakMod=msg->iduehb_ctrl_brakmod;
    IDUEHB_Ctrl_TgtPressure=msg->iduehb_ctrl_tgtpressure;
    IDUEHB_Ctrl_TgtDece=msg->iduehb_ctrl_tgtdece;
    IDUBMS_Ctrl_Power=msg->idubms_ctrl_power;
    IDUBCM_Ctrl_HeadLight=msg->idubcm_ctrl_headlight;
    IDUBCM_Ctrl_LeftFlash=msg->idubcm_ctrl_leftflash;
    IDUBCM_Ctrl_RightFlash=msg->idubcm_ctrl_rightflash;
    IDUBCM_Ctrl_BackLight=msg->idubcm_ctrl_backlight;
    IDUBCM_Ctrl_BrakeLight=msg->idubcm_ctrl_brakelight;
    IDUBCM_Ctrl_Siren=msg->idubcm_ctrl_siren;
    IDUBCM_Ctrl_Voice=msg->idubcm_ctrl_voice;
    IDUBCM_Ctrl_DoubleFlash=msg->idubcm_ctrl_doubleflash;
  }

  void data_sendCAN() {
    /*配置CAN报文信息*/
    if(IDUVCU_Ctrl_RollCnt<15){
      IDUVCU_Ctrl_RollCnt = IDUVCU_Ctrl_RollCnt + 1;
    }else{
      IDUVCU_Ctrl_RollCnt = 0;
    }
      /*地址0x351*/
    can_vec[0].data[0]=((IDUVCU_Ctrl_ReStop*0x04+IDUVCU_Ctrl_ESTOP)*0x04+IDUVCU_Ctrl_WorkMod)%0x08;
    can_vec[0].data[6]=IDUVCU_Ctrl_RollCnt%0x10;
    IDUVCU_Ctrl_Checksum = can_vec[0].data[0]^can_vec[0].data[1]^can_vec[0].data[2]^can_vec[0].data[3]^can_vec[0].data[4]^can_vec[0].data[5]^can_vec[0].data[6];
    can_vec[0].data[7]=IDUVCU_Ctrl_Checksum%0x100;
      /*地址0x353*/
    can_vec[1].data[0]=((IDUMCU_Ctrl_TgtSped%0x04*0x04+IDUMCU_Ctrl_RND)*0x08+IDUMCU_Ctrl_WorkMod)%0x80*0x02;
    can_vec[1].data[1]=(IDUMCU_Ctrl_TgtSped%0x104/0x04)%0x100;
    can_vec[1].data[2]=(IDUMCU_Ctrl_TgtAcc%0x40*0x04+IDUMCU_Ctrl_TgtSped/0x104)%0x100;
    can_vec[1].data[3]=(IDUMCU_Ctrl_TgtAcc%0x110/0x40)%0x40;
    can_vec[1].data[5]=IDUMCU_Ctrl_TgtTorq%0x40*0x04;
    can_vec[1].data[6]=(IDUMCU_Ctrl_TgtTorq%0x110/0x40)%0x40;
    // RCLCPP_INFO(this->get_logger(),"IDUMCU_Ctrl_TgtSped raw data : %x %x %x, target data : %d",can_vec[1].data[0],can_vec[1].data[1],can_vec[1].data[2],IDUMCU_Ctrl_TgtSped);

      /*地址0x355*/
    can_vec[2].data[0]=IDUEPB_Ctrl_ReqPark%0x04;
      /*地址0x357*/
    if(IDUEPS_Ctrl_TgtAngle<0){
    	can_vec[3].data[0]=(0x10000+IDUEPS_Ctrl_TgtAngle)%0x100;
    	can_vec[3].data[1]=(0x10000+IDUEPS_Ctrl_TgtAngle)/0x100%0x100;
    }else{
    	can_vec[3].data[0]=IDUEPS_Ctrl_TgtAngle%0x100;
    	can_vec[3].data[1]=IDUEPS_Ctrl_TgtAngle/0x100%0x100;
    }
    
    // RCLCPP_INFO(this->get_logger(),"IDUEPS_Ctrl_TgtAngle raw data : %x %x , control target : %d",can_vec[3].data[1],can_vec[3].data[0],IDUEPS_Ctrl_TgtAngle);
      /*地址0x359*/
    can_vec[4].data[0]=(IDUEHB_Ctrl_TgtPressure%0x40*0x04+IDUEHB_Ctrl_BrakMod)%0x100;
    can_vec[4].data[1]=IDUEHB_Ctrl_TgtPressure/0x40*0x04;
    can_vec[4].data[2]=IDUEHB_Ctrl_TgtDece%0x100;
    can_vec[4].data[3]=IDUEHB_Ctrl_TgtDece/0x100%0x10;
      /*地址0x361*/
    can_vec[5].data[0]=IDUBMS_Ctrl_Power%0x02;
      /*地址0x363*/
    can_vec[6].data[0]=(((IDUBCM_Ctrl_BackLight%0x04*0x04+IDUBCM_Ctrl_RightFlash%0x04)*0x40+IDUBCM_Ctrl_LeftFlash%0x04)*0x40+IDUBCM_Ctrl_HeadLight%0x04);
    can_vec[6].data[1]=(((IDUBCM_Ctrl_DoubleFlash%0x04*0x04+IDUBCM_Ctrl_Voice%0x04)*0x40+IDUBCM_Ctrl_Siren%0x04)*0x40+IDUBCM_Ctrl_BrakeLight%0x04);
  
    /*发送CAN报文协议*/
    frame.can_dlc = 8;
    for(int i = 0; i < 7; i ++){
      frame.can_id = can_vec[i].can_id;
      for(int j = 0; j < 8; j ++) frame.data[j] = can_vec[i].data[j];
      nbytes = write(s, &frame, sizeof(frame)); //发送报文
      if(nbytes != sizeof(frame)) printf("Send Error!\n");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<chassis_can>("chassis_can_driver_node");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
