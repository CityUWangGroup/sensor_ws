#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

using std::placeholders::_1;

class SensorTF2Publisher : public rclcpp::Node
{
public:
    explicit SensorTF2Publisher(std::string name) : Node(name)
    {
        /*声明参数*/
        this->declare_parameter("lidar_main_tf2_flag", true);
        this->declare_parameter("lidar_imu_tf2_flag", true); 
        this->declare_parameter("lidar_1_tf2_flag", false);
        this->declare_parameter("lidar_2_tf2_flag", false);
        this->declare_parameter("lidar_3_tf2_flag", false);
        this->declare_parameter("lidar_4_tf2_flag", false);
        this->declare_parameter("lidar_5_tf2_flag", false);
        this->declare_parameter("lidar_6_tf2_flag", false);
        this->declare_parameter("ins_tf2_flag", true);  
        this->declare_parameter("radar_tf2_flag", false); 
        this->declare_parameter("world_frame_id", "/base_link"); 
        this->declare_parameter("lidar_main_frame_id", "/lidar_front"); 
        this->declare_parameter("lidar_imu_frame_id", "/lidar_front_imu"); 
        this->declare_parameter("lidar_1_frame_id", "/lidar_front/points_xyzi"); 
        this->declare_parameter("lidar_2_frame_id", "/lidar_front/points_xyzi"); 
        this->declare_parameter("lidar_3_frame_id", "/lidar_front/points_xyzi"); 
        this->declare_parameter("lidar_4_frame_id", "/lidar_front/points_xyzi"); 
        this->declare_parameter("lidar_5_frame_id", "/lidar_front/points_xyzi"); 
        this->declare_parameter("lidar_6_frame_id", "/lidar_front/points_xyzi"); 
        this->declare_parameter("ins_frame_id", "/ins"); 
        this->declare_parameter("radar_frame_id", "/radar"); 
        this->declare_parameter("lidar_main_orintation_flag", true);
        this->declare_parameter("lidar_imu_orintation_flag", true); 
        this->declare_parameter("lidar_1_orintation_flag", true);
        this->declare_parameter("lidar_2_orintation_flag", true);
        this->declare_parameter("lidar_3_orintation_flag", true);
        this->declare_parameter("lidar_4_orintation_flag", true);
        this->declare_parameter("lidar_5_orintation_flag", true);
        this->declare_parameter("lidar_6_orintation_flag", true);
        this->declare_parameter("ins_orintation_flag", true); 
        this->declare_parameter("radar_orintation_flag", true); 
        this->declare_parameter("base_2_lidar_main_linear_x", 1.665);
        this->declare_parameter("base_2_lidar_main_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_main_linear_z", 1.742);
        this->declare_parameter("base_2_lidar_main_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_main_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_main_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_main_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_main_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_main_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_main_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_imu_linear_x", 1.665);
        this->declare_parameter("base_2_lidar_imu_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_imu_linear_z", 1.676);
        this->declare_parameter("base_2_lidar_imu_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_imu_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_imu_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_imu_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_imu_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_imu_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_imu_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_1_linear_x", 0.0);
        this->declare_parameter("base_2_lidar_1_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_1_linear_z", 0.0);
        this->declare_parameter("base_2_lidar_1_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_1_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_1_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_1_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_1_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_1_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_1_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_2_linear_x", 0.0);
        this->declare_parameter("base_2_lidar_2_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_2_linear_z", 0.0);
        this->declare_parameter("base_2_lidar_2_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_2_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_2_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_2_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_2_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_2_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_2_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_3_linear_x", 0.0);
        this->declare_parameter("base_2_lidar_3_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_3_linear_z", 0.0);
        this->declare_parameter("base_2_lidar_3_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_3_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_3_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_3_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_3_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_3_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_3_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_4_linear_x", 0.0);
        this->declare_parameter("base_2_lidar_4_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_4_linear_z", 0.0);
        this->declare_parameter("base_2_lidar_4_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_4_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_4_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_4_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_4_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_4_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_4_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_5_linear_x", 0.0);
        this->declare_parameter("base_2_lidar_5_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_5_linear_z", 0.0);
        this->declare_parameter("base_2_lidar_5_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_5_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_5_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_5_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_5_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_5_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_5_angle_z", 0.0);
        this->declare_parameter("base_2_lidar_6_linear_x", 0.0);
        this->declare_parameter("base_2_lidar_6_linear_y", 0.0);
        this->declare_parameter("base_2_lidar_6_linear_z", 0.0);
        this->declare_parameter("base_2_lidar_6_orintation_x", 0.0);
        this->declare_parameter("base_2_lidar_6_orintation_y", 0.0);
        this->declare_parameter("base_2_lidar_6_orintation_z", 0.0);
        this->declare_parameter("base_2_lidar_6_orintation_w", 0.0);
        this->declare_parameter("base_2_lidar_6_angle_x", 0.0);
        this->declare_parameter("base_2_lidar_6_angle_y", 0.0);
        this->declare_parameter("base_2_lidar_6_angle_z", 0.0);
        this->declare_parameter("base_2_ins_linear_x", 0.613);
        this->declare_parameter("base_2_ins_linear_y", 0.0);
        this->declare_parameter("base_2_ins_linear_z", 0.3182);
        this->declare_parameter("base_2_ins_orintation_x", 0.0);
        this->declare_parameter("base_2_ins_orintation_y", 0.0);
        this->declare_parameter("base_2_ins_orintation_z", 0.0);
        this->declare_parameter("base_2_ins_orintation_w", 0.0);
        this->declare_parameter("base_2_ins_angle_x", 0.0);
        this->declare_parameter("base_2_ins_angle_y", 0.0);
        this->declare_parameter("base_2_ins_angle_z", 0.0);
        this->declare_parameter("base_2_radar_linear_x", 0.0);
        this->declare_parameter("base_2_radar_linear_y", 0.0);
        this->declare_parameter("base_2_radar_linear_z", 0.0);
        this->declare_parameter("base_2_radar_orintation_x", 0.0);
        this->declare_parameter("base_2_radar_orintation_y", 0.0);
        this->declare_parameter("base_2_radar_orintation_z", 0.0);
        this->declare_parameter("base_2_radar_orintation_w", 0.0);
        this->declare_parameter("base_2_radar_angle_x", 0.0);
        this->declare_parameter("base_2_radar_angle_y", 0.0);
        this->declare_parameter("base_2_radar_angle_z", 0.0);
        
        /*获取参数*/
        this->get_parameter("lidar_main_tf2_flag", lidar_main_tf2_flag);
        this->get_parameter("lidar_imu_tf2_flag", lidar_imu_tf2_flag);
        this->get_parameter("lidar_1_tf2_flag", lidar_1_tf2_flag);
        this->get_parameter("lidar_2_tf2_flag", lidar_2_tf2_flag);
        this->get_parameter("lidar_3_tf2_flag", lidar_3_tf2_flag);
        this->get_parameter("lidar_4_tf2_flag", lidar_4_tf2_flag);
        this->get_parameter("lidar_5_tf2_flag", lidar_5_tf2_flag);
        this->get_parameter("lidar_6_tf2_flag", lidar_6_tf2_flag);
        this->get_parameter("ins_tf2_flag", ins_tf2_flag);
        this->get_parameter("radar_tf2_flag", radar_tf2_flag);
        this->get_parameter("world_frame_id", world_frame_id);
        this->get_parameter("lidar_main_frame_id", lidar_main_frame_id);
        this->get_parameter("lidar_imu_frame_id", lidar_imu_frame_id);
        this->get_parameter("lidar_1_frame_id", lidar_1_frame_id);
        this->get_parameter("lidar_2_frame_id", lidar_2_frame_id);
        this->get_parameter("lidar_3_frame_id", lidar_3_frame_id);
        this->get_parameter("lidar_4_frame_id", lidar_4_frame_id);
        this->get_parameter("lidar_5_frame_id", lidar_5_frame_id);
        this->get_parameter("lidar_6_frame_id", lidar_6_frame_id);
        this->get_parameter("ins_frame_id", ins_frame_id);
        this->get_parameter("radar_frame_id", radar_frame_id);
        this->get_parameter("base_2_lidar_main_linear_x",base_2_lidar_main_linear_x);
        this->get_parameter("base_2_lidar_main_linear_y",base_2_lidar_main_linear_y);
        this->get_parameter("base_2_lidar_main_linear_z",base_2_lidar_main_linear_z);
        this->get_parameter("base_2_lidar_main_orintation_x",base_2_lidar_main_orintation_x);
        this->get_parameter("base_2_lidar_main_orintation_y",base_2_lidar_main_orintation_y);
        this->get_parameter("base_2_lidar_main_orintation_z",base_2_lidar_main_orintation_z);
        this->get_parameter("base_2_lidar_main_orintation_w",base_2_lidar_main_orintation_w);
        this->get_parameter("base_2_lidar_main_angle_x",base_2_lidar_main_angle_x);
        this->get_parameter("base_2_lidar_main_angle_y",base_2_lidar_main_angle_y);
        this->get_parameter("base_2_lidar_main_angle_z",base_2_lidar_main_angle_z);
        this->get_parameter("base_2_lidar_imu_linear_x",base_2_lidar_imu_linear_x);
        this->get_parameter("base_2_lidar_imu_linear_y",base_2_lidar_imu_linear_y);
        this->get_parameter("base_2_lidar_imu_linear_z",base_2_lidar_imu_linear_z);
        this->get_parameter("base_2_lidar_imu_orintation_x",base_2_lidar_imu_orintation_x);
        this->get_parameter("base_2_lidar_imu_orintation_y",base_2_lidar_imu_orintation_y);
        this->get_parameter("base_2_lidar_imu_orintation_z",base_2_lidar_imu_orintation_z);
        this->get_parameter("base_2_lidar_imu_orintation_w",base_2_lidar_imu_orintation_w);
        this->get_parameter("base_2_lidar_imu_angle_x",base_2_lidar_imu_angle_x);
        this->get_parameter("base_2_lidar_imu_angle_y",base_2_lidar_imu_angle_y);
        this->get_parameter("base_2_lidar_imu_angle_z",base_2_lidar_imu_angle_z);
        this->get_parameter("base_2_lidar_1_linear_x",base_2_lidar_1_linear_x);
        this->get_parameter("base_2_lidar_1_linear_y",base_2_lidar_1_linear_y);
        this->get_parameter("base_2_lidar_1_linear_z",base_2_lidar_1_linear_z);
        this->get_parameter("base_2_lidar_1_orintation_x",base_2_lidar_1_orintation_x);
        this->get_parameter("base_2_lidar_1_orintation_y",base_2_lidar_1_orintation_y);
        this->get_parameter("base_2_lidar_1_orintation_z",base_2_lidar_1_orintation_z);
        this->get_parameter("base_2_lidar_1_orintation_w",base_2_lidar_1_orintation_w);
        this->get_parameter("base_2_lidar_1_angle_x",base_2_lidar_1_angle_x);
        this->get_parameter("base_2_lidar_1_angle_y",base_2_lidar_1_angle_y);
        this->get_parameter("base_2_lidar_1_angle_z",base_2_lidar_1_angle_z);
        this->get_parameter("base_2_lidar_2_linear_x",base_2_lidar_2_linear_x);
        this->get_parameter("base_2_lidar_2_linear_y",base_2_lidar_2_linear_y);
        this->get_parameter("base_2_lidar_2_linear_z",base_2_lidar_2_linear_z);
        this->get_parameter("base_2_lidar_2_orintation_x",base_2_lidar_2_orintation_x);
        this->get_parameter("base_2_lidar_2_orintation_y",base_2_lidar_2_orintation_y);
        this->get_parameter("base_2_lidar_2_orintation_z",base_2_lidar_2_orintation_z);
        this->get_parameter("base_2_lidar_2_orintation_w",base_2_lidar_2_orintation_w);
        this->get_parameter("base_2_lidar_2_angle_x",base_2_lidar_2_angle_x);
        this->get_parameter("base_2_lidar_2_angle_y",base_2_lidar_2_angle_y);
        this->get_parameter("base_2_lidar_2_angle_z",base_2_lidar_2_angle_z);
        this->get_parameter("base_2_lidar_3_linear_x",base_2_lidar_3_linear_x);
        this->get_parameter("base_2_lidar_3_linear_y",base_2_lidar_3_linear_y);
        this->get_parameter("base_2_lidar_3_linear_z",base_2_lidar_3_linear_z);
        this->get_parameter("base_2_lidar_3_orintation_x",base_2_lidar_3_orintation_x);
        this->get_parameter("base_2_lidar_3_orintation_y",base_2_lidar_3_orintation_y);
        this->get_parameter("base_2_lidar_3_orintation_z",base_2_lidar_3_orintation_z);
        this->get_parameter("base_2_lidar_3_orintation_w",base_2_lidar_3_orintation_w);
        this->get_parameter("base_2_lidar_3_angle_x",base_2_lidar_3_angle_x);
        this->get_parameter("base_2_lidar_3_angle_y",base_2_lidar_3_angle_y);
        this->get_parameter("base_2_lidar_3_angle_z",base_2_lidar_3_angle_z);
        this->get_parameter("base_2_lidar_4_linear_x",base_2_lidar_4_linear_x);
        this->get_parameter("base_2_lidar_4_linear_y",base_2_lidar_4_linear_y);
        this->get_parameter("base_2_lidar_4_linear_z",base_2_lidar_4_linear_z);
        this->get_parameter("base_2_lidar_4_orintation_x",base_2_lidar_4_orintation_x);
        this->get_parameter("base_2_lidar_4_orintation_y",base_2_lidar_4_orintation_y);
        this->get_parameter("base_2_lidar_4_orintation_z",base_2_lidar_4_orintation_z);
        this->get_parameter("base_2_lidar_4_orintation_w",base_2_lidar_4_orintation_w);
        this->get_parameter("base_2_lidar_4_angle_x",base_2_lidar_4_angle_x);
        this->get_parameter("base_2_lidar_4_angle_y",base_2_lidar_4_angle_y);
        this->get_parameter("base_2_lidar_4_angle_z",base_2_lidar_4_angle_z);
        this->get_parameter("base_2_lidar_5_linear_x",base_2_lidar_5_linear_x);
        this->get_parameter("base_2_lidar_5_linear_y",base_2_lidar_5_linear_y);
        this->get_parameter("base_2_lidar_5_linear_z",base_2_lidar_5_linear_z);
        this->get_parameter("base_2_lidar_5_orintation_x",base_2_lidar_5_orintation_x);
        this->get_parameter("base_2_lidar_5_orintation_y",base_2_lidar_5_orintation_y);
        this->get_parameter("base_2_lidar_5_orintation_z",base_2_lidar_5_orintation_z);
        this->get_parameter("base_2_lidar_5_orintation_w",base_2_lidar_5_orintation_w);
        this->get_parameter("base_2_lidar_5_angle_x",base_2_lidar_5_angle_x);
        this->get_parameter("base_2_lidar_5_angle_y",base_2_lidar_5_angle_y);
        this->get_parameter("base_2_lidar_5_angle_z",base_2_lidar_5_angle_z);
        this->get_parameter("base_2_lidar_6_linear_x",base_2_lidar_6_linear_x);
        this->get_parameter("base_2_lidar_6_linear_y",base_2_lidar_6_linear_y);
        this->get_parameter("base_2_lidar_6_linear_z",base_2_lidar_6_linear_z);
        this->get_parameter("base_2_lidar_6_orintation_x",base_2_lidar_6_orintation_x);
        this->get_parameter("base_2_lidar_6_orintation_y",base_2_lidar_6_orintation_y);
        this->get_parameter("base_2_lidar_6_orintation_z",base_2_lidar_6_orintation_z);
        this->get_parameter("base_2_lidar_6_orintation_w",base_2_lidar_6_orintation_w);
        this->get_parameter("base_2_lidar_6_angle_x",base_2_lidar_6_angle_x);
        this->get_parameter("base_2_lidar_6_angle_y",base_2_lidar_6_angle_y);
        this->get_parameter("base_2_lidar_6_angle_z",base_2_lidar_6_angle_z);
        this->get_parameter("base_2_ins_linear_x",base_2_ins_linear_x);
        this->get_parameter("base_2_ins_linear_y",base_2_ins_linear_y);
        this->get_parameter("base_2_ins_linear_z",base_2_ins_linear_z);
        this->get_parameter("base_2_ins_orintation_x",base_2_ins_orintation_x);
        this->get_parameter("base_2_ins_orintation_y",base_2_ins_orintation_y);
        this->get_parameter("base_2_ins_orintation_z",base_2_ins_orintation_z);
        this->get_parameter("base_2_ins_orintation_w",base_2_ins_orintation_w);
        this->get_parameter("base_2_ins_angle_x",base_2_ins_angle_x);
        this->get_parameter("base_2_ins_angle_y",base_2_ins_angle_y);
        this->get_parameter("base_2_ins_angle_z",base_2_ins_angle_z);
        this->get_parameter("base_2_radar_linear_x",base_2_radar_linear_x);
        this->get_parameter("base_2_radar_linear_y",base_2_radar_linear_y);
        this->get_parameter("base_2_radar_linear_z",base_2_radar_linear_z);
        this->get_parameter("base_2_radar_orintation_x",base_2_radar_orintation_x);
        this->get_parameter("base_2_radar_orintation_y",base_2_radar_orintation_y);
        this->get_parameter("base_2_radar_orintation_z",base_2_radar_orintation_z);
        this->get_parameter("base_2_radar_orintation_w",base_2_radar_orintation_w);
        this->get_parameter("base_2_radar_angle_x",base_2_radar_angle_x);
        this->get_parameter("base_2_radar_angle_y",base_2_radar_angle_y);
        this->get_parameter("base_2_radar_angle_z",base_2_radar_angle_z);


        /*定义发布者*/
        if(lidar_main_tf2_flag){lidar_main_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_imu_tf2_flag){lidar_imu_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_1_tf2_flag){lidar_1_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_2_tf2_flag){lidar_2_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_3_tf2_flag){lidar_3_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_4_tf2_flag){lidar_4_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_5_tf2_flag){lidar_5_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(lidar_6_tf2_flag){lidar_6_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(ins_tf2_flag){ins_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}
        if(radar_tf2_flag){radar_tf2_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);}

        /*创建定时器，500ms为周期，定时发布*/
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SensorTF2Publisher::make_transforms, this));
    }

private:
    bool lidar_main_tf2_flag;
    bool lidar_imu_tf2_flag;
    bool lidar_1_tf2_flag;
    bool lidar_2_tf2_flag;
    bool lidar_3_tf2_flag;
    bool lidar_4_tf2_flag;
    bool lidar_5_tf2_flag;
    bool lidar_6_tf2_flag;
    bool ins_tf2_flag;
    bool radar_tf2_flag;
    std::string world_frame_id;
    std::string lidar_main_frame_id;
    std::string lidar_imu_frame_id;
    std::string lidar_1_frame_id;
    std::string lidar_2_frame_id;
    std::string lidar_3_frame_id;
    std::string lidar_4_frame_id;
    std::string lidar_5_frame_id;
    std::string lidar_6_frame_id;
    std::string ins_frame_id;
    std::string radar_frame_id;
    bool lidar_main_orintation_flag;
    bool lidar_imu_orintation_flag;
    bool lidar_1_orintation_flag;
    bool lidar_2_orintation_flag;
    bool lidar_3_orintation_flag;
    bool lidar_4_orintation_flag;
    bool lidar_5_orintation_flag;
    bool lidar_6_orintation_flag;
    bool ins_orintation_flag;
    bool radar_orintation_flag;
    double base_2_lidar_main_linear_x,base_2_lidar_main_linear_y,base_2_lidar_main_linear_z;
    double base_2_lidar_main_orintation_x,base_2_lidar_main_orintation_y,base_2_lidar_main_orintation_z,base_2_lidar_main_orintation_w;
    double base_2_lidar_main_angle_x,base_2_lidar_main_angle_y,base_2_lidar_main_angle_z;
    double base_2_lidar_imu_linear_x,base_2_lidar_imu_linear_y,base_2_lidar_imu_linear_z;
    double base_2_lidar_imu_orintation_x,base_2_lidar_imu_orintation_y,base_2_lidar_imu_orintation_z,base_2_lidar_imu_orintation_w;
    double base_2_lidar_imu_angle_x,base_2_lidar_imu_angle_y,base_2_lidar_imu_angle_z;
    double base_2_lidar_1_linear_x,base_2_lidar_1_linear_y,base_2_lidar_1_linear_z;
    double base_2_lidar_1_orintation_x,base_2_lidar_1_orintation_y,base_2_lidar_1_orintation_z,base_2_lidar_1_orintation_w;
    double base_2_lidar_1_angle_x,base_2_lidar_1_angle_y,base_2_lidar_1_angle_z;
    double base_2_lidar_2_linear_x,base_2_lidar_2_linear_y,base_2_lidar_2_linear_z;
    double base_2_lidar_2_orintation_x,base_2_lidar_2_orintation_y,base_2_lidar_2_orintation_z,base_2_lidar_2_orintation_w;
    double base_2_lidar_2_angle_x,base_2_lidar_2_angle_y,base_2_lidar_2_angle_z;
    double base_2_lidar_3_linear_x,base_2_lidar_3_linear_y,base_2_lidar_3_linear_z;
    double base_2_lidar_3_orintation_x,base_2_lidar_3_orintation_y,base_2_lidar_3_orintation_z,base_2_lidar_3_orintation_w;
    double base_2_lidar_3_angle_x,base_2_lidar_3_angle_y,base_2_lidar_3_angle_z;
    double base_2_lidar_4_linear_x,base_2_lidar_4_linear_y,base_2_lidar_4_linear_z;
    double base_2_lidar_4_orintation_x,base_2_lidar_4_orintation_y,base_2_lidar_4_orintation_z,base_2_lidar_4_orintation_w;
    double base_2_lidar_4_angle_x,base_2_lidar_4_angle_y,base_2_lidar_4_angle_z;
    double base_2_lidar_5_linear_x,base_2_lidar_5_linear_y,base_2_lidar_5_linear_z;
    double base_2_lidar_5_orintation_x,base_2_lidar_5_orintation_y,base_2_lidar_5_orintation_z,base_2_lidar_5_orintation_w;
    double base_2_lidar_5_angle_x,base_2_lidar_5_angle_y,base_2_lidar_5_angle_z;
    double base_2_lidar_6_linear_x,base_2_lidar_6_linear_y,base_2_lidar_6_linear_z;
    double base_2_lidar_6_orintation_x,base_2_lidar_6_orintation_y,base_2_lidar_6_orintation_z,base_2_lidar_6_orintation_w;
    double base_2_lidar_6_angle_x,base_2_lidar_6_angle_y,base_2_lidar_6_angle_z;
    double base_2_ins_linear_x,base_2_ins_linear_y,base_2_ins_linear_z;
    double base_2_ins_orintation_x,base_2_ins_orintation_y,base_2_ins_orintation_z,base_2_ins_orintation_w;
    double base_2_ins_angle_x,base_2_ins_angle_y,base_2_ins_angle_z;
    double base_2_radar_linear_x,base_2_radar_linear_y,base_2_radar_linear_z;
    double base_2_radar_orintation_x,base_2_radar_orintation_y,base_2_radar_orintation_z,base_2_radar_orintation_w;
    double base_2_radar_angle_x,base_2_radar_angle_y,base_2_radar_angle_z;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_main_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_imu_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_1_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_2_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_3_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_4_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_5_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> lidar_6_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> ins_tf2_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> radar_tf2_publisher_;
    tf2::Quaternion q_;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;

    void make_transforms()
    {
        rclcpp::Time now;
        if(lidar_main_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_main_t_;
            lidar_main_t_.header.stamp = now;
            lidar_main_t_.header.frame_id = world_frame_id;
            lidar_main_t_.child_frame_id = lidar_main_frame_id;
            lidar_main_t_.transform.translation.x = base_2_lidar_main_linear_x;
            lidar_main_t_.transform.translation.y = base_2_lidar_main_linear_y;
            lidar_main_t_.transform.translation.z = base_2_lidar_main_linear_z;
            if(lidar_main_orintation_flag){
                lidar_main_t_.transform.rotation.x = base_2_lidar_main_orintation_x;
                lidar_main_t_.transform.rotation.y = base_2_lidar_main_orintation_y;
                lidar_main_t_.transform.rotation.z = base_2_lidar_main_orintation_z;
                lidar_main_t_.transform.rotation.w = base_2_lidar_main_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_main_angle_x,base_2_lidar_main_angle_y,base_2_lidar_main_angle_z);
                lidar_main_t_.transform.rotation.x = q_.x();
                lidar_main_t_.transform.rotation.y = q_.y();
                lidar_main_t_.transform.rotation.z = q_.z();
                lidar_main_t_.transform.rotation.w = q_.w();
            }
            lidar_main_tf2_publisher_->sendTransform(lidar_main_t_);
        }
        if(lidar_imu_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_imu_t_;
            lidar_imu_t_.header.stamp = now;
            lidar_imu_t_.header.frame_id = world_frame_id;
            lidar_imu_t_.child_frame_id = lidar_imu_frame_id;
            lidar_imu_t_.transform.translation.x = base_2_lidar_imu_linear_x;
            lidar_imu_t_.transform.translation.y = base_2_lidar_imu_linear_y;
            lidar_imu_t_.transform.translation.z = base_2_lidar_imu_linear_z;
            if(lidar_imu_orintation_flag){
                lidar_imu_t_.transform.rotation.x = base_2_lidar_imu_orintation_x;
                lidar_imu_t_.transform.rotation.y = base_2_lidar_imu_orintation_y;
                lidar_imu_t_.transform.rotation.z = base_2_lidar_imu_orintation_z;
                lidar_imu_t_.transform.rotation.w = base_2_lidar_imu_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_imu_angle_x,base_2_lidar_imu_angle_y,base_2_lidar_imu_angle_z);
                lidar_imu_t_.transform.rotation.x = q_.x();
                lidar_imu_t_.transform.rotation.y = q_.y();
                lidar_imu_t_.transform.rotation.z = q_.z();
                lidar_imu_t_.transform.rotation.w = q_.w();
            }
            lidar_imu_tf2_publisher_->sendTransform(lidar_imu_t_);
        }
        if(lidar_1_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_1_t_;
            lidar_1_t_.header.stamp = now;
            lidar_1_t_.header.frame_id = world_frame_id;
            lidar_1_t_.child_frame_id = lidar_1_frame_id;
            lidar_1_t_.transform.translation.x = base_2_lidar_1_linear_x;
            lidar_1_t_.transform.translation.y = base_2_lidar_1_linear_y;
            lidar_1_t_.transform.translation.z = base_2_lidar_1_linear_z;
            if(lidar_1_orintation_flag){
                lidar_1_t_.transform.rotation.x = base_2_lidar_1_orintation_x;
                lidar_1_t_.transform.rotation.y = base_2_lidar_1_orintation_y;
                lidar_1_t_.transform.rotation.z = base_2_lidar_1_orintation_z;
                lidar_1_t_.transform.rotation.w = base_2_lidar_1_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_1_angle_x,base_2_lidar_1_angle_y,base_2_lidar_1_angle_z);
                lidar_1_t_.transform.rotation.x = q_.x();
                lidar_1_t_.transform.rotation.y = q_.y();
                lidar_1_t_.transform.rotation.z = q_.z();
                lidar_1_t_.transform.rotation.w = q_.w();
            }
            lidar_1_tf2_publisher_->sendTransform(lidar_1_t_);
        }
        if(lidar_2_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_2_t_;
            lidar_2_t_.header.stamp = now;
            lidar_2_t_.header.frame_id = world_frame_id;
            lidar_2_t_.child_frame_id = lidar_2_frame_id;
            lidar_2_t_.transform.translation.x = base_2_lidar_2_linear_x;
            lidar_2_t_.transform.translation.y = base_2_lidar_2_linear_y;
            lidar_2_t_.transform.translation.z = base_2_lidar_2_linear_z;
            if(lidar_2_orintation_flag){
                lidar_2_t_.transform.rotation.x = base_2_lidar_2_orintation_x;
                lidar_2_t_.transform.rotation.y = base_2_lidar_2_orintation_y;
                lidar_2_t_.transform.rotation.z = base_2_lidar_2_orintation_z;
                lidar_2_t_.transform.rotation.w = base_2_lidar_2_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_2_angle_x,base_2_lidar_2_angle_y,base_2_lidar_2_angle_z);
                lidar_2_t_.transform.rotation.x = q_.x();
                lidar_2_t_.transform.rotation.y = q_.y();
                lidar_2_t_.transform.rotation.z = q_.z();
                lidar_2_t_.transform.rotation.w = q_.w();
            }
            lidar_2_tf2_publisher_->sendTransform(lidar_2_t_);
        }
        if(lidar_3_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_3_t_;
            lidar_3_t_.header.stamp = now;
            lidar_3_t_.header.frame_id = world_frame_id;
            lidar_3_t_.child_frame_id = lidar_3_frame_id;
            lidar_3_t_.transform.translation.x = base_2_lidar_3_linear_x;
            lidar_3_t_.transform.translation.y = base_2_lidar_3_linear_y;
            lidar_3_t_.transform.translation.z = base_2_lidar_3_linear_z;
            if(lidar_3_orintation_flag){
                lidar_3_t_.transform.rotation.x = base_2_lidar_3_orintation_x;
                lidar_3_t_.transform.rotation.y = base_2_lidar_3_orintation_y;
                lidar_3_t_.transform.rotation.z = base_2_lidar_3_orintation_z;
                lidar_3_t_.transform.rotation.w = base_2_lidar_3_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_3_angle_x,base_2_lidar_3_angle_y,base_2_lidar_3_angle_z);
                lidar_3_t_.transform.rotation.x = q_.x();
                lidar_3_t_.transform.rotation.y = q_.y();
                lidar_3_t_.transform.rotation.z = q_.z();
                lidar_3_t_.transform.rotation.w = q_.w();
            }
            lidar_3_tf2_publisher_->sendTransform(lidar_3_t_);
        }
        if(lidar_4_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_4_t_;
            lidar_4_t_.header.stamp = now;
            lidar_4_t_.header.frame_id = world_frame_id;
            lidar_4_t_.child_frame_id = lidar_4_frame_id;
            lidar_4_t_.transform.translation.x = base_2_lidar_4_linear_x;
            lidar_4_t_.transform.translation.y = base_2_lidar_4_linear_y;
            lidar_4_t_.transform.translation.z = base_2_lidar_4_linear_z;
            if(lidar_4_orintation_flag){
                lidar_4_t_.transform.rotation.x = base_2_lidar_4_orintation_x;
                lidar_4_t_.transform.rotation.y = base_2_lidar_4_orintation_y;
                lidar_4_t_.transform.rotation.z = base_2_lidar_4_orintation_z;
                lidar_4_t_.transform.rotation.w = base_2_lidar_4_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_4_angle_x,base_2_lidar_4_angle_y,base_2_lidar_4_angle_z);
                lidar_4_t_.transform.rotation.x = q_.x();
                lidar_4_t_.transform.rotation.y = q_.y();
                lidar_4_t_.transform.rotation.z = q_.z();
                lidar_4_t_.transform.rotation.w = q_.w();
            }
            lidar_4_tf2_publisher_->sendTransform(lidar_4_t_);
        }
        if(lidar_5_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_5_t_;
            lidar_5_t_.header.stamp = now;
            lidar_5_t_.header.frame_id = world_frame_id;
            lidar_5_t_.child_frame_id = lidar_5_frame_id;
            lidar_5_t_.transform.translation.x = base_2_lidar_5_linear_x;
            lidar_5_t_.transform.translation.y = base_2_lidar_5_linear_y;
            lidar_5_t_.transform.translation.z = base_2_lidar_5_linear_z;
            if(lidar_5_orintation_flag){
                lidar_5_t_.transform.rotation.x = base_2_lidar_5_orintation_x;
                lidar_5_t_.transform.rotation.y = base_2_lidar_5_orintation_y;
                lidar_5_t_.transform.rotation.z = base_2_lidar_5_orintation_z;
                lidar_5_t_.transform.rotation.w = base_2_lidar_5_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_5_angle_x,base_2_lidar_5_angle_y,base_2_lidar_5_angle_z);
                lidar_5_t_.transform.rotation.x = q_.x();
                lidar_5_t_.transform.rotation.y = q_.y();
                lidar_5_t_.transform.rotation.z = q_.z();
                lidar_5_t_.transform.rotation.w = q_.w();
            }
            lidar_5_tf2_publisher_->sendTransform(lidar_5_t_);
        }
        if(lidar_6_tf2_flag){
            geometry_msgs::msg::TransformStamped lidar_6_t_;
            lidar_6_t_.header.stamp = now;
            lidar_6_t_.header.frame_id = world_frame_id;
            lidar_6_t_.child_frame_id = lidar_6_frame_id;
            lidar_6_t_.transform.translation.x = base_2_lidar_6_linear_x;
            lidar_6_t_.transform.translation.y = base_2_lidar_6_linear_y;
            lidar_6_t_.transform.translation.z = base_2_lidar_6_linear_z;
            if(lidar_1_orintation_flag){
                lidar_6_t_.transform.rotation.x = base_2_lidar_6_orintation_x;
                lidar_6_t_.transform.rotation.y = base_2_lidar_6_orintation_y;
                lidar_6_t_.transform.rotation.z = base_2_lidar_6_orintation_z;
                lidar_6_t_.transform.rotation.w = base_2_lidar_6_orintation_w;
            }else{
                q_.setRPY(base_2_lidar_6_angle_x,base_2_lidar_6_angle_y,base_2_lidar_6_angle_z);
                lidar_6_t_.transform.rotation.x = q_.x();
                lidar_6_t_.transform.rotation.y = q_.y();
                lidar_6_t_.transform.rotation.z = q_.z();
                lidar_6_t_.transform.rotation.w = q_.w();
            }
            lidar_6_tf2_publisher_->sendTransform(lidar_6_t_);
        }
        if(ins_tf2_flag){
            geometry_msgs::msg::TransformStamped ins_t_;
            ins_t_.header.stamp = now;
            ins_t_.header.frame_id = world_frame_id;
            ins_t_.child_frame_id = ins_frame_id;
            ins_t_.transform.translation.x = base_2_ins_linear_x;
            ins_t_.transform.translation.y = base_2_ins_linear_y;
            ins_t_.transform.translation.z = base_2_ins_linear_z;
            if(ins_orintation_flag){
                ins_t_.transform.rotation.x = base_2_ins_orintation_x;
                ins_t_.transform.rotation.y = base_2_ins_orintation_y;
                ins_t_.transform.rotation.z = base_2_ins_orintation_z;
                ins_t_.transform.rotation.w = base_2_ins_orintation_w;
            }else{
                q_.setRPY(base_2_ins_angle_x,base_2_ins_angle_y,base_2_ins_angle_z);
                ins_t_.transform.rotation.x = q_.x();
                ins_t_.transform.rotation.y = q_.y();
                ins_t_.transform.rotation.z = q_.z();
                ins_t_.transform.rotation.w = q_.w();
            }
            ins_tf2_publisher_->sendTransform(ins_t_);
        }
        if(radar_tf2_flag){
            geometry_msgs::msg::TransformStamped radar_t_;
            radar_t_.header.stamp = now;
            radar_t_.header.frame_id = world_frame_id;
            radar_t_.child_frame_id = radar_frame_id;
            radar_t_.transform.translation.x = base_2_radar_linear_x;
            radar_t_.transform.translation.y = base_2_radar_linear_y;
            radar_t_.transform.translation.z = base_2_radar_linear_z;
            if(radar_orintation_flag){
                radar_t_.transform.rotation.x = base_2_radar_orintation_x;
                radar_t_.transform.rotation.y = base_2_radar_orintation_y;
                radar_t_.transform.rotation.z = base_2_radar_orintation_z;
                radar_t_.transform.rotation.w = base_2_radar_orintation_w;
            }else{
                q_.setRPY(base_2_radar_angle_x,base_2_radar_angle_y,base_2_radar_angle_z);
                radar_t_.transform.rotation.x = q_.x();
                radar_t_.transform.rotation.y = q_.y();
                radar_t_.transform.rotation.z = q_.z();
                radar_t_.transform.rotation.w = q_.w();
            }
            radar_tf2_publisher_->sendTransform(radar_t_);
        }
        RCLCPP_INFO(this->get_logger(),"publish tf2 transform data once.");
    }
};

int main(int argc, char * argv[])
{
  /* initialize node */
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorTF2Publisher>("sensor_tf2_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
