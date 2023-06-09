#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include <math.h>

/*驱动六台UART自动发布的超声波雷达中转机*/
class Lidar_flag : public rclcpp::Node{
 public:
  // 构造函数,有一个参数为节点名称
  explicit Lidar_flag(std::string name) : Node(name){

    /*声明参数*/
    this->declare_parameter("lidar_origin_data_topic", "/lidar_front/points_xyzi");
    this->declare_parameter("lidar_frame_id", "/lidar_front/points_xyzi");
    this->declare_parameter("lidar_filtered_data_topic", "/lidar_front/filtered_xyzi");
    this->declare_parameter("lidar_filtering_data_topic", "/lidar_front/filtering_xyzi");
    this->declare_parameter("a1", 0.38); 
    this->declare_parameter("b1", 2.257); 
    this->declare_parameter("l_l", 0.56); 
    this->declare_parameter("l_r", 0.56); 
    this->declare_parameter("h1", -1.8); 
    this->declare_parameter("h2", 0.15); 
    this->declare_parameter("lidar_autoware_flag", false); 
    this->declare_parameter("lidar_autoware_topic", "/sensing/lidar/top/pointcloud_raw_ex"); 
    this->declare_parameter("lidar_autoware_frame_id", "sensor_kit_base_link"); 
    
    /*获取参数*/
    this->get_parameter("lidar_origin_data_topic", lidar_origin_data_topic);
    this->get_parameter("lidar_frame_id", lidar_frame_id);
    this->get_parameter("lidar_filtered_data_topic", lidar_filtered_data_topic);
    this->get_parameter("lidar_filtering_data_topic", lidar_filtering_data_topic);
    this->get_parameter("a1", a1);
    this->get_parameter("b1", b1);
    this->get_parameter("l_l", l_l);
    this->get_parameter("l_r", l_r);
    this->get_parameter("h1", h1);
    this->get_parameter("h2", h2);
    this->get_parameter("lidar_autoware_flag", lidar_autoware_flag);
    this->get_parameter("lidar_autoware_topic", lidar_autoware_topic);
    this->get_parameter("lidar_autoware_frame_id", lidar_autoware_frame_id);
    
    std::stringstream ss;
    ss.str("");
    ss << "\nlidar_origin_data_topic = " << lidar_origin_data_topic;
    ss << "\nlidar_frame_id = " << lidar_frame_id;
    ss << "\nlidar_filtered_data_topic = " << lidar_filtered_data_topic;
    ss << "\nlidar_filtering_data_topic = " << lidar_filtering_data_topic;
    ss << "\na1 = " << a1;
    ss << "\nb1 = " << b1;
    ss << "\nl_l = " << l_l;
    ss << "\nl_r = " << l_r;
    ss << "\nh1 = " << h1;
    ss << "\nh2 = " << h2;
    ss << "\nlidar_autoware_flag = " << lidar_autoware_flag;
    ss << "\nlidar_autoware_topic = " << lidar_autoware_topic;
    ss << "\nlidar_autoware_frame_id = " << lidar_autoware_frame_id;
    std::string print_str = ss.str();
    RCLCPP_INFO(this->get_logger(), print_str.c_str());
    
    /*创建订阅、发布信息*/
    lidar_data_subscribe_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_origin_data_topic, 10, std::bind(&Lidar_flag::pointCloud2ToZ, this, std::placeholders::_1));
    lidar_data_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_filtered_data_topic, 10);
    lidar_filtering_data_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_filtering_data_topic, 10);
    if(lidar_autoware_flag){
    	lidar_autoware_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_autoware_topic,10);
    }
  }

 private:
    /*声明参数*/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_data_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_data_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_filtering_data_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_autoware_publisher_;
    std::string lidar_origin_data_topic;
    std::string lidar_filtered_data_topic;
    std::string lidar_filtering_data_topic;
    std::string lidar_frame_id;
    float a1=0.75;
    float b1=2.95;
    float l_l=1.1;
    float l_r=1.1;
    float h1=-1.6;
    float h2=0.2;
    bool lidar_autoware_flag;
    std::string lidar_autoware_topic;
    std::string lidar_autoware_frame_id;
    bool point_state = false;

    void pointCloud2ToZ(const sensor_msgs::msg::PointCloud2 &msg)
    {
        sensor_msgs::msg::PointCloud p;
        sensor_msgs::msg::PointCloud p_new;
        sensor_msgs::msg::PointCloud p_filtered;
        sensor_msgs::msg::PointCloud2 msg_new;
        sensor_msgs::msg::PointCloud2 msg_filtered;
        sensor_msgs::convertPointCloud2ToPointCloud(msg,p);
        int points_sizes = p.points.size();
        
        for(int i=0;i<points_sizes;i++){
            if(p.points[i].z <= h1 || p.points[i].z >= h2)point_state = true;
            else if(p.points[i].x <= -b1 || p.points[i].x >= a1)point_state = true;
            else if(p.points[i].y <= -l_r || p.points[i].y >= l_l)point_state = true;
            else point_state = false;

            if(point_state)p_new.points.push_back(p.points[i]);
            else p_filtered.points.push_back(p.points[i]);
        }
        sensor_msgs::convertPointCloudToPointCloud2(p_new,msg_new);
        msg_new.header = msg.header;
        lidar_data_publisher_->publish(msg_new);
        sensor_msgs::convertPointCloudToPointCloud2(p_filtered,msg_filtered);
        msg_filtered.header = msg.header;
        lidar_filtering_data_publisher_->publish(msg_filtered);
        if(lidar_autoware_flag){
            sensor_msgs::msg::PointCloud2 msg_autoware;
            msg_autoware = msg_new;
            msg_autoware.header.frame_id = lidar_autoware_frame_id;
            lidar_autoware_publisher_->publish(msg_autoware);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Lidar_flag>("lidar_filter_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
