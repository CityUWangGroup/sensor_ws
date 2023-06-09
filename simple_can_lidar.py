import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # chassis arg
    chassis_can_name_arg = DeclareLaunchArgument('chassis_can_name',default_value=TextSubstitution(text='can1'))
    speed_coefficient_arg = DeclareLaunchArgument('speed_coefficient',default_value=TextSubstitution(text='10.0'))
    lr_signal_arg = DeclareLaunchArgument('lr_signal',default_value=TextSubstitution(text='1'))
    control_methon_all_flag_arg = DeclareLaunchArgument('control_methon_all_flag',default_value=TextSubstitution(text='true'))
    control_methon_ssb_flag_arg = DeclareLaunchArgument('control_methon_ssb_flag',default_value=TextSubstitution(text='false'))
    speed_subscribe_topic_arg = DeclareLaunchArgument('speed_subscribe_topic',default_value=TextSubstitution(text='/Control_msgs/speed'))
    steerAngle_subscribe_topic_arg = DeclareLaunchArgument('steerAngle_subscribe_topic',default_value=TextSubstitution(text='/Control_msgs/steerAngle'))
    brake_subscribe_topic_arg = DeclareLaunchArgument('brake_subscribe_topic',default_value=TextSubstitution(text='/Control_msgs/brakeRate'))
    all_subscribe_topic_arg = DeclareLaunchArgument('all_subscribe_topic',default_value=TextSubstitution(text='/Control_msgs/chassisCan'))
    rawData_publisher_topic_arg = DeclareLaunchArgument('rawData_publisher_topic',default_value=TextSubstitution(text='/Sensor_msgs/chassis_can/rawData'))
    speed_publisher_topic_arg = DeclareLaunchArgument('speed_publisher_topic',default_value=TextSubstitution(text='/Sensor_msgs/chassis_can/speed'))
    steerAngle_publisher_topic_arg = DeclareLaunchArgument('steerAngle_publisher_topic',default_value=TextSubstitution(text='/Sensor_msgs/chassis_can/steerAngle'))
    brake_publisher_topic_arg = DeclareLaunchArgument('brake_publisher_topic',default_value=TextSubstitution(text='/Sensor_msgs/chassis_can/brakeRate'))
    soc_publisher_topic_arg = DeclareLaunchArgument('soc_publisher_topic',default_value=TextSubstitution(text='/Sensor_msgs/chassis_can/SOC'))
    all_publisher_topic_arg = DeclareLaunchArgument('all_publisher_topic',default_value=TextSubstitution(text='/Sensor_msgs/chassis_can/chassisCan'))

    # ins arg
    baudrate_INS_arg = DeclareLaunchArgument('baudrate_INS',default_value=TextSubstitution(text='921600'))
    INS_ComName_arg = DeclareLaunchArgument('INS_ComName',default_value=TextSubstitution(text='/dev/insCOM0'))
    ins_frame_id_arg = DeclareLaunchArgument('ins_frame_id',default_value=TextSubstitution(text='/base_link'))
    
    # lidar filter arg
    lidar_origin_data_topic_arg = DeclareLaunchArgument('lidar_origin_data_topic',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_frame_id_arg = DeclareLaunchArgument('lidar_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_filtered_data_topic_arg = DeclareLaunchArgument('lidar_filtered_data_topic',default_value=TextSubstitution(text='/lidar_front/filtered_xyzi'))
    lidar_filtering_data_topic_arg = DeclareLaunchArgument('lidar_filtering_data_topic',default_value=TextSubstitution(text='/lidar_front/filtering_xyzi'))
    a1_arg = DeclareLaunchArgument('a1',default_value=TextSubstitution(text='0.38')) 
    b1_arg = DeclareLaunchArgument('b1',default_value=TextSubstitution(text='2.257')) 
    l_l_arg = DeclareLaunchArgument('l_l',default_value=TextSubstitution(text='0.56')) 
    l_r_arg = DeclareLaunchArgument('l_r',default_value=TextSubstitution(text='0.56')) 
    h1_arg = DeclareLaunchArgument('h1',default_value=TextSubstitution(text='-1.8')) 
    h2_arg = DeclareLaunchArgument('h2',default_value=TextSubstitution(text='0.15')) 
    lidar_autoware_flag_arg = DeclareLaunchArgument('lidar_autoware_flag',default_value=TextSubstitution(text='true')) 
    lidar_autoware_topic_arg = DeclareLaunchArgument('lidar_autoware_topic',default_value=TextSubstitution(text='/sensing/lidar/top/pointcloud_raw_ex')) 
    lidar_autoware_frame_id_arg = DeclareLaunchArgument('lidar_autoware_frame_id',default_value=TextSubstitution(text='sensor_kit_base_link')) 

    # camera launch
    camera_usb_node = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join(
    	    get_package_share_directory('camera_usb_pkg'),'launch'),
    	    '/camera_usb_launch.py'])
    	)
    	
    # rslidar launch
    rslidar_sdk_node = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join(
    	    get_package_share_directory('rslidar_sdk'),'launch'),
    	    '/rslidar_simple_launch.py'])
    	)
    
    return LaunchDescription([
        chassis_can_name_arg,
        speed_coefficient_arg,
        lr_signal_arg,
        control_methon_all_flag_arg,
        control_methon_ssb_flag_arg,
        speed_subscribe_topic_arg,
        steerAngle_subscribe_topic_arg,
        brake_subscribe_topic_arg,
        all_subscribe_topic_arg,
        rawData_publisher_topic_arg,
        speed_publisher_topic_arg,
        steerAngle_publisher_topic_arg,
        brake_publisher_topic_arg,
        soc_publisher_topic_arg,
        all_publisher_topic_arg,
        Node(
            package='chassis_can_pkg',
            executable='chassis_can_driver_node',
            name='chassis_can',
            parameters=[{
                'chassis_can_name': LaunchConfiguration('chassis_can_name'),
                'speed_coefficient': LaunchConfiguration('speed_coefficient'),
                'lr_signal': LaunchConfiguration('lr_signal'),
                'control_methon_all_flag': LaunchConfiguration('control_methon_all_flag'),
                'control_methon_ssb_flag': LaunchConfiguration('control_methon_ssb_flag'),
                'speed_subscribe_topic': LaunchConfiguration('speed_subscribe_topic'),
                'steerAngle_subscribe_topic': LaunchConfiguration('steerAngle_subscribe_topic'),
                'brake_subscribe_topic': LaunchConfiguration('brake_subscribe_topic'),
                'all_subscribe_topic': LaunchConfiguration('all_subscribe_topic'),
                'rawData_publisher_topic': LaunchConfiguration('rawData_publisher_topic'),
                'speed_publisher_topic': LaunchConfiguration('speed_publisher_topic'),
                'steerAngle_publisher_topic': LaunchConfiguration('steerAngle_publisher_topic'),
                'brake_publisher_topic': LaunchConfiguration('brake_publisher_topic'),
                'soc_publisher_topic': LaunchConfiguration('soc_publisher_topic'),
                'all_publisher_topic': LaunchConfiguration('all_publisher_topic')
            }]
        ),
        baudrate_INS_arg,
        INS_ComName_arg,
        ins_frame_id_arg,
        Node(
            package='ins_com_pkg',
            executable='ins_com_driver_node',
            name='ins_com',
            parameters=[{
                'baudrate_INS': LaunchConfiguration('baudrate_INS'),
                'INS_ComName': LaunchConfiguration('INS_ComName'),
                'ins_frame_id': LaunchConfiguration('ins_frame_id')
            }]
        ),
        lidar_origin_data_topic_arg,
        lidar_frame_id_arg,
        lidar_filtered_data_topic_arg,
        lidar_filtering_data_topic_arg,
        a1_arg,
        b1_arg,
        l_l_arg,
        l_r_arg,
        h1_arg,
        h2_arg,
        lidar_autoware_flag_arg,
        lidar_autoware_topic_arg,
        lidar_autoware_frame_id_arg,
        Node(
            package='lidar_filter_pkg',
            executable='lidar_filter_node',
            name='lidar_filter',
            parameters=[{
                'lidar_origin_data_topic': LaunchConfiguration('lidar_origin_data_topic'),
                'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
                'lidar_filtered_data_topic': LaunchConfiguration('lidar_filtered_data_topic'),
                'lidar_filtering_data_topic': LaunchConfiguration('lidar_filtering_data_topic'),
                'a1': LaunchConfiguration('a1'),
                'b1': LaunchConfiguration('b1'),
                'l_l': LaunchConfiguration('l_l'),
                'l_r': LaunchConfiguration('l_r'),
                'h1': LaunchConfiguration('h1'),
                'h2': LaunchConfiguration('h2'),
                'lidar_autoware_flag': LaunchConfiguration('lidar_autoware_flag'),
                'lidar_autoware_topic': LaunchConfiguration('lidar_autoware_topic'),
                'lidar_autoware_frame_id': LaunchConfiguration('lidar_autoware_frame_id')
            }]
        ),
        camera_usb_node,
        rslidar_sdk_node
        ])
