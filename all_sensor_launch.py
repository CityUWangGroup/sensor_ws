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
    control_methon_all_flag_arg = DeclareLaunchArgument('control_methon_all_flag',default_value=TextSubstitution(text='false'))
    control_methon_ssb_flag_arg = DeclareLaunchArgument('control_methon_ssb_flag',default_value=TextSubstitution(text='true'))
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

    # imu arg
    baudrate_IMU_arg = DeclareLaunchArgument('baudrate_IMU',default_value=TextSubstitution(text='230400'))
    IMU_ComName_arg = DeclareLaunchArgument('IMU_ComName',default_value=TextSubstitution(text='/dev/lidarIMU'))
    gravity_acc_arg = DeclareLaunchArgument('gravity_acc',default_value=TextSubstitution(text='9.8'))
    imu_frame_id_arg = DeclareLaunchArgument('imu_frame_id',default_value=TextSubstitution(text='/lidar_front/imu'))

    # ins arg
    baudrate_INS_arg = DeclareLaunchArgument('baudrate_INS',default_value=TextSubstitution(text='921600'))
    INS_ComName_arg = DeclareLaunchArgument('INS_ComName',default_value=TextSubstitution(text='/dev/insCOM0'))
    ins_frame_id_arg = DeclareLaunchArgument('ins_frame_id',default_value=TextSubstitution(text='/ins'))
    
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

    # rslidar launch
    rslidar_sdk_node = IncludeLaunchDescription(
	PythonLaunchDescriptionSource([os.path.join(
    	    get_package_share_directory('rslidar_sdk'),'launch'),
    	    '/rslidar_purepursuit_launch.py'])
    	)

    # sensor tf2 arg
    # lidar_main_tf2_flag_arg = DeclareLaunchArgument('lidar_main_tf2_flag',default_value=TextSubstitution(text='true'))
    # lidar_imu_tf2_flag_arg = DeclareLaunchArgument('lidar_imu_tf2_flag',default_value=TextSubstitution(text='true'))
    # ins_tf2_flag_arg = DeclareLaunchArgument('ins_tf2_flag',default_value=TextSubstitution(text='true'))
    # world_frame_id_arg = DeclareLaunchArgument('world_frame_id',default_value=TextSubstitution(text='/base_link'))
    # lidar_main_frame_id_arg = DeclareLaunchArgument('lidar_main_frame_id',default_value=TextSubstitution(text='/lidar_front'))
    # lidar_imu_frame_id_arg = DeclareLaunchArgument('lidar_imu_frame_id',default_value=TextSubstitution(text='/lidar_front_imu'))
    # ins_frame_id_arg = DeclareLaunchArgument('ins_frame_id',default_value=TextSubstitution(text='/ins'))
    # lidar_main_orintation_flag_arg = DeclareLaunchArgument('lidar_main_orintation_flag',default_value=TextSubstitution(text='true'))
    # lidar_imu_orintation_flag_arg = DeclareLaunchArgument('lidar_imu_orintation_flag',default_value=TextSubstitution(text='true'))
    # ins_orintation_flag_arg = DeclareLaunchArgument('ins_orintation_flag',default_value=TextSubstitution(text='true'))
    # base_2_lidar_main_linear_x_arg = DeclareLaunchArgument('base_2_lidar_main_linear_x',default_value=TextSubstitution(text='1.665'))
    # base_2_lidar_main_linear_y_arg = DeclareLaunchArgument('base_2_lidar_main_linear_y',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_linear_z_arg = DeclareLaunchArgument('base_2_lidar_main_linear_z',default_value=TextSubstitution(text='1.742'))
    # base_2_lidar_main_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_x',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_y',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_z',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_w',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_angle_x_arg = DeclareLaunchArgument('base_2_lidar_main_angle_x',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_angle_y_arg = DeclareLaunchArgument('base_2_lidar_main_angle_y',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_main_angle_z_arg = DeclareLaunchArgument('base_2_lidar_main_angle_z',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_linear_x_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_x',default_value=TextSubstitution(text='1.665'))
    # base_2_lidar_imu_linear_y_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_y',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_linear_z_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_z',default_value=TextSubstitution(text='1.676'))
    # base_2_lidar_imu_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_x',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_y',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_z',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_w',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_angle_x_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_x',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_angle_y_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_y',default_value=TextSubstitution(text='0.0'))
    # base_2_lidar_imu_angle_z_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_z',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_linear_x_arg = DeclareLaunchArgument('base_2_ins_linear_x',default_value=TextSubstitution(text='0.613'))
    # base_2_ins_linear_y_arg = DeclareLaunchArgument('base_2_ins_linear_y',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_linear_z_arg = DeclareLaunchArgument('base_2_ins_linear_z',default_value=TextSubstitution(text='0.3182'))
    # base_2_ins_orintation_x_arg = DeclareLaunchArgument('base_2_ins_orintation_x',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_orintation_y_arg = DeclareLaunchArgument('base_2_ins_orintation_y',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_orintation_z_arg = DeclareLaunchArgument('base_2_ins_orintation_z',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_orintation_w_arg = DeclareLaunchArgument('base_2_ins_orintation_w',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_angle_x_arg = DeclareLaunchArgument('base_2_ins_angle_x',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_angle_y_arg = DeclareLaunchArgument('base_2_ins_angle_y',default_value=TextSubstitution(text='0.0'))
    # base_2_ins_angle_z_arg = DeclareLaunchArgument('base_2_ins_angle_z',default_value=TextSubstitution(text='0.0'))

    # ultrasonic arg
    baudrate_UltrasonicRadar_arg = DeclareLaunchArgument('baudrate_UltrasonicRadar',default_value=TextSubstitution(text='9600'))
    UltrasonicRadar1_ComName_arg = DeclareLaunchArgument('UltrasonicRadar1_ComName',default_value=TextSubstitution(text='/dev/ultraCOM0'))
    UltrasonicRadar2_ComName_arg = DeclareLaunchArgument('UltrasonicRadar2_ComName',default_value=TextSubstitution(text='/dev/ultraCOM1'))
    UltrasonicRadar3_ComName_arg = DeclareLaunchArgument('UltrasonicRadar3_ComName',default_value=TextSubstitution(text='/dev/ultraCOM2'))
    UltrasonicRadar4_ComName_arg = DeclareLaunchArgument('UltrasonicRadar4_ComName',default_value=TextSubstitution(text='/dev/ultraCOM3'))
    UltrasonicRadar5_ComName_arg = DeclareLaunchArgument('UltrasonicRadar5_ComName',default_value=TextSubstitution(text='/dev/ultraCOM4'))
    UltrasonicRadar6_ComName_arg = DeclareLaunchArgument('UltrasonicRadar6_ComName',default_value=TextSubstitution(text='/dev/ultraCOM5'))
    Publish_topic_1_arg = DeclareLaunchArgument('Publish_topic_1',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM01'))
    Publish_topic_2_arg = DeclareLaunchArgument('Publish_topic_2',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM02'))
    Publish_topic_3_arg = DeclareLaunchArgument('Publish_topic_3',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM03'))
    Publish_topic_4_arg = DeclareLaunchArgument('Publish_topic_4',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM04'))
    Publish_topic_5_arg = DeclareLaunchArgument('Publish_topic_5',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM05'))
    Publish_topic_6_arg = DeclareLaunchArgument('Publish_topic_6',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM06'))
    Publish_topic_7_arg = DeclareLaunchArgument('Publish_topic_7',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM07'))
    Publish_topic_8_arg = DeclareLaunchArgument('Publish_topic_8',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM08'))
    Publish_topic_9_arg = DeclareLaunchArgument('Publish_topic_9',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM09'))
    Publish_topic_10_arg = DeclareLaunchArgument('Publish_topic_10',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM10'))
    Publish_topic_11_arg = DeclareLaunchArgument('Publish_topic_11',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM11'))
    Publish_topic_12_arg = DeclareLaunchArgument('Publish_topic_12',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM12'))
    Publish_topic_13_arg = DeclareLaunchArgument('Publish_topic_13',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM13'))
    Publish_topic_14_arg = DeclareLaunchArgument('Publish_topic_14',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM14'))
    Publish_topic_15_arg = DeclareLaunchArgument('Publish_topic_15',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM15'))
    Publish_topic_16_arg = DeclareLaunchArgument('Publish_topic_16',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM16'))
    Publish_topic_17_arg = DeclareLaunchArgument('Publish_topic_17',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM17'))
    Publish_topic_18_arg = DeclareLaunchArgument('Publish_topic_18',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM18'))
    Publish_topic_19_arg = DeclareLaunchArgument('Publish_topic_19',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM19'))
    Publish_topic_20_arg = DeclareLaunchArgument('Publish_topic_20',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM20'))
    
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
        baudrate_IMU_arg,
        IMU_ComName_arg,
        gravity_acc_arg,
        imu_frame_id_arg,
        Node(
            package='imu_com_pkg',
            executable='imu_com_driver_node',
            name='imu_com',
            parameters=[{
                'baudrate_IMU': LaunchConfiguration('baudrate_IMU'),
                'IMU_ComName': LaunchConfiguration('IMU_ComName'),
                'gravity_acc': LaunchConfiguration('gravity_acc'),
                'imu_frame_id': LaunchConfiguration('imu_frame_id')
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
                'h2': LaunchConfiguration('h2')
            }]
        ),
        # lidar_main_tf2_flag_arg,
        # lidar_imu_tf2_flag_arg,
        # ins_tf2_flag_arg,
        # world_frame_id_arg,
        # lidar_main_frame_id_arg,
        # lidar_imu_frame_id_arg,
        # ins_frame_id_arg,
        # lidar_main_orintation_flag_arg,
        # lidar_imu_orintation_flag_arg,
        # ins_orintation_flag_arg,
        # base_2_lidar_main_linear_x_arg,
        # base_2_lidar_main_linear_y_arg,
        # base_2_lidar_main_linear_z_arg,
        # base_2_lidar_main_orintation_x_arg,
        # base_2_lidar_main_orintation_y_arg,
        # base_2_lidar_main_orintation_z_arg,
        # base_2_lidar_main_orintation_w_arg,
        # base_2_lidar_main_angle_x_arg,
        # base_2_lidar_main_angle_y_arg,
        # base_2_lidar_main_angle_z_arg,
        # base_2_lidar_imu_linear_x_arg,
        # base_2_lidar_imu_linear_y_arg,
        # base_2_lidar_imu_linear_z_arg,
        # base_2_lidar_imu_orintation_x_arg,
        # base_2_lidar_imu_orintation_y_arg,
        # base_2_lidar_imu_orintation_z_arg,
        # base_2_lidar_imu_orintation_w_arg,
        # base_2_lidar_imu_angle_x_arg,
        # base_2_lidar_imu_angle_y_arg,
        # base_2_lidar_imu_angle_z_arg,
        # base_2_ins_linear_x_arg,
        # base_2_ins_linear_y_arg,
        # base_2_ins_linear_z_arg,
        # base_2_ins_orintation_x_arg,
        # base_2_ins_orintation_y_arg,
        # base_2_ins_orintation_z_arg,
        # base_2_ins_orintation_w_arg,
        # base_2_ins_angle_x_arg,
        # base_2_ins_angle_y_arg,
        # base_2_ins_angle_z_arg,
        Node(
            package='sensor_tf2_pkg',
            executable='sensor_tf2_node',
            name='sensor_tf2'
            # parameters=[{
            #     'lidar_main_tf2_flag': LaunchConfiguration('lidar_main_tf2_flag'),
            #     'lidar_imu_tf2_flag': LaunchConfiguration('lidar_imu_tf2_flag'),
            #     'ins_tf2_flag': LaunchConfiguration('ins_tf2_flag'),
            #     'world_frame_id': LaunchConfiguration('world_frame_id'),
            #     'lidar_main_frame_id': LaunchConfiguration('lidar_main_frame_id'),
            #     'lidar_imu_frame_id': LaunchConfiguration('lidar_imu_frame_id'),
            #     'ins_frame_id': LaunchConfiguration('ins_frame_id'),
            #     'lidar_main_orintation_flag': LaunchConfiguration('lidar_main_orintation_flag'),
            #     'lidar_imu_orintation_flag': LaunchConfiguration('lidar_imu_orintation_flag'),
            #     'ins_orintation_flag': LaunchConfiguration('ins_orintation_flag'),
            #     'base_2_lidar_main_linear_x': LaunchConfiguration('base_2_lidar_main_linear_x'),
            #     'base_2_lidar_main_linear_y': LaunchConfiguration('base_2_lidar_main_linear_y'),
            #     'base_2_lidar_main_linear_z': LaunchConfiguration('base_2_lidar_main_linear_z'),
            #     'base_2_lidar_main_orintation_x': LaunchConfiguration('base_2_lidar_main_orintation_x'),
            #     'base_2_lidar_main_orintation_y': LaunchConfiguration('base_2_lidar_main_orintation_y'),
            #     'base_2_lidar_main_orintation_z': LaunchConfiguration('base_2_lidar_main_orintation_z'),
            #     'base_2_lidar_main_orintation_w': LaunchConfiguration('base_2_lidar_main_orintation_w'),
            #     'base_2_lidar_main_angle_x': LaunchConfiguration('base_2_lidar_main_angle_x'),
            #     'base_2_lidar_main_angle_y': LaunchConfiguration('base_2_lidar_main_angle_y'),
            #     'base_2_lidar_main_angle_z': LaunchConfiguration('base_2_lidar_main_angle_z'),
            #     'base_2_lidar_imu_linear_x': LaunchConfiguration('base_2_lidar_imu_linear_x'),
            #     'base_2_lidar_imu_linear_y': LaunchConfiguration('base_2_lidar_imu_linear_y'),
            #     'base_2_lidar_imu_linear_z': LaunchConfiguration('base_2_lidar_imu_linear_z'),
            #     'base_2_lidar_imu_orintation_x': LaunchConfiguration('base_2_lidar_imu_orintation_x'),
            #     'base_2_lidar_imu_orintation_y': LaunchConfiguration('base_2_lidar_imu_orintation_y'),
            #     'base_2_lidar_imu_orintation_z': LaunchConfiguration('base_2_lidar_imu_orintation_z'),
            #     'base_2_lidar_imu_orintation_w': LaunchConfiguration('base_2_lidar_imu_orintation_w'),
            #     'base_2_lidar_imu_angle_x': LaunchConfiguration('base_2_lidar_imu_angle_x'),
            #     'base_2_lidar_imu_angle_y': LaunchConfiguration('base_2_lidar_imu_angle_y'),
            #     'base_2_lidar_imu_angle_z': LaunchConfiguration('base_2_lidar_imu_angle_z'),
            #     'base_2_ins_linear_x': LaunchConfiguration('base_2_ins_linear_x'),
            #     'base_2_ins_linear_y': LaunchConfiguration('base_2_ins_linear_y'),
            #     'base_2_ins_linear_z': LaunchConfiguration('base_2_ins_linear_z'),
            #     'base_2_ins_orintation_x': LaunchConfiguration('base_2_ins_orintation_x'),
            #     'base_2_ins_orintation_y': LaunchConfiguration('base_2_ins_orintation_y'),
            #     'base_2_ins_orintation_z': LaunchConfiguration('base_2_ins_orintation_z'),
            #     'base_2_ins_orintation_w': LaunchConfiguration('base_2_ins_orintation_w'),
            #     'base_2_ins_angle_x': LaunchConfiguration('base_2_ins_angle_x'),
            #     'base_2_ins_angle_y': LaunchConfiguration('base_2_ins_angle_y'),
            #     'base_2_ins_angle_z': LaunchConfiguration('base_2_ins_angle_z')
            # }]
        ),
        baudrate_UltrasonicRadar_arg,
        UltrasonicRadar1_ComName_arg,
        UltrasonicRadar2_ComName_arg,
        UltrasonicRadar3_ComName_arg,
        UltrasonicRadar4_ComName_arg,
        UltrasonicRadar5_ComName_arg,
        UltrasonicRadar6_ComName_arg,
        Publish_topic_1_arg,
        Publish_topic_2_arg,
        Publish_topic_3_arg,
        Publish_topic_4_arg,
        Publish_topic_5_arg,
        Publish_topic_6_arg,
        Publish_topic_7_arg,
        Publish_topic_8_arg,
        Publish_topic_9_arg,
        Publish_topic_10_arg,
        Publish_topic_11_arg,
        Publish_topic_12_arg,
        Publish_topic_13_arg,
        Publish_topic_14_arg,
        Publish_topic_15_arg,
        Publish_topic_16_arg,
        Publish_topic_17_arg,
        Publish_topic_18_arg,
        Publish_topic_19_arg,
        Publish_topic_20_arg,
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_1_node',
            name='ultrasonic_radar_com1',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar1_ComName': LaunchConfiguration('UltrasonicRadar1_ComName'),
                'Publish_topic_1': LaunchConfiguration('Publish_topic_1'),
                'Publish_topic_2': LaunchConfiguration('Publish_topic_2'),
                'Publish_topic_3': LaunchConfiguration('Publish_topic_3')
            }]
        ),
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_2_node',
            name='ultrasonic_radar_com2',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar2_ComName': LaunchConfiguration('UltrasonicRadar2_ComName'),
                'Publish_topic_4': LaunchConfiguration('Publish_topic_4'),
                'Publish_topic_5': LaunchConfiguration('Publish_topic_5'),
                'Publish_topic_6': LaunchConfiguration('Publish_topic_6'),
            }]
        ),
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_3_node',
            name='ultrasonic_radar_com3',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar3_ComName': LaunchConfiguration('UltrasonicRadar3_ComName'),
                'Publish_topic_7': LaunchConfiguration('Publish_topic_7'),
                'Publish_topic_8': LaunchConfiguration('Publish_topic_8'),
                'Publish_topic_9': LaunchConfiguration('Publish_topic_9'),
                'Publish_topic_10': LaunchConfiguration('Publish_topic_10')
            }]
        ),
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_4_node',
            name='ultrasonic_radar_com4',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar4_ComName': LaunchConfiguration('UltrasonicRadar4_ComName'),
                'Publish_topic_11': LaunchConfiguration('Publish_topic_11'),
                'Publish_topic_12': LaunchConfiguration('Publish_topic_12'),
                'Publish_topic_13': LaunchConfiguration('Publish_topic_13')
            }]
        ),
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_5_node',
            name='ultrasonic_radar_com5',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar5_ComName': LaunchConfiguration('UltrasonicRadar5_ComName'),
                'Publish_topic_14': LaunchConfiguration('Publish_topic_14'),
                'Publish_topic_15': LaunchConfiguration('Publish_topic_15'),
                'Publish_topic_16': LaunchConfiguration('Publish_topic_16'),
            }]
        ),
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_6_node',
            name='ultrasonic_radar_com6',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar6_ComName': LaunchConfiguration('UltrasonicRadar6_ComName'),
                'Publish_topic_17': LaunchConfiguration('Publish_topic_17'),
                'Publish_topic_18': LaunchConfiguration('Publish_topic_18'),
                'Publish_topic_19': LaunchConfiguration('Publish_topic_19'),
                'Publish_topic_20': LaunchConfiguration('Publish_topic_20')
            }]
        ),
        rslidar_sdk_node
        ])
