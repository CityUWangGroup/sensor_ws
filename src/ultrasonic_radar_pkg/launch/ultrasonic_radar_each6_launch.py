from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    baudrate_UltrasonicRadar_arg = DeclareLaunchArgument(
        'baudrate_UltrasonicRadar',default_value=TextSubstitution(text='9600')
    )
    UltrasonicRadar1_ComName_arg = DeclareLaunchArgument(
        'UltrasonicRadar1_ComName',default_value=TextSubstitution(text='/dev/ultraCOM0')
    )
    UltrasonicRadar2_ComName_arg = DeclareLaunchArgument(
        'UltrasonicRadar2_ComName',default_value=TextSubstitution(text='/dev/ultraCOM1')
    )
    UltrasonicRadar3_ComName_arg = DeclareLaunchArgument(
        'UltrasonicRadar3_ComName',default_value=TextSubstitution(text='/dev/ultraCOM2')
    )
    UltrasonicRadar4_ComName_arg = DeclareLaunchArgument(
        'UltrasonicRadar4_ComName',default_value=TextSubstitution(text='/dev/ultraCOM3')
    )
    UltrasonicRadar5_ComName_arg = DeclareLaunchArgument(
        'UltrasonicRadar5_ComName',default_value=TextSubstitution(text='/dev/ultraCOM4')
    )
    UltrasonicRadar6_ComName_arg = DeclareLaunchArgument(
        'UltrasonicRadar6_ComName',default_value=TextSubstitution(text='/dev/ultraCOM5')
    )
    Publish_topic_1_arg = DeclareLaunchArgument(
        'Publish_topic_1',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM01')
    )
    Publish_topic_2_arg = DeclareLaunchArgument(
        'Publish_topic_2',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM02')
    )
    Publish_topic_3_arg = DeclareLaunchArgument(
        'Publish_topic_3',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM03')
    )
    Publish_topic_4_arg = DeclareLaunchArgument(
        'Publish_topic_4',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM04')
    )
    Publish_topic_5_arg = DeclareLaunchArgument(
        'Publish_topic_5',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM05')
    )
    Publish_topic_6_arg = DeclareLaunchArgument(
        'Publish_topic_6',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM06')
    )
    Publish_topic_7_arg = DeclareLaunchArgument(
        'Publish_topic_7',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM07')
    )
    Publish_topic_8_arg = DeclareLaunchArgument(
        'Publish_topic_8',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM08')
    )
    Publish_topic_9_arg = DeclareLaunchArgument(
        'Publish_topic_9',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM09')
    )
    Publish_topic_10_arg = DeclareLaunchArgument(
        'Publish_topic_10',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM10')
    )
    Publish_topic_11_arg = DeclareLaunchArgument(
        'Publish_topic_11',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM11')
    )
    Publish_topic_12_arg = DeclareLaunchArgument(
        'Publish_topic_12',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM12')
    )
    Publish_topic_13_arg = DeclareLaunchArgument(
        'Publish_topic_13',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM13')
    )
    Publish_topic_14_arg = DeclareLaunchArgument(
        'Publish_topic_14',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM14')
    )
    Publish_topic_15_arg = DeclareLaunchArgument(
        'Publish_topic_15',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM15')
    )
    Publish_topic_16_arg = DeclareLaunchArgument(
        'Publish_topic_16',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM16')
    )
    Publish_topic_17_arg = DeclareLaunchArgument(
        'Publish_topic_17',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM17')
    )
    Publish_topic_18_arg = DeclareLaunchArgument(
        'Publish_topic_18',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM18')
    )
    Publish_topic_19_arg = DeclareLaunchArgument(
        'Publish_topic_19',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM19')
    )
    Publish_topic_20_arg = DeclareLaunchArgument(
        'Publish_topic_20',default_value=TextSubstitution(text='/Sensor_msgs/UltrasonicRadar/COM20')
    )
    
    return LaunchDescription([
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
        )])
