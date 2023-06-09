from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    chassis_can_name_arg = DeclareLaunchArgument(
        'chassis_can_name',default_value=TextSubstitution(text='can0')
    )
    speed_coefficient_arg = DeclareLaunchArgument(
        'speed_coefficient',default_value=TextSubstitution(text='8.0')
    )
    lr_signal_arg = DeclareLaunchArgument(
        'lr_signal',default_value=TextSubstitution(text='1')
    )
    
    return LaunchDescription([
        chassis_can_name_arg,
        speed_coefficient_arg,
        lr_signal_arg,
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_node',
            name='ultrasonic_radar',
            parameters=[{
                'chassis_can_name': LaunchConfiguration('chassis_can_name'),
                'speed_coefficient': LaunchConfiguration('speed_coefficient'),
                'lr_signal': LaunchConfiguration('lr_signal')
            }]
        )])
