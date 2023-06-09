from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    imu_can_name_arg = DeclareLaunchArgument(
        'imu_can_name',default_value=TextSubstitution(text='can0')
    )
    gravity_acc_arg = DeclareLaunchArgument(
        'gravity_acc',default_value=TextSubstitution(text='9.8')
    )
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',default_value=TextSubstitution(text='/lidar_front/imu')
    )
    
    return LaunchDescription([
        imu_can_name_arg,
        gravity_acc_arg,
        imu_frame_id_arg,
        Node(
            package='imu_can_pkg',
            executable='imu_can_driver_node',
            name='imu_can',
            parameters=[{
                'imu_can_name': LaunchConfiguration('imu_can_name'),
                'gravity_acc': LaunchConfiguration('gravity_acc'),
                'imu_frame_id': LaunchConfiguration('imu_frame_id')
            }]
        )])
