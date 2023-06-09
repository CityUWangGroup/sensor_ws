from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    baudrate_IMU_arg = DeclareLaunchArgument(
        'baudrate_IMU',default_value=TextSubstitution(text='230400')
    )
    IMU_ComName_arg = DeclareLaunchArgument(
        'IMU_ComName',default_value=TextSubstitution(text='/dev/lidarIMU')
    )
    gravity_acc_arg = DeclareLaunchArgument(
        'gravity_acc',default_value=TextSubstitution(text='9.8')
    )
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',default_value=TextSubstitution(text='/lidar_front/imu')
    )
    
    return LaunchDescription([
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
        )])
