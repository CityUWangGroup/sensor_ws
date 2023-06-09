from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    baudrate_INS_arg = DeclareLaunchArgument(
        'baudrate_INS',default_value=TextSubstitution(text='115200')
    )
    INS_ComName_arg = DeclareLaunchArgument(
        'INS_ComName',default_value=TextSubstitution(text='/dev/ttyUSB0')
    )
    gnss_frame_id_arg = DeclareLaunchArgument(
        'gnss_frame_id',default_value=TextSubstitution(text='/gnss')
    )
    
    return LaunchDescription([
        baudrate_INS_arg,
        INS_ComName_arg,
        gnss_frame_id_arg,
        Node(
            package='gnss_com_pkg',
            executable='gnss_com_driver_node',
            name='gnss_com',
            parameters=[{
                'baudrate_INS': LaunchConfiguration('baudrate_INS'),
                'INS_ComName': LaunchConfiguration('INS_ComName'),
                'gnss_frame_id': LaunchConfiguration('gnss_frame_id')
            }]
        )])
