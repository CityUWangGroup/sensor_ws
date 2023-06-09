from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    baudrate_INS_arg = DeclareLaunchArgument(
        'baudrate_INS',default_value=TextSubstitution(text='921600')
    )
    INS_ComName_arg = DeclareLaunchArgument(
        'INS_ComName',default_value=TextSubstitution(text='/dev/insCOM0')
    )
    ins_frame_id_arg = DeclareLaunchArgument(
        'ins_frame_id',default_value=TextSubstitution(text='/ins')
    )
    
    return LaunchDescription([
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
        )])
