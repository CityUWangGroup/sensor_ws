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
    
    return LaunchDescription([
        baudrate_UltrasonicRadar_arg,
        UltrasonicRadar1_ComName_arg,
        UltrasonicRadar2_ComName_arg,
        UltrasonicRadar3_ComName_arg,
        UltrasonicRadar4_ComName_arg,
        UltrasonicRadar5_ComName_arg,
        UltrasonicRadar6_ComName_arg,
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_node',
            name='ultrasonic_radar',
            parameters=[{
                'baudrate_UltrasonicRadar': LaunchConfiguration('baudrate_UltrasonicRadar'),
                'UltrasonicRadar1_ComName': LaunchConfiguration('UltrasonicRadar1_ComName'),
                'UltrasonicRadar2_ComName': LaunchConfiguration('UltrasonicRadar2_ComName'),
                'UltrasonicRadar3_ComName': LaunchConfiguration('UltrasonicRadar3_ComName'),
                'UltrasonicRadar4_ComName': LaunchConfiguration('UltrasonicRadar4_ComName'),
                'UltrasonicRadar5_ComName': LaunchConfiguration('UltrasonicRadar5_ComName'),
                'UltrasonicRadar6_ComName': LaunchConfiguration('UltrasonicRadar6_ComName')
            }]
        )])
