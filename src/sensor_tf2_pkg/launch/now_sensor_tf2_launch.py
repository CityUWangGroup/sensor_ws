from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    # sensor tf2 arg
    lidar_main_tf2_flag_arg = DeclareLaunchArgument('lidar_main_tf2_flag',default_value=TextSubstitution(text='true'))
    lidar_imu_tf2_flag_arg = DeclareLaunchArgument('lidar_imu_tf2_flag',default_value=TextSubstitution(text='true'))
    ins_tf2_flag_arg = DeclareLaunchArgument('ins_tf2_flag',default_value=TextSubstitution(text='true'))
    world_frame_id_arg = DeclareLaunchArgument('world_frame_id',default_value=TextSubstitution(text='/base_link'))
    lidar_main_frame_id_arg = DeclareLaunchArgument('lidar_main_frame_id',default_value=TextSubstitution(text='/lidar_front'))
    lidar_imu_frame_id_arg = DeclareLaunchArgument('lidar_imu_frame_id',default_value=TextSubstitution(text='/lidar_front_imu'))
    ins_frame_id_arg = DeclareLaunchArgument('ins_frame_id',default_value=TextSubstitution(text='/ins'))
    lidar_main_orintation_flag_arg = DeclareLaunchArgument('lidar_main_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_imu_orintation_flag_arg = DeclareLaunchArgument('lidar_imu_orintation_flag',default_value=TextSubstitution(text='true'))
    ins_orintation_flag_arg = DeclareLaunchArgument('ins_orintation_flag',default_value=TextSubstitution(text='true'))
    base_2_lidar_main_linear_x_arg = DeclareLaunchArgument('base_2_lidar_main_linear_x',default_value=TextSubstitution(text='1.665'))
    base_2_lidar_main_linear_y_arg = DeclareLaunchArgument('base_2_lidar_main_linear_y',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_linear_z_arg = DeclareLaunchArgument('base_2_lidar_main_linear_z',default_value=TextSubstitution(text='1.742'))
    base_2_lidar_main_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_x',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_y',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_z',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_w',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_angle_x_arg = DeclareLaunchArgument('base_2_lidar_main_angle_x',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_angle_y_arg = DeclareLaunchArgument('base_2_lidar_main_angle_y',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_main_angle_z_arg = DeclareLaunchArgument('base_2_lidar_main_angle_z',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_linear_x_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_x',default_value=TextSubstitution(text='1.665'))
    base_2_lidar_imu_linear_y_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_y',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_linear_z_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_z',default_value=TextSubstitution(text='1.676'))
    base_2_lidar_imu_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_x',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_y',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_z',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_w',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_angle_x_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_x',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_angle_y_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_y',default_value=TextSubstitution(text='0.0'))
    base_2_lidar_imu_angle_z_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_z',default_value=TextSubstitution(text='0.0'))
    base_2_ins_linear_x_arg = DeclareLaunchArgument('base_2_ins_linear_x',default_value=TextSubstitution(text='0.613'))
    base_2_ins_linear_y_arg = DeclareLaunchArgument('base_2_ins_linear_y',default_value=TextSubstitution(text='0.0'))
    base_2_ins_linear_z_arg = DeclareLaunchArgument('base_2_ins_linear_z',default_value=TextSubstitution(text='0.3182'))
    base_2_ins_orintation_x_arg = DeclareLaunchArgument('base_2_ins_orintation_x',default_value=TextSubstitution(text='0.0'))
    base_2_ins_orintation_y_arg = DeclareLaunchArgument('base_2_ins_orintation_y',default_value=TextSubstitution(text='0.0'))
    base_2_ins_orintation_z_arg = DeclareLaunchArgument('base_2_ins_orintation_z',default_value=TextSubstitution(text='0.0'))
    base_2_ins_orintation_w_arg = DeclareLaunchArgument('base_2_ins_orintation_w',default_value=TextSubstitution(text='0.0'))
    base_2_ins_angle_x_arg = DeclareLaunchArgument('base_2_ins_angle_x',default_value=TextSubstitution(text='0.0'))
    base_2_ins_angle_y_arg = DeclareLaunchArgument('base_2_ins_angle_y',default_value=TextSubstitution(text='0.0'))
    base_2_ins_angle_z_arg = DeclareLaunchArgument('base_2_ins_angle_z',default_value=TextSubstitution(text='0.0'))

    
    return LaunchDescription([
        lidar_main_tf2_flag_arg,
        lidar_imu_tf2_flag_arg,
        ins_tf2_flag_arg,
        world_frame_id_arg,
        lidar_main_frame_id_arg,
        lidar_imu_frame_id_arg,
        ins_frame_id_arg,
        lidar_main_orintation_flag_arg,
        lidar_imu_orintation_flag_arg,
        ins_orintation_flag_arg,
        base_2_lidar_main_linear_x_arg,
        base_2_lidar_main_linear_y_arg,
        base_2_lidar_main_linear_z_arg,
        base_2_lidar_main_orintation_x_arg,
        base_2_lidar_main_orintation_y_arg,
        base_2_lidar_main_orintation_z_arg,
        base_2_lidar_main_orintation_w_arg,
        base_2_lidar_main_angle_x_arg,
        base_2_lidar_main_angle_y_arg,
        base_2_lidar_main_angle_z_arg,
        base_2_lidar_imu_linear_x_arg,
        base_2_lidar_imu_linear_y_arg,
        base_2_lidar_imu_linear_z_arg,
        base_2_lidar_imu_orintation_x_arg,
        base_2_lidar_imu_orintation_y_arg,
        base_2_lidar_imu_orintation_z_arg,
        base_2_lidar_imu_orintation_w_arg,
        base_2_lidar_imu_angle_x_arg,
        base_2_lidar_imu_angle_y_arg,
        base_2_lidar_imu_angle_z_arg,
        base_2_ins_linear_x_arg,
        base_2_ins_linear_y_arg,
        base_2_ins_linear_z_arg,
        base_2_ins_orintation_x_arg,
        base_2_ins_orintation_y_arg,
        base_2_ins_orintation_z_arg,
        base_2_ins_orintation_w_arg,
        base_2_ins_angle_x_arg,
        base_2_ins_angle_y_arg,
        base_2_ins_angle_z_arg,
        Node(
            package='sensor_tf2_pkg',
            executable='sensor_tf2_node',
            name='sensor_tf2',
            parameters=[{
                'lidar_main_tf2_flag': LaunchConfiguration('lidar_main_tf2_flag'),
                'lidar_imu_tf2_flag': LaunchConfiguration('lidar_imu_tf2_flag'),
                'ins_tf2_flag': LaunchConfiguration('ins_tf2_flag'),
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'lidar_main_frame_id': LaunchConfiguration('lidar_main_frame_id'),
                'lidar_imu_frame_id': LaunchConfiguration('lidar_imu_frame_id'),
                'ins_frame_id': LaunchConfiguration('ins_frame_id'),
                'lidar_main_orintation_flag': LaunchConfiguration('lidar_main_orintation_flag'),
                'lidar_imu_orintation_flag': LaunchConfiguration('lidar_imu_orintation_flag'),
                'ins_orintation_flag': LaunchConfiguration('ins_orintation_flag'),
                'base_2_lidar_main_linear_x': LaunchConfiguration('base_2_lidar_main_linear_x'),
                'base_2_lidar_main_linear_y': LaunchConfiguration('base_2_lidar_main_linear_y'),
                'base_2_lidar_main_linear_z': LaunchConfiguration('base_2_lidar_main_linear_z'),
                'base_2_lidar_main_orintation_x': LaunchConfiguration('base_2_lidar_main_orintation_x'),
                'base_2_lidar_main_orintation_y': LaunchConfiguration('base_2_lidar_main_orintation_y'),
                'base_2_lidar_main_orintation_z': LaunchConfiguration('base_2_lidar_main_orintation_z'),
                'base_2_lidar_main_orintation_w': LaunchConfiguration('base_2_lidar_main_orintation_w'),
                'base_2_lidar_main_angle_x': LaunchConfiguration('base_2_lidar_main_angle_x'),
                'base_2_lidar_main_angle_y': LaunchConfiguration('base_2_lidar_main_angle_y'),
                'base_2_lidar_main_angle_z': LaunchConfiguration('base_2_lidar_main_angle_z'),
                'base_2_lidar_imu_linear_x': LaunchConfiguration('base_2_lidar_imu_linear_x'),
                'base_2_lidar_imu_linear_y': LaunchConfiguration('base_2_lidar_imu_linear_y'),
                'base_2_lidar_imu_linear_z': LaunchConfiguration('base_2_lidar_imu_linear_z'),
                'base_2_lidar_imu_orintation_x': LaunchConfiguration('base_2_lidar_imu_orintation_x'),
                'base_2_lidar_imu_orintation_y': LaunchConfiguration('base_2_lidar_imu_orintation_y'),
                'base_2_lidar_imu_orintation_z': LaunchConfiguration('base_2_lidar_imu_orintation_z'),
                'base_2_lidar_imu_orintation_w': LaunchConfiguration('base_2_lidar_imu_orintation_w'),
                'base_2_lidar_imu_angle_x': LaunchConfiguration('base_2_lidar_imu_angle_x'),
                'base_2_lidar_imu_angle_y': LaunchConfiguration('base_2_lidar_imu_angle_y'),
                'base_2_lidar_imu_angle_z': LaunchConfiguration('base_2_lidar_imu_angle_z'),
                'base_2_ins_linear_x': LaunchConfiguration('base_2_ins_linear_x'),
                'base_2_ins_linear_y': LaunchConfiguration('base_2_ins_linear_y'),
                'base_2_ins_linear_z': LaunchConfiguration('base_2_ins_linear_z'),
                'base_2_ins_orintation_x': LaunchConfiguration('base_2_ins_orintation_x'),
                'base_2_ins_orintation_y': LaunchConfiguration('base_2_ins_orintation_y'),
                'base_2_ins_orintation_z': LaunchConfiguration('base_2_ins_orintation_z'),
                'base_2_ins_orintation_w': LaunchConfiguration('base_2_ins_orintation_w'),
                'base_2_ins_angle_x': LaunchConfiguration('base_2_ins_angle_x'),
                'base_2_ins_angle_y': LaunchConfiguration('base_2_ins_angle_y'),
                'base_2_ins_angle_z': LaunchConfiguration('base_2_ins_angle_z')
            }]
        ),
    ])
