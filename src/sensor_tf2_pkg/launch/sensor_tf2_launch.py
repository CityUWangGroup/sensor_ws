from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    lidar_main_tf2_flag_arg = DeclareLaunchArgument('lidar_main_tf2_flag',default_value=TextSubstitution(text='true'))
    lidar_imu_tf2_flag_arg = DeclareLaunchArgument('lidar_imu_tf2_flag',default_value=TextSubstitution(text='true'))
    lidar_1_tf2_flag_arg = DeclareLaunchArgument('lidar_1_tf2_flag',default_value=TextSubstitution(text='false'))
    lidar_2_tf2_flag_arg = DeclareLaunchArgument('lidar_2_tf2_flag',default_value=TextSubstitution(text='false'))
    lidar_3_tf2_flag_arg = DeclareLaunchArgument('lidar_3_tf2_flag',default_value=TextSubstitution(text='false'))
    lidar_4_tf2_flag_arg = DeclareLaunchArgument('lidar_4_tf2_flag',default_value=TextSubstitution(text='false'))
    lidar_5_tf2_flag_arg = DeclareLaunchArgument('lidar_5_tf2_flag',default_value=TextSubstitution(text='false'))
    lidar_6_tf2_flag_arg = DeclareLaunchArgument('lidar_6_tf2_flag',default_value=TextSubstitution(text='false'))
    ins_tf2_flag_arg = DeclareLaunchArgument('ins_tf2_flag',default_value=TextSubstitution(text='true'))
    radar_tf2_flag_arg = DeclareLaunchArgument('radar_tf2_flag',default_value=TextSubstitution(text='false'))

    world_frame_id_arg = DeclareLaunchArgument('world_frame_id',default_value=TextSubstitution(text='/base_link'))
    lidar_main_frame_id_arg = DeclareLaunchArgument('lidar_main_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_imu_frame_id_arg = DeclareLaunchArgument('lidar_imu_frame_id',default_value=TextSubstitution(text='/lidar_front/imu'))
    lidar_1_frame_id_arg = DeclareLaunchArgument('lidar_1_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_2_frame_id_arg = DeclareLaunchArgument('lidar_2_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_3_frame_id_arg = DeclareLaunchArgument('lidar_3_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_4_frame_id_arg = DeclareLaunchArgument('lidar_4_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_5_frame_id_arg = DeclareLaunchArgument('lidar_5_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    lidar_6_frame_id_arg = DeclareLaunchArgument('lidar_6_frame_id',default_value=TextSubstitution(text='/lidar_front/points_xyzi'))
    ins_frame_id_arg = DeclareLaunchArgument('ins_frame_id',default_value=TextSubstitution(text='/ins'))
    radar_frame_id_arg = DeclareLaunchArgument('radar_frame_id',default_value=TextSubstitution(text='/radar'))
    
    lidar_main_orintation_flag_arg = DeclareLaunchArgument('lidar_main_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_imu_orintation_flag_arg = DeclareLaunchArgument('lidar_imu_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_1_orintation_flag_arg = DeclareLaunchArgument('lidar_1_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_2_orintation_flag_arg = DeclareLaunchArgument('lidar_2_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_3_orintation_flag_arg = DeclareLaunchArgument('lidar_3_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_4_orintation_flag_arg = DeclareLaunchArgument('lidar_4_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_5_orintation_flag_arg = DeclareLaunchArgument('lidar_5_orintation_flag',default_value=TextSubstitution(text='true'))
    lidar_6_orintation_flag_arg = DeclareLaunchArgument('lidar_6_orintation_flag',default_value=TextSubstitution(text='true'))
    ins_orintation_flag_arg = DeclareLaunchArgument('ins_orintation_flag',default_value=TextSubstitution(text='true'))
    radar_orintation_flag_arg = DeclareLaunchArgument('radar_orintation_flag',default_value=TextSubstitution(text='true'))

    #                          linear_x y z       orintation_x y z w      angle_x y z
    base_2_lidar_main_matrix = ['1.665','0.0','1.742','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_imu_matrix = ['1.665','0.0','1.676','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_1_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_2_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_3_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_4_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_5_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_6_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_ins_matrix = ['0.613','0.0','0.3182','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_radar_matrix = ['0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0','0.0']
    base_2_lidar_main_linear_x_arg = DeclareLaunchArgument('base_2_lidar_main_linear_x',default_value=TextSubstitution(text=base_2_lidar_main_matrix[0]))
    base_2_lidar_main_linear_y_arg = DeclareLaunchArgument('base_2_lidar_main_linear_y',default_value=TextSubstitution(text=base_2_lidar_main_matrix[1]))
    base_2_lidar_main_linear_z_arg = DeclareLaunchArgument('base_2_lidar_main_linear_z',default_value=TextSubstitution(text=base_2_lidar_main_matrix[2]))
    base_2_lidar_main_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_x',default_value=TextSubstitution(text=base_2_lidar_main_matrix[3]))
    base_2_lidar_main_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_y',default_value=TextSubstitution(text=base_2_lidar_main_matrix[4]))
    base_2_lidar_main_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_z',default_value=TextSubstitution(text=base_2_lidar_main_matrix[5]))
    base_2_lidar_main_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_main_orintation_w',default_value=TextSubstitution(text=base_2_lidar_main_matrix[6]))
    base_2_lidar_main_angle_x_arg = DeclareLaunchArgument('base_2_lidar_main_angle_x',default_value=TextSubstitution(text=base_2_lidar_main_matrix[7]))
    base_2_lidar_main_angle_y_arg = DeclareLaunchArgument('base_2_lidar_main_angle_y',default_value=TextSubstitution(text=base_2_lidar_main_matrix[8]))
    base_2_lidar_main_angle_z_arg = DeclareLaunchArgument('base_2_lidar_main_angle_z',default_value=TextSubstitution(text=base_2_lidar_main_matrix[9]))
    base_2_lidar_imu_linear_x_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_x',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[0]))
    base_2_lidar_imu_linear_y_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_y',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[1]))
    base_2_lidar_imu_linear_z_arg = DeclareLaunchArgument('base_2_lidar_imu_linear_z',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[2]))
    base_2_lidar_imu_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_x',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[3]))
    base_2_lidar_imu_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_y',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[4]))
    base_2_lidar_imu_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_z',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[5]))
    base_2_lidar_imu_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_imu_orintation_w',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[6]))
    base_2_lidar_imu_angle_x_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_x',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[7]))
    base_2_lidar_imu_angle_y_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_y',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[8]))
    base_2_lidar_imu_angle_z_arg = DeclareLaunchArgument('base_2_lidar_imu_angle_z',default_value=TextSubstitution(text=base_2_lidar_imu_matrix[9]))
    base_2_lidar_1_linear_x_arg = DeclareLaunchArgument('base_2_lidar_1_linear_x',default_value=TextSubstitution(text=base_2_lidar_1_matrix[0]))
    base_2_lidar_1_linear_y_arg = DeclareLaunchArgument('base_2_lidar_1_linear_y',default_value=TextSubstitution(text=base_2_lidar_1_matrix[1]))
    base_2_lidar_1_linear_z_arg = DeclareLaunchArgument('base_2_lidar_1_linear_z',default_value=TextSubstitution(text=base_2_lidar_1_matrix[2]))
    base_2_lidar_1_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_1_orintation_x',default_value=TextSubstitution(text=base_2_lidar_1_matrix[3]))
    base_2_lidar_1_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_1_orintation_y',default_value=TextSubstitution(text=base_2_lidar_1_matrix[4]))
    base_2_lidar_1_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_1_orintation_z',default_value=TextSubstitution(text=base_2_lidar_1_matrix[5]))
    base_2_lidar_1_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_1_orintation_w',default_value=TextSubstitution(text=base_2_lidar_1_matrix[6]))
    base_2_lidar_1_angle_x_arg = DeclareLaunchArgument('base_2_lidar_1_angle_x',default_value=TextSubstitution(text=base_2_lidar_1_matrix[7]))
    base_2_lidar_1_angle_y_arg = DeclareLaunchArgument('base_2_lidar_1_angle_y',default_value=TextSubstitution(text=base_2_lidar_1_matrix[8]))
    base_2_lidar_1_angle_z_arg = DeclareLaunchArgument('base_2_lidar_1_angle_z',default_value=TextSubstitution(text=base_2_lidar_1_matrix[9]))
    base_2_lidar_2_linear_x_arg = DeclareLaunchArgument('base_2_lidar_2_linear_x',default_value=TextSubstitution(text=base_2_lidar_2_matrix[0]))
    base_2_lidar_2_linear_y_arg = DeclareLaunchArgument('base_2_lidar_2_linear_y',default_value=TextSubstitution(text=base_2_lidar_2_matrix[1]))
    base_2_lidar_2_linear_z_arg = DeclareLaunchArgument('base_2_lidar_2_linear_z',default_value=TextSubstitution(text=base_2_lidar_2_matrix[2]))
    base_2_lidar_2_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_2_orintation_x',default_value=TextSubstitution(text=base_2_lidar_2_matrix[3]))
    base_2_lidar_2_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_2_orintation_y',default_value=TextSubstitution(text=base_2_lidar_2_matrix[4]))
    base_2_lidar_2_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_2_orintation_z',default_value=TextSubstitution(text=base_2_lidar_2_matrix[5]))
    base_2_lidar_2_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_2_orintation_w',default_value=TextSubstitution(text=base_2_lidar_2_matrix[6]))
    base_2_lidar_2_angle_x_arg = DeclareLaunchArgument('base_2_lidar_2_angle_x',default_value=TextSubstitution(text=base_2_lidar_2_matrix[7]))
    base_2_lidar_2_angle_y_arg = DeclareLaunchArgument('base_2_lidar_2_angle_y',default_value=TextSubstitution(text=base_2_lidar_2_matrix[8]))
    base_2_lidar_2_angle_z_arg = DeclareLaunchArgument('base_2_lidar_2_angle_z',default_value=TextSubstitution(text=base_2_lidar_2_matrix[9]))
    base_2_lidar_3_linear_x_arg = DeclareLaunchArgument('base_2_lidar_3_linear_x',default_value=TextSubstitution(text=base_2_lidar_3_matrix[0]))
    base_2_lidar_3_linear_y_arg = DeclareLaunchArgument('base_2_lidar_3_linear_y',default_value=TextSubstitution(text=base_2_lidar_3_matrix[1]))
    base_2_lidar_3_linear_z_arg = DeclareLaunchArgument('base_2_lidar_3_linear_z',default_value=TextSubstitution(text=base_2_lidar_3_matrix[2]))
    base_2_lidar_3_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_3_orintation_x',default_value=TextSubstitution(text=base_2_lidar_3_matrix[3]))
    base_2_lidar_3_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_3_orintation_y',default_value=TextSubstitution(text=base_2_lidar_3_matrix[4]))
    base_2_lidar_3_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_3_orintation_z',default_value=TextSubstitution(text=base_2_lidar_3_matrix[5]))
    base_2_lidar_3_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_3_orintation_w',default_value=TextSubstitution(text=base_2_lidar_3_matrix[6]))
    base_2_lidar_3_angle_x_arg = DeclareLaunchArgument('base_2_lidar_3_angle_x',default_value=TextSubstitution(text=base_2_lidar_3_matrix[7]))
    base_2_lidar_3_angle_y_arg = DeclareLaunchArgument('base_2_lidar_3_angle_y',default_value=TextSubstitution(text=base_2_lidar_3_matrix[8]))
    base_2_lidar_3_angle_z_arg = DeclareLaunchArgument('base_2_lidar_3_angle_z',default_value=TextSubstitution(text=base_2_lidar_3_matrix[9]))
    base_2_lidar_4_linear_x_arg = DeclareLaunchArgument('base_2_lidar_4_linear_x',default_value=TextSubstitution(text=base_2_lidar_4_matrix[0]))
    base_2_lidar_4_linear_y_arg = DeclareLaunchArgument('base_2_lidar_4_linear_y',default_value=TextSubstitution(text=base_2_lidar_4_matrix[1]))
    base_2_lidar_4_linear_z_arg = DeclareLaunchArgument('base_2_lidar_4_linear_z',default_value=TextSubstitution(text=base_2_lidar_4_matrix[2]))
    base_2_lidar_4_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_4_orintation_x',default_value=TextSubstitution(text=base_2_lidar_4_matrix[3]))
    base_2_lidar_4_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_4_orintation_y',default_value=TextSubstitution(text=base_2_lidar_4_matrix[4]))
    base_2_lidar_4_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_4_orintation_z',default_value=TextSubstitution(text=base_2_lidar_4_matrix[5]))
    base_2_lidar_4_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_4_orintation_w',default_value=TextSubstitution(text=base_2_lidar_4_matrix[6]))
    base_2_lidar_4_angle_x_arg = DeclareLaunchArgument('base_2_lidar_4_angle_x',default_value=TextSubstitution(text=base_2_lidar_4_matrix[7]))
    base_2_lidar_4_angle_y_arg = DeclareLaunchArgument('base_2_lidar_4_angle_y',default_value=TextSubstitution(text=base_2_lidar_4_matrix[8]))
    base_2_lidar_4_angle_z_arg = DeclareLaunchArgument('base_2_lidar_4_angle_z',default_value=TextSubstitution(text=base_2_lidar_4_matrix[9]))
    base_2_lidar_5_linear_x_arg = DeclareLaunchArgument('base_2_lidar_5_linear_x',default_value=TextSubstitution(text=base_2_lidar_5_matrix[0]))
    base_2_lidar_5_linear_y_arg = DeclareLaunchArgument('base_2_lidar_5_linear_y',default_value=TextSubstitution(text=base_2_lidar_5_matrix[1]))
    base_2_lidar_5_linear_z_arg = DeclareLaunchArgument('base_2_lidar_5_linear_z',default_value=TextSubstitution(text=base_2_lidar_5_matrix[2]))
    base_2_lidar_5_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_5_orintation_x',default_value=TextSubstitution(text=base_2_lidar_5_matrix[3]))
    base_2_lidar_5_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_5_orintation_y',default_value=TextSubstitution(text=base_2_lidar_5_matrix[4]))
    base_2_lidar_5_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_5_orintation_z',default_value=TextSubstitution(text=base_2_lidar_5_matrix[5]))
    base_2_lidar_5_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_5_orintation_w',default_value=TextSubstitution(text=base_2_lidar_5_matrix[6]))
    base_2_lidar_5_angle_x_arg = DeclareLaunchArgument('base_2_lidar_5_angle_x',default_value=TextSubstitution(text=base_2_lidar_5_matrix[7]))
    base_2_lidar_5_angle_y_arg = DeclareLaunchArgument('base_2_lidar_5_angle_y',default_value=TextSubstitution(text=base_2_lidar_5_matrix[8]))
    base_2_lidar_5_angle_z_arg = DeclareLaunchArgument('base_2_lidar_5_angle_z',default_value=TextSubstitution(text=base_2_lidar_5_matrix[9]))
    base_2_lidar_6_linear_x_arg = DeclareLaunchArgument('base_2_lidar_6_linear_x',default_value=TextSubstitution(text=base_2_lidar_6_matrix[0]))
    base_2_lidar_6_linear_y_arg = DeclareLaunchArgument('base_2_lidar_6_linear_y',default_value=TextSubstitution(text=base_2_lidar_6_matrix[1]))
    base_2_lidar_6_linear_z_arg = DeclareLaunchArgument('base_2_lidar_6_linear_z',default_value=TextSubstitution(text=base_2_lidar_6_matrix[2]))
    base_2_lidar_6_orintation_x_arg = DeclareLaunchArgument('base_2_lidar_6_orintation_x',default_value=TextSubstitution(text=base_2_lidar_6_matrix[3]))
    base_2_lidar_6_orintation_y_arg = DeclareLaunchArgument('base_2_lidar_6_orintation_y',default_value=TextSubstitution(text=base_2_lidar_6_matrix[4]))
    base_2_lidar_6_orintation_z_arg = DeclareLaunchArgument('base_2_lidar_6_orintation_z',default_value=TextSubstitution(text=base_2_lidar_6_matrix[5]))
    base_2_lidar_6_orintation_w_arg = DeclareLaunchArgument('base_2_lidar_6_orintation_w',default_value=TextSubstitution(text=base_2_lidar_6_matrix[6]))
    base_2_lidar_6_angle_x_arg = DeclareLaunchArgument('base_2_lidar_6_angle_x',default_value=TextSubstitution(text=base_2_lidar_6_matrix[7]))
    base_2_lidar_6_angle_y_arg = DeclareLaunchArgument('base_2_lidar_6_angle_y',default_value=TextSubstitution(text=base_2_lidar_6_matrix[8]))
    base_2_lidar_6_angle_z_arg = DeclareLaunchArgument('base_2_lidar_6_angle_z',default_value=TextSubstitution(text=base_2_lidar_6_matrix[9]))
    base_2_ins_linear_x_arg = DeclareLaunchArgument('base_2_ins_linear_x',default_value=TextSubstitution(text=base_2_ins_matrix[0]))
    base_2_ins_linear_y_arg = DeclareLaunchArgument('base_2_ins_linear_y',default_value=TextSubstitution(text=base_2_ins_matrix[1]))
    base_2_ins_linear_z_arg = DeclareLaunchArgument('base_2_ins_linear_z',default_value=TextSubstitution(text=base_2_ins_matrix[2]))
    base_2_ins_orintation_x_arg = DeclareLaunchArgument('base_2_ins_orintation_x',default_value=TextSubstitution(text=base_2_ins_matrix[3]))
    base_2_ins_orintation_y_arg = DeclareLaunchArgument('base_2_ins_orintation_y',default_value=TextSubstitution(text=base_2_ins_matrix[4]))
    base_2_ins_orintation_z_arg = DeclareLaunchArgument('base_2_ins_orintation_z',default_value=TextSubstitution(text=base_2_ins_matrix[5]))
    base_2_ins_orintation_w_arg = DeclareLaunchArgument('base_2_ins_orintation_w',default_value=TextSubstitution(text=base_2_ins_matrix[6]))
    base_2_ins_angle_x_arg = DeclareLaunchArgument('base_2_ins_angle_x',default_value=TextSubstitution(text=base_2_ins_matrix[7]))
    base_2_ins_angle_y_arg = DeclareLaunchArgument('base_2_ins_angle_y',default_value=TextSubstitution(text=base_2_ins_matrix[8]))
    base_2_ins_angle_z_arg = DeclareLaunchArgument('base_2_ins_angle_z',default_value=TextSubstitution(text=base_2_ins_matrix[9]))
    base_2_radar_linear_x_arg = DeclareLaunchArgument('base_2_radar_linear_x',default_value=TextSubstitution(text=base_2_radar_matrix[0]))
    base_2_radar_linear_y_arg = DeclareLaunchArgument('base_2_radar_linear_y',default_value=TextSubstitution(text=base_2_radar_matrix[1]))
    base_2_radar_linear_z_arg = DeclareLaunchArgument('base_2_radar_linear_z',default_value=TextSubstitution(text=base_2_radar_matrix[2]))
    base_2_radar_orintation_x_arg = DeclareLaunchArgument('base_2_radar_orintation_x',default_value=TextSubstitution(text=base_2_radar_matrix[3]))
    base_2_radar_orintation_y_arg = DeclareLaunchArgument('base_2_radar_orintation_y',default_value=TextSubstitution(text=base_2_radar_matrix[4]))
    base_2_radar_orintation_z_arg = DeclareLaunchArgument('base_2_radar_orintation_z',default_value=TextSubstitution(text=base_2_radar_matrix[5]))
    base_2_radar_orintation_w_arg = DeclareLaunchArgument('base_2_radar_orintation_w',default_value=TextSubstitution(text=base_2_radar_matrix[6]))
    base_2_radar_angle_x_arg = DeclareLaunchArgument('base_2_radar_angle_x',default_value=TextSubstitution(text=base_2_radar_matrix[7]))
    base_2_radar_angle_y_arg = DeclareLaunchArgument('base_2_radar_angle_y',default_value=TextSubstitution(text=base_2_radar_matrix[8]))
    base_2_radar_angle_z_arg = DeclareLaunchArgument('base_2_radar_angle_z',default_value=TextSubstitution(text=base_2_radar_matrix[9]))
    
    return LaunchDescription([
        lidar_main_tf2_flag_arg,
        lidar_imu_tf2_flag_arg,
        lidar_1_tf2_flag_arg,
        lidar_2_tf2_flag_arg,
        lidar_3_tf2_flag_arg,
        lidar_4_tf2_flag_arg,
        lidar_5_tf2_flag_arg,
        lidar_6_tf2_flag_arg,
        ins_tf2_flag_arg,
        radar_tf2_flag_arg,
        world_frame_id_arg,
        lidar_main_frame_id_arg,
        lidar_imu_frame_id_arg,
        lidar_1_frame_id_arg,
        lidar_2_frame_id_arg,
        lidar_3_frame_id_arg,
        lidar_4_frame_id_arg,
        lidar_5_frame_id_arg,
        lidar_6_frame_id_arg,
        ins_frame_id_arg,
        radar_frame_id_arg,
        lidar_main_orintation_flag_arg,
        lidar_imu_orintation_flag_arg,
        lidar_1_orintation_flag_arg,
        lidar_2_orintation_flag_arg,
        lidar_3_orintation_flag_arg,
        lidar_4_orintation_flag_arg,
        lidar_5_orintation_flag_arg,
        lidar_6_orintation_flag_arg,
        ins_orintation_flag_arg,
        radar_orintation_flag_arg,
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
        base_2_lidar_1_linear_x_arg,
        base_2_lidar_1_linear_y_arg,
        base_2_lidar_1_linear_z_arg,
        base_2_lidar_1_orintation_x_arg,
        base_2_lidar_1_orintation_y_arg,
        base_2_lidar_1_orintation_z_arg,
        base_2_lidar_1_orintation_w_arg,
        base_2_lidar_1_angle_x_arg,
        base_2_lidar_1_angle_y_arg,
        base_2_lidar_1_angle_z_arg,
        base_2_lidar_2_linear_x_arg,
        base_2_lidar_2_linear_y_arg,
        base_2_lidar_2_linear_z_arg,
        base_2_lidar_2_orintation_x_arg,
        base_2_lidar_2_orintation_y_arg,
        base_2_lidar_2_orintation_z_arg,
        base_2_lidar_2_orintation_w_arg,
        base_2_lidar_2_angle_x_arg,
        base_2_lidar_2_angle_y_arg,
        base_2_lidar_2_angle_z_arg,
        base_2_lidar_3_linear_x_arg,
        base_2_lidar_3_linear_y_arg,
        base_2_lidar_3_linear_z_arg,
        base_2_lidar_3_orintation_x_arg,
        base_2_lidar_3_orintation_y_arg,
        base_2_lidar_3_orintation_z_arg,
        base_2_lidar_3_orintation_w_arg,
        base_2_lidar_3_angle_x_arg,
        base_2_lidar_3_angle_y_arg,
        base_2_lidar_3_angle_z_arg,
        base_2_lidar_4_linear_x_arg,
        base_2_lidar_4_linear_y_arg,
        base_2_lidar_4_linear_z_arg,
        base_2_lidar_4_orintation_x_arg,
        base_2_lidar_4_orintation_y_arg,
        base_2_lidar_4_orintation_z_arg,
        base_2_lidar_4_orintation_w_arg,
        base_2_lidar_4_angle_x_arg,
        base_2_lidar_4_angle_y_arg,
        base_2_lidar_4_angle_z_arg,
        base_2_lidar_5_linear_x_arg,
        base_2_lidar_5_linear_y_arg,
        base_2_lidar_5_linear_z_arg,
        base_2_lidar_5_orintation_x_arg,
        base_2_lidar_5_orintation_y_arg,
        base_2_lidar_5_orintation_z_arg,
        base_2_lidar_5_orintation_w_arg,
        base_2_lidar_5_angle_x_arg,
        base_2_lidar_5_angle_y_arg,
        base_2_lidar_5_angle_z_arg,
        base_2_lidar_6_linear_x_arg,
        base_2_lidar_6_linear_y_arg,
        base_2_lidar_6_linear_z_arg,
        base_2_lidar_6_orintation_x_arg,
        base_2_lidar_6_orintation_y_arg,
        base_2_lidar_6_orintation_z_arg,
        base_2_lidar_6_orintation_w_arg,
        base_2_lidar_6_angle_x_arg,
        base_2_lidar_6_angle_y_arg,
        base_2_lidar_6_angle_z_arg,
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
        base_2_radar_linear_x_arg,
        base_2_radar_linear_y_arg,
        base_2_radar_linear_z_arg,
        base_2_radar_orintation_x_arg,
        base_2_radar_orintation_y_arg,
        base_2_radar_orintation_z_arg,
        base_2_radar_orintation_w_arg,
        base_2_radar_angle_x_arg,
        base_2_radar_angle_y_arg,
        base_2_radar_angle_z_arg,
        Node(
            package='ultrasonic_radar_pkg',
            executable='ultrasonic_radar_node',
            name='ultrasonic_radar',
            parameters=[{
                'lidar_main_tf2_flag': LaunchConfiguration('lidar_main_tf2_flag'),
                'lidar_imu_tf2_flag': LaunchConfiguration('lidar_imu_tf2_flag'),
                'lidar_1_tf2_flag': LaunchConfiguration('lidar_1_tf2_flag'),
                'lidar_2_tf2_flag': LaunchConfiguration('lidar_2_tf2_flag'),
                'lidar_3_tf2_flag': LaunchConfiguration('lidar_3_tf2_flag'),
                'lidar_4_tf2_flag': LaunchConfiguration('lidar_4_tf2_flag'),
                'lidar_5_tf2_flag': LaunchConfiguration('lidar_5_tf2_flag'),
                'lidar_6_tf2_flag': LaunchConfiguration('lidar_6_tf2_flag'),
                'ins_tf2_flag': LaunchConfiguration('ins_tf2_flag'),
                'radar_tf2_flag': LaunchConfiguration('radar_tf2_flag'),
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'lidar_main_frame_id': LaunchConfiguration('lidar_main_frame_id'),
                'lidar_imu_frame_id': LaunchConfiguration('lidar_imu_frame_id'),
                'lidar_1_frame_id': LaunchConfiguration('lidar_1_frame_id'),
                'lidar_2_frame_id': LaunchConfiguration('lidar_2_frame_id'),
                'lidar_3_frame_id': LaunchConfiguration('lidar_3_frame_id'),
                'lidar_4_frame_id': LaunchConfiguration('lidar_4_frame_id'),
                'lidar_5_frame_id': LaunchConfiguration('lidar_5_frame_id'),
                'lidar_6_frame_id': LaunchConfiguration('lidar_6_frame_id'),
                'ins_frame_id': LaunchConfiguration('ins_frame_id'),
                'radar_frame_id': LaunchConfiguration('radar_frame_id'),
                'lidar_main_orintation_flag': LaunchConfiguration('lidar_main_orintation_flag'),
                'lidar_imu_orintation_flag': LaunchConfiguration('lidar_imu_orintation_flag'),
                'lidar_1_orintation_flag': LaunchConfiguration('lidar_1_orintation_flag'),
                'lidar_2_orintation_flag': LaunchConfiguration('lidar_2_orintation_flag'),
                'lidar_3_orintation_flag': LaunchConfiguration('lidar_3_orintation_flag'),
                'lidar_4_orintation_flag': LaunchConfiguration('lidar_4_orintation_flag'),
                'lidar_5_orintation_flag': LaunchConfiguration('lidar_5_orintation_flag'),
                'lidar_6_orintation_flag': LaunchConfiguration('lidar_6_orintation_flag'),
                'ins_orintation_flag': LaunchConfiguration('ins_orintation_flag'),
                'radar_orintation_flag': LaunchConfiguration('radar_orintation_flag'),
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
                'base_2_lidar_1_linear_x': LaunchConfiguration('base_2_lidar_1_linear_x'),
                'base_2_lidar_1_linear_y': LaunchConfiguration('base_2_lidar_1_linear_y'),
                'base_2_lidar_1_linear_z': LaunchConfiguration('base_2_lidar_1_linear_z'),
                'base_2_lidar_1_orintation_x': LaunchConfiguration('base_2_lidar_1_orintation_x'),
                'base_2_lidar_1_orintation_y': LaunchConfiguration('base_2_lidar_1_orintation_y'),
                'base_2_lidar_1_orintation_z': LaunchConfiguration('base_2_lidar_1_orintation_z'),
                'base_2_lidar_1_orintation_w': LaunchConfiguration('base_2_lidar_1_orintation_w'),
                'base_2_lidar_1_angle_x': LaunchConfiguration('base_2_lidar_1_angle_x'),
                'base_2_lidar_1_angle_y': LaunchConfiguration('base_2_lidar_1_angle_y'),
                'base_2_lidar_1_angle_z': LaunchConfiguration('base_2_lidar_1_angle_z'),
                'base_2_lidar_2_linear_x': LaunchConfiguration('base_2_lidar_2_linear_x'),
                'base_2_lidar_2_linear_y': LaunchConfiguration('base_2_lidar_2_linear_y'),
                'base_2_lidar_2_linear_z': LaunchConfiguration('base_2_lidar_2_linear_z'),
                'base_2_lidar_2_orintation_x': LaunchConfiguration('base_2_lidar_2_orintation_x'),
                'base_2_lidar_2_orintation_y': LaunchConfiguration('base_2_lidar_2_orintation_y'),
                'base_2_lidar_2_orintation_z': LaunchConfiguration('base_2_lidar_2_orintation_z'),
                'base_2_lidar_2_orintation_w': LaunchConfiguration('base_2_lidar_2_orintation_w'),
                'base_2_lidar_2_angle_x': LaunchConfiguration('base_2_lidar_2_angle_x'),
                'base_2_lidar_2_angle_y': LaunchConfiguration('base_2_lidar_2_angle_y'),
                'base_2_lidar_2_angle_z': LaunchConfiguration('base_2_lidar_2_angle_z'),
                'base_2_lidar_3_linear_x': LaunchConfiguration('base_2_lidar_3_linear_x'),
                'base_2_lidar_3_linear_y': LaunchConfiguration('base_2_lidar_3_linear_y'),
                'base_2_lidar_3_linear_z': LaunchConfiguration('base_2_lidar_3_linear_z'),
                'base_2_lidar_3_orintation_x': LaunchConfiguration('base_2_lidar_3_orintation_x'),
                'base_2_lidar_3_orintation_y': LaunchConfiguration('base_2_lidar_3_orintation_y'),
                'base_2_lidar_3_orintation_z': LaunchConfiguration('base_2_lidar_3_orintation_z'),
                'base_2_lidar_3_orintation_w': LaunchConfiguration('base_2_lidar_3_orintation_w'),
                'base_2_lidar_3_angle_x': LaunchConfiguration('base_2_lidar_3_angle_x'),
                'base_2_lidar_3_angle_y': LaunchConfiguration('base_2_lidar_3_angle_y'),
                'base_2_lidar_3_angle_z': LaunchConfiguration('base_2_lidar_3_angle_z'),
                'base_2_lidar_4_linear_x': LaunchConfiguration('base_2_lidar_4_linear_x'),
                'base_2_lidar_4_linear_y': LaunchConfiguration('base_2_lidar_4_linear_y'),
                'base_2_lidar_4_linear_z': LaunchConfiguration('base_2_lidar_4_linear_z'),
                'base_2_lidar_4_orintation_x': LaunchConfiguration('base_2_lidar_4_orintation_x'),
                'base_2_lidar_4_orintation_y': LaunchConfiguration('base_2_lidar_4_orintation_y'),
                'base_2_lidar_4_orintation_z': LaunchConfiguration('base_2_lidar_4_orintation_z'),
                'base_2_lidar_4_orintation_w': LaunchConfiguration('base_2_lidar_4_orintation_w'),
                'base_2_lidar_4_angle_x': LaunchConfiguration('base_2_lidar_4_angle_x'),
                'base_2_lidar_4_angle_y': LaunchConfiguration('base_2_lidar_4_angle_y'),
                'base_2_lidar_4_angle_z': LaunchConfiguration('base_2_lidar_4_angle_z'),
                'base_2_lidar_5_linear_x': LaunchConfiguration('base_2_lidar_5_linear_x'),
                'base_2_lidar_5_linear_y': LaunchConfiguration('base_2_lidar_5_linear_y'),
                'base_2_lidar_5_linear_z': LaunchConfiguration('base_2_lidar_5_linear_z'),
                'base_2_lidar_5_orintation_x': LaunchConfiguration('base_2_lidar_5_orintation_x'),
                'base_2_lidar_5_orintation_y': LaunchConfiguration('base_2_lidar_5_orintation_y'),
                'base_2_lidar_5_orintation_z': LaunchConfiguration('base_2_lidar_5_orintation_z'),
                'base_2_lidar_5_orintation_w': LaunchConfiguration('base_2_lidar_5_orintation_w'),
                'base_2_lidar_5_angle_x': LaunchConfiguration('base_2_lidar_5_angle_x'),
                'base_2_lidar_5_angle_y': LaunchConfiguration('base_2_lidar_5_angle_y'),
                'base_2_lidar_5_angle_z': LaunchConfiguration('base_2_lidar_5_angle_z'),
                'base_2_lidar_6_linear_x': LaunchConfiguration('base_2_lidar_6_linear_x'),
                'base_2_lidar_6_linear_y': LaunchConfiguration('base_2_lidar_6_linear_y'),
                'base_2_lidar_6_linear_z': LaunchConfiguration('base_2_lidar_6_linear_z'),
                'base_2_lidar_6_orintation_x': LaunchConfiguration('base_2_lidar_6_orintation_x'),
                'base_2_lidar_6_orintation_y': LaunchConfiguration('base_2_lidar_6_orintation_y'),
                'base_2_lidar_6_orintation_z': LaunchConfiguration('base_2_lidar_6_orintation_z'),
                'base_2_lidar_6_orintation_w': LaunchConfiguration('base_2_lidar_6_orintation_w'),
                'base_2_lidar_6_angle_x': LaunchConfiguration('base_2_lidar_6_angle_x'),
                'base_2_lidar_6_angle_y': LaunchConfiguration('base_2_lidar_6_angle_y'),
                'base_2_lidar_6_angle_z': LaunchConfiguration('base_2_lidar_6_angle_z'),
                'base_2_ins_linear_x': LaunchConfiguration('base_2_ins_linear_x'),
                'base_2_ins_linear_y': LaunchConfiguration('base_2_ins_linear_y'),
                'base_2_ins_linear_z': LaunchConfiguration('base_2_ins_linear_z'),
                'base_2_ins_orintation_x': LaunchConfiguration('base_2_ins_orintation_x'),
                'base_2_ins_orintation_y': LaunchConfiguration('base_2_ins_orintation_y'),
                'base_2_ins_orintation_z': LaunchConfiguration('base_2_ins_orintation_z'),
                'base_2_ins_orintation_w': LaunchConfiguration('base_2_ins_orintation_w'),
                'base_2_ins_angle_x': LaunchConfiguration('base_2_ins_angle_x'),
                'base_2_ins_angle_y': LaunchConfiguration('base_2_ins_angle_y'),
                'base_2_ins_angle_z': LaunchConfiguration('base_2_ins_angle_z'),
                'base_2_radar_linear_x': LaunchConfiguration('base_2_radar_linear_x'),
                'base_2_radar_linear_y': LaunchConfiguration('base_2_radar_linear_y'),
                'base_2_radar_linear_z': LaunchConfiguration('base_2_radar_linear_z'),
                'base_2_radar_orintation_x': LaunchConfiguration('base_2_radar_orintation_x'),
                'base_2_radar_orintation_y': LaunchConfiguration('base_2_radar_orintation_y'),
                'base_2_radar_orintation_z': LaunchConfiguration('base_2_radar_orintation_z'),
                'base_2_radar_orintation_w': LaunchConfiguration('base_2_radar_orintation_w'),
                'base_2_radar_angle_x': LaunchConfiguration('base_2_radar_angle_x'),
                'base_2_radar_angle_y': LaunchConfiguration('base_2_radar_angle_y'),
                'base_2_radar_angle_z': LaunchConfiguration('base_2_radar_angle_z'),
            }]
        )]
    )
