import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        

        # 2. 启动 Lidar Odometry 节点
        # 这里以一个假定的雷达里程计为例，你需要将 package 和 executable 替换为你实际使用的包
        # 比如 rf2o_laser_odometry 或 slam_toolbox 等
        Node(
            package='lidar_odometry',    # 替换为你实际的 Lidar Odom 包名
            executable='lidar_odometry_node', # 替换为实际的可执行文件
            name='lidar_odometry',
            output='screen',
            # parameters=[
            #     {'laser_scan_topic' : '/scan_corrected'},  # 让里程计订阅【矫正后】的雷达数据
            #     {'odom_topic' : '/odom'},
            #     {'publish_tf' : True},
            #     {'base_frame_id' : 'base_link'},
            #     {'odom_frame_id' : 'odom'},
            #     {'init_pose_from_topic' : ''},
            #     {'freq' : 20.0}
            # ],
            # 如果该节点不使用 parameters 定义话题，而是默认订阅 /scan，可以通过 remappings 强制映射
            # remappings=[
            #     ('/scan', '/scan_corrected')
            # ]
        ),

        # 1. 启动 IMU 雷达补偿节点
        Node(
            package='lidar_odometry',
            executable='scan_corrector_node',
            name='scan_corrector_node',
            output='screen',
            parameters=[
                {'debug_mode': True},       # 打开终端调试信息
                {'filter_size': 8},         # 滤波窗口大小
                {'max_angle_rad': 0.4}      # 最大允许补偿角度 (弧度)
            ],
            # remappings=[
            #     ('/scan', '/scan'),                   # 接收原始激光话题
            #     ('/imu/data', '/imu/data'),           # 接收IMU话题 (根据实机情况修改)
            #     ('/scan_corrected', '/scan_corrected')# 输出矫正后的激光话题
            # ]
        )
    ])