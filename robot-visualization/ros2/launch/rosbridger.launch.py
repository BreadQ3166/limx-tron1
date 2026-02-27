from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import sys


def generate_launch_description():

    # Robot ip address
    # os.environ['MROS_AGENT_IP'] = '10.192.1.2'
    os.environ['MROS_AGENT_IP'] = '127.0.0.1'

    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_mros2ros",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "mros2ros_include",
            default_value="/tf;/tf_static;/odom",
        ),
        Node(
            package='mrosbridger',
            executable='mrosbridger',
            output='screen',
            parameters=[
                {
                    "bridge_mros2ros": LaunchConfiguration("bridge_mros2ros"),
                    "mros2ros_include": LaunchConfiguration("mros2ros_include"),
                },
            ],
        ),
    ])