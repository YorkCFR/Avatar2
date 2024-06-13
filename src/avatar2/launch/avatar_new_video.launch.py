import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
             package='avatar2',
             executable='avatar_camera',
             name='avatar_camera',
             output='screen',
             namespace="/avatar2"),
        Node(
             package='avatar2',
             executable='head_detect',
             name='head_detect',
             output='screen',
             namespace="/avatar2")
    ])

