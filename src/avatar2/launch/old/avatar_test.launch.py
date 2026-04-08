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
             package='rosbridge_server',
             executable='rosbridge_websocket',
             name='rosbridge_websocket',
             output='screen'),
        Node(
             package='avatar2',
             executable='play_audio_syncd',
             name='play_audio_syncd',
             output='screen',
             namespace="/avatar2"),
    ])

