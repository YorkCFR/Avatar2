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
             executable='sound_capture',
             name='sound_capture',
             output='screen',
             namespace="/avatar2",
             parameters=[{'non_speaking_duration': 1.0, 'pause_threshold': 1.0}]),
    ])

