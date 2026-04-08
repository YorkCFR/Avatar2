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
             executable='llm_engine',
             name='llm_engine',
             output='screen',
             namespace="/avatar2",
             parameters=[{'avatar': 'dummy'}])
    ])

