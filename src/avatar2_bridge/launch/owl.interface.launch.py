import os
import sys
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(get_package_share_directory('avatar2_bridge'), 'config', 'ava_interface_params.yaml')
    print(config)
    
    nodes = []
    interface_node = Node(
             package='avatar2_bridge',
             executable='ava_bridge',
             name='ava_bridge',
             output='screen',
             namespace = '/welcomeAvatar/avatar',
             parameters=[config])
    nodes.append(interface_node)

    return LaunchDescription(nodes)
