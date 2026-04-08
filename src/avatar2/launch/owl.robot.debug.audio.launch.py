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
    ros_ui = False

    config = os.path.join(get_package_share_directory('avatar2'), 'config', 'owl_robotr_params.yaml')
    print(config)
    
    nodes = []
    microphone_node = Node(
             package='avatar2',
             executable='sound_capture',
             name='sound_capture',
             output='screen',
             namespace = '/robotAvatar/avatar',
             parameters=[config])
    nodes.append(microphone_node)

    sound_to_text_node = Node(
             package='avatar2',
             executable='sound_to_text',
             name='sound_to_text',
             output='screen',
             namespace = '/robotAvatar/avatar',
             parameters=[config])
    nodes.append(sound_to_text_node)

    return LaunchDescription(nodes)
