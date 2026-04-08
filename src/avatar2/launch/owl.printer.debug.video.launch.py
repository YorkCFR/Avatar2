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

    config = os.path.join(get_package_share_directory('avatar2'), 'config', 'owl_printer_params.yaml')
    print(config)
    
    nodes = []
    camera_node = Node(
            package='avatar2',
            executable='avatar_camera',
            name='avatar_camera',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(camera_node)

    face_recognizer_node = Node(
            package='avatar2',
            executable='head_detect',
            name='head_detect',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(face_recognizer_node)
    
    user_tracker_node = Node(
            package='avatar2',
            executable='user_tracker',
            name='user_tracker',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(user_tracker_node)

    return LaunchDescription(nodes)
