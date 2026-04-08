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

    camera_view_node = Node(
            package='avatar2',
            executable='avatar_camera_view',
            name='avatar_camera_view',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(camera_view_node)

    face_recognizer_node = Node(
            package='avatar2',
            executable='head_detect',
            name='head_detect',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(face_recognizer_node)

    face_recognizer_view_node = Node(
            package='avatar2',
            executable='view_head_info',
            name='view_head_info',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(face_recognizer_view_node)
    
    user_tracker_node = Node(
            package='avatar2',
            executable='user_tracker',
            name='user_tracker',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(user_tracker_node)

    user_monitor_node = Node(
            package='avatar2',
            executable='user_monitor',
            name='user_monitor',
            output='screen',
            namespace = '/printerAvatar/avatar',
            parameters=[config])
    nodes.append(user_monitor_node)
    
    microphone_node = Node(
             package='avatar2',
             executable='sound_capture',
             name='sound_capture',
             output='screen',
             namespace = '/printerAvatar/avatar',
             parameters=[config])
    nodes.append(microphone_node)

    sound_to_text_node = Node(
             package='avatar2',
             executable='sound_to_text',
             name='sound_to_text',
             output='screen',
             namespace = '/printerAvatar/avatar',
             parameters=[config])
    nodes.append(sound_to_text_node)

    return LaunchDescription(nodes)
