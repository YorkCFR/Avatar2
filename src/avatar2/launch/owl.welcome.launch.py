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

    config = os.path.join(get_package_share_directory('avatar2'), 'config', 'owl_params.yaml')
    print(config)
    
    nodes = []
    microphone_node = Node(
             package='avatar2',
             executable='sound_capture',
             name='sound_capture',
             output='screen',
             namespace = '/welcomeAvatar/avatar',
             parameters=[config])
    nodes.append(microphone_node)

    sound_to_text_node = Node(
             package='avatar2',
             executable='sound_to_text',
             name='sound_to_text',
             output='screen',
             namespace = '/welcomeAvatar/avatar',
             parameters=[config])
    nodes.append(sound_to_text_node)

    camera_node = Node(
            package='avatar2',
            executable='avatar_camera',
            name='avatar_camera',
            output='screen',
            namespace = '/welcomeAvatar/avatar',
            parameters=[config])
    nodes.append(camera_node)

    sentiment_node = Node(
            package='avatar2',
            executable='sentiment_analysis',
            name='analysis',
            output='screen',
            namespace = '/welcomeAvatar/avatar',
            parameters=[config])
#    nodes.append(sentiment_node)  # currently some issue with tensorRT on my home machine

    text_to_sound = Node(
            package='avatar2',
            executable='text_to_sound',
            name='text_to_sound',
            output='screen',
            namespace = '/welcomeAvatar/avatar',
            parameters=[config])
    nodes.append(text_to_sound)

    rosbridge_node = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            namespace="/welcomeAvatar/avatar")
    nodes.append(rosbridge_node)

    face_recognizer_node = Node(
            package='avatar2',
            executable='head_detect',
            name='head_detect',
            output='screen',
            namespace = '/welcomeAvatar/avatar',
            parameters=[config])
    nodes.append(face_recognizer_node)
    
    user_tracker_node = Node(
            package='avatar2',
            executable='user_tracker',
            name='user_tracker',
            output='screen',
            namespace = '/welcomeAvatar/avatar',
            parameters=[config])
    nodes.append(user_tracker_node)

#    if ros_ui:
#        ros_node = Node(
#             package='avatar2',
#             executable='ros_avatar',
#             name='ros_avatar',
#             output='screen',
#             parameters=[{'imagery': ui_imagery, 'debug': False}])
#        nodes.append(ros_node)

    return LaunchDescription(nodes)
