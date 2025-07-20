#
# low level tool to validate that the input audio mechanism is working
# Note: audio comes from the *default* input audio device. Under Ubuntu you
# can select this using the Sound setting (found from the Settings choice in the power icon
# on the far right of the menu bar
#
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
             parameters=[{'non_speaking_duration': 1.0, 'pause_threshold': 1.0, 'debug': True}]),
        Node(
             package='avatar2',
             executable='sentiment_analysis',
             name='sentiment_analysis',
             output='screen'),
        Node(
             package='avatar2',
             executable='sound_to_text',
             name='sound_to_text',
             output='screen',
             parameters=[{'debug': False}]),
        Node(
             package='avatar2',
             executable='text_dump',
             name='text_dump',
             output='screen'),
    ])
