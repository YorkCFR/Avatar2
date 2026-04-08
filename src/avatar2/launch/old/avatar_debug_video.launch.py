#
# low level tool to validate that the camera is working (debug tool)
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
             executable='avatar_camera',
             name='avatar_camera',
             output='screen',
             namespace="/avatar2",
             parameters=[{'port': 0}]),
        Node(
             package='avatar2',
             executable='avatar_camera_view',
             name='avatar_camera_view',
             output='screen',
             namespace="/avatar2",
             parameters=[{'image': '/avatar2/avatar_camera/image_raw'}])
    ])

