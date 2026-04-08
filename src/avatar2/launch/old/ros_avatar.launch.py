import sys
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    imagery = '/home/baranparsai/Documents/Avatar2/ros_avatar'   # default imagery location
    debug = False
    for arg in sys.argv[4:]:
        if arg.startswith('imagery:='):
           print(arg.split('imagery:=', 1)[1])
           imagery = arg.split('imagery:=', 1)[1]

        elif ':=' in arg:
           print(f"Unknown argument in {arg}. Usage ros2 launch ros_avatar.launch.py [imagery:=path_to_imagery] [debug:=False|True]")
           sys.exit(0)
    print(f"Launching ros_avatar.launch using imagery from {imagery}")

    return LaunchDescription([
        Node(
             package='avatar2',
             executable='ros_avatar',
             name='ros_avatar',
             output='screen',
             parameters=[{'imagery': imagery, 'debug': debug}]),
    ])

