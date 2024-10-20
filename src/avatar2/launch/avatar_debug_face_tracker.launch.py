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
    root = './scenarios'        # default location of scenarios
    scenario = 'hearing_clinic' # default scenario

    for arg in sys.argv[4:]:
        if arg.startswith('scenario:='):
            scenario = arg.split('scenario:=', 1)[1]
            print(f"Launching with scenario as {scenario}")
        elif arg.startswith('root:='):
            print(f"Launching with root as {arg.split('root:=', 1)[1]}")
            root = arg.split('root:=', 1)[1]
        else:
            print("Usage: launch avatar2 avatar_debug_face_tracker.launch.py [root:=<root>] [scenario:=<scenario>]")
            sys.exit()
    root = os.path.abspath(root)
    config_file = os.path.join(root, scenario, 'config.json')
    print(f"Launching face tracker using config from {config_file}")

    with open(config_file) as f:
        config = json.load(f)
    print(f)
    try:
        camera_port = config['camera_port']
    except:
        print(f"Usage: no camera_port defined in {config_file}")
        sys.exit()
    print(f"Launching with camera port {camera_port}")

    return LaunchDescription([
        Node(
             package='avatar2',
             executable='avatar_camera',
             name='avatar_camera',
             output='screen',
             namespace="/avatar2",
             parameters=[{'port': camera_port}]),
        Node(
             package='avatar2',
             executable='head_detect',
             name='head_detect',
             output='screen',
             namespace="/avatar2",
             parameters=[{'root' : root, 'scenario': scenario}]),
        Node(
             package='avatar2',
             executable='user_tracker',
             name='user_tracker',
             output='screen',
             namespace="/avatar2",
             parameters=[{'root' : root, 'scenario': scenario}]),
        Node(
             package='avatar2',
             executable='user_monitor',
             name='user_monitor',
             output='screen',
             namespace="/avatar2"),
    ])

