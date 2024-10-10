#
# launch the face detection/recogntion datastream. This needs to be pointed at the appropriate scenario
# in order to work properly
#
# This is a tool designed to assist in debugging the low level video recognition system
#
# Version History
#     V2.0 - moer serious cleanup
#     v1.1 - some general code cleanup (and launch file renaming)
#     V1.0 - some initial hacking to get it working
#
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

    root = '/home/baranparsai/Documents/Avatar2/scenarios'   # default location of faces.json
    scenario = 'hearing_clinic'

    for arg in sys.argv[4:]:
        if arg.startswith('scenario:='):
            scenario = arg.split('scenario:=', 1)[1]
            print(f"Launching with scenario as {scenario}")
        elif arg.startswith('root:='):
            print(f"Launching with root as {arg.split('root:=', 1)[1]}")
            root = arg.split('root:=', 1)[1]
        else:
            print("Usage: launch avatar2 avatar_debug_recognizer.launch.py [root:=<root>] [scenario:=<scenario>] [debug:=True|False]")
            sys.exit()
    config_file = os.path.join(root, scenario, 'config.json')
    print(f"Launching face recognition using config from {config_file}")

    with open(config_file) as f:
        config = json.load(f)
    print(f)
    try:
        camera_port = config['camera_port']
    except:
        print("Usage: no camera_port defined in {config_file}")
        sys.exit()

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
             parameters=[{'config_file' : config_file}]),
        Node(
             package='avatar2',
             executable='view_head_info',
             name='view_head_info',
             output='screen',
             namespace="/avatar2",
             parameters=[{'config_file': config_file}]),
    ])

