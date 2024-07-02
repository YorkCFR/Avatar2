#
# launch the face detection/recogntion datastream. This needs to be pointed at the appropriate face database
# which is constructed using the build_database tool in the src directory
#
# Version History
#     v1.1 - some general code cleanup (and launch file renaming)
#     V1.0 - some initial hacking to get it working
#
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root_dir = '/home/baranparsai/Documents/Avatar2/scenarios/hearing_clinic/faces'
    debug = False
    for arg in sys.argv[4:]:
        if arg.startswith('root:='):
            print(f"Launching with face recognition database from {arg.split('root:=', 1)[1]}")
            root_dir = arg.split('root:=', 1)[1]

        if arg.startswith('debug'):
            print(f"Launching with debug is {arg.split('debug:=', 1)[1]}")
            debug = bool(arg.split('debug:=', 1)[1])
            
        else:
            print("Usage: launch avatar2 avatar_recognizer_video.launch.py [root:=<face_recogntion_db>] [debug:= False|True]")
            sys.exit()


    return LaunchDescription([
        Node(
             package='avatar2',
             executable='avatar_camera',
             name='avatar_camera',
             output='screen',
             namespace="/avatar2"),
        Node(
             package='avatar2',
             executable='head_detect',
             name='head_detect',
             output='screen',
             namespace="/avatar2",
             parameters=[{'root_dir' : root_dir, 'debug': debug}])
    ])

