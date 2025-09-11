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
    root = '/home/walleed/Avatar2/scenarios'   # default location of config files
    scenario = 'hearing_clinic'
    config_file = os.path.join(root, scenario, 'config.json')
    debug = True  # Enable debug for testing
    ui_imagery = '/home/walleed/Avatar2/ros_avatar'

    # Parse command line arguments
    for arg in sys.argv[4:]:
        if arg.startswith('scenario:='):
            scenario = arg.split('scenario:=', 1)[1]
            print(f"Launching with scenario as {scenario}")
        elif arg.startswith('root:='):
            root = arg.split('root:=', 1)[1]
            print(f"Launching with root as {root}")
        elif arg.startswith('debug:='):
            debug = bool(arg.split('debug:=', 1)[1].lower() == 'true')
            print(f"Launching with debug as {debug}")
        elif arg.startswith('ui_imagery:='):
           ui_imagery = arg.split('ui_imagery:=', 1)[1]
           print(f"Launching with imagery at {ui_imagery}")
        else:
            print("Usage: ros2 launch avatar2 avatar_debug_full_pipeline.launch.py [root:=<root>] [scenario:=<scenario>] [debug:=true|false]")
    
    config_file = os.path.join(root, scenario, 'config.json')
    print(f"Using config file: {config_file}")
    
    # Verify config file exists
    if not os.path.exists(config_file):
        print(f"ERROR: Config file not found at {config_file}")
        sys.exit(1)
    
    nodes = []
    
    # Audio capture node
    microphone_node = Node(
        package='avatar2',
        executable='sound_capture',
        name='sound_capture',
        output='screen',
        namespace="/avatar2",
        parameters=[{'non_speaking_duration': 1.0, 'pause_threshold': 1.0, 'debug': debug}])
    nodes.append(microphone_node)

    # Speech to text node
    sound_to_text_node = Node(
        package='avatar2',
        executable='sound_to_text',
        name='sound_to_text',
        output='screen',
        namespace="/avatar2",
        parameters=[{'debug': debug}])
    nodes.append(sound_to_text_node)

    # Sentiment analysis node
    sentiment_node = Node(
        package='avatar2',
        executable='sentiment_analysis',
        name='sentiment_analysis',
        output='screen',
        namespace="/avatar2")
    nodes.append(sentiment_node)

    # LLM engine node
    llm_engine_node = Node(
        package='avatar2',
        executable='llm_engine',
        name='llm_engine',
        output='screen',
        namespace="/avatar2",
        parameters=[{
            'root': root, 
            'scenario': scenario, 
            'config_file': 'config.json'
        }])
    nodes.append(llm_engine_node)
    
    # Text to speech node
    text_to_sound_node = Node(
        package='avatar2',
        executable='text_to_sound',
        name='text_to_sound',
        output='screen',
        namespace="/avatar2",
        parameters=[{
            'debug': debug,
            'voice_model': '/home/walleed/Avatar2/tts_models/en_US-lessac-high.onnx'
        }])
    nodes.append(text_to_sound_node)

    # ROS Avatar node
    ros_avatar_node = Node(
        package='avatar2',
        executable='ros_avatar',
        name='ros_avatar',
        output='screen',
        namespace="/avatar2",
        parameters=[{'imagery': ui_imagery, 'debug': debug}])
    nodes.append(ros_avatar_node)

    # Text dump node for debugging
    text_dump_node = Node(
        package='avatar2',
        executable='text_dump',
        name='text_dump',
        output='screen',
        namespace="/avatar2")
    nodes.append(text_dump_node)

    print("=== Avatar2 Debug Pipeline ===")
    print("Nodes launched:")
    print("- sound_capture: Audio input")
    print("- sound_to_text: Speech recognition")
    print("- sentiment_analysis: Emotion detection")
    print("- llm_engine: Language model processing")
    print("- text_to_sound: Text to speech")
    print("- ros_avatar: Avatar display")
    print("- text_dump: Output debugging")
    print("=============================")

    return LaunchDescription(nodes)
