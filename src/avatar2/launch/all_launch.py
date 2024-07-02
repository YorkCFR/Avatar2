import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    #avatar_audio.launch.py
    audio_node = [
        Node(
             package='avatar2',
             executable='sound_to_text',
             name='sound_to_text',
             output='screen',
             namespace="/avatar2"),
        Node(
             package='avatar2',
             executable='text_to_sound',
             name='text_to_sound',
             output='screen',
             namespace="/avatar2")]



    #avatar_microphone.launch.py
    microphone_node = Node(
             package='avatar2',
             executable='sound_capture',
             name='sound_capture',
             output='screen',
             namespace="/avatar2",
             parameters=[{'non_speaking_duration': 1.0, 'pause_threshold': 1.0}])
    
    

    #avatar_recognizer_video.launch.py
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

    recognizer_video_node = [
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
            parameters=[{'root_dir' : root_dir, 'debug': debug}])]
        


    #avatar_video.launch.py
    video_node = [
        Node(
             package='avatar2',
             executable='avatar_camera',
             name='avatar_camera',
             output='screen',
             namespace="/avatar2"),
        Node(
             package='avatar2',
             executable='head_info',
             name='head_info',
             output='screen',
             namespace="/avatar2",
             parameters=[{'device': '0'}])]
    


    #ros_avatar.launch.py
    imagery = '/home/baranparsai/Documents/Avatar2/ros_avatar'   # default imagery location
    for arg in sys.argv[4:]: # there must be a better way...
        if arg.startswith('imagery:='):
           print(arg.split('imagery:=', 1)[1])
           imagery = arg.split('imagery:=', 1)[1]
        elif ':=' in arg:
           print(f"Unknown argument in {arg}. Usage ros2 launch ros_avatar.launch.py [imagery:=path_to_imagery] [debug:= False|True]")
           sys.exit(0)
    print(f"Launching ros_avatar.launch using imagery from {imagery}")

    ros_node = Node(
             package='avatar2',
             executable='ros_avatar',
             name='ros_avatar',
             output='screen',
             parameters=[{'imagery': imagery, 'debug': debug}])
    

    #avatar_llm_wizard_clinic.launch.py
    root = "/home/baranparsai/Documents/Avatar2/models/hearing_clinic/"  # NB: need trailing slash
    model = "WizardLM-7B-uncensored.Q4_K_M.gguf"
    format = "\n### Input: {question}\n### Response:"
    vectorstore = "hearing.pkl"
    test_cache = "test_cache.json"

    prompt = """You are a helpful assistant at the Exquisite Hearing Clinic.
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           Your name is Mary. Use the following pieces of context to answer the user's question. """
    
    llm_wizard_clinic_node = Node(
             package='avatar2',
             executable='llm_engine',
             name='llm_engine',
             output='screen',
             namespace="/avatar2",
             parameters=[{'avatar' : 'faces',
                          'root' : root,
                          'model' : model,
                          'format' : format,
                          'vectorstore' : vectorstore,
                          'prompt' : prompt, 
                          'test_cache' : test_cache}])


    return LaunchDescription([
        *audio_node,
        microphone_node,
        *recognizer_video_node,
        *video_node,
        ros_node,
        llm_wizard_clinic_node
        ])
