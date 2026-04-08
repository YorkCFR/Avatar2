#
# This launches a wizard llm for the museum. You will likely have to change the root for your
# installation
#
#
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

root = "/home/baranparsai/Documents/Avatar2/models/hearing_clinic/"  # NB: need trailing slash
model = "WizardLM-7B-uncensored.Q4_K_M.gguf"
for arg in sys.argv[3:]: # there must be a better way...
    if arg.startswith('root:='):
        root = arg.split('root:=', 1)[1]
    if arg.startswith('model:='):
        model = arg.split('model:=', 1)[1]
    elif ':=' in arg:
        print(f"Unknown argument in {arg}. Usage ros2 launch ros_avatar.launch.py [imagery:=path_to_imagery]")
        sys.exit()
print(f'Launching with root {root} and model {model}')

def generate_launch_description():
    format = "\n### USER: {question}\n### ASSISTANT:"
    vectorstore = "hearing.pkl"
    test_cache = "test_cache.json"

    prompt = """You are a helpful assistant at the Exquisite Hearing Clinic.
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           Your name is Mary. Use the following pieces of context to answer the user's question. """
    return LaunchDescription([

        Node(
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
    ])

