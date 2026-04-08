#
# This launches a wizard llm for the museum. You will likely have to change the root for your
# installation
#
#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    root = "/home/baranparsai/Documents/Avatar2/models/museum/"  # NB: need trailing slash
    model = "WizardLM-7B-uncensored.Q4_K_M.gguf"
    format = "\n### USER: {question}\n### ASSISTANT:"
    vectorstore = "museum.pkl"

    prompt = """You are an assistant at the Ontario Regiment Museum in Oshawa Ontario. 
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
                          'prompt' : prompt}])
    ])

