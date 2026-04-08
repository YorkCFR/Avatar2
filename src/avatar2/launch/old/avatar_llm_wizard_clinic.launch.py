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
<<<<<<< HEAD
    root = "/home/baranparsai/Documents/Avatar2/models/hearing_clinic/"  # NB: need trailing slash
=======
    root = "/home/walleed/Avatar2/models/hearing_clinic/"  # NB: need trailing slash
>>>>>>> 869f8c6687afe53d29982b712565a199564a474e
    model = "WizardLM-7B-uncensored.Q4_K_M.gguf"
    format = "\n### Input: {question}\n### Response:"
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

