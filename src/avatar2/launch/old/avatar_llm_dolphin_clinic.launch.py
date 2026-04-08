#
# This launches a dolphin llm for the hearing clinic. You will likely have to change the root for your
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
    root = "/home/baranparsai/Documents//Avatar2/models/hearing_clinic/"  # NB: need trailing slash
    model = "dolphin-2.1-mistral-7b.Q5_K_S.gguf"
    format = "<|im_end|>\n<|im_start|>user\n{question}<|im_end|>\n<|im_start|>assistant"
    vectorstore = "hearing.pkl"

    prompt = """<|im_start|>system
           You are a helpful assistant at the Exquisite Hearing Clinic.
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           Your name is Mary. Use the following pieces of context to answer the user's question.\n """
    return LaunchDescription([

        Node(
             package='avatar2',
             executable='llm_engine',
             name='llm_engine',
             output='screen',
             namespace="/avatar2",
             parameters=[{'avatar' : 'langchain',
                          'root' : root,
                          'model' : model,
                          'format' : format,
                          'vectorstore' : vectorstore,
                          'prompt' : prompt}])
    ])

