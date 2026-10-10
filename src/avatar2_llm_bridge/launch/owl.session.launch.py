import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(get_package_share_directory('avatar2_llm_bridge'), 'config', 'owl_llm_params.yaml')
    session_config = os.path.join(get_package_share_directory('avatar2_llm_bridge'), 'config', 'owl_session_params.yaml')
    print(config)
    print(session_config)

    nodes = []
    llm_node = Node(
             package='avatar2_llm_bridge',
             executable='avatar2_llm_bridge_node',
             name='avatar2_llm_bridge',
             output='screen',
             parameters=[config, session_config])
    nodes.append(llm_node)

    return LaunchDescription(nodes)
