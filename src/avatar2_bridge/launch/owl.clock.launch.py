import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(get_package_share_directory('avatar2_bridge'), 'config', 'ava_clock_params.yaml')
    print(config)
    
    nodes = []
    interface_node = Node(
             package='avatar2_bridge',
             executable='ava_bridge',
             name='ava_bridge',
             output='screen',
             namespace = '/clockAvatar/avatar',
             parameters=[config])
    nodes.append(interface_node)

    rosbridge_node = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            namespace="/clockAvatar/avatar")
    nodes.append(rosbridge_node)

    return LaunchDescription(nodes)
