import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # every owl screen runs only its Unity scene and connects to the rosbridge started here
    # this file is used to launch all the owl screens at once, each in its own namespace, so that they can be run on the same machine
    screens = {'welcomeAvatar': 'ava_interface_params.yaml',
               'clockAvatar': 'ava_clock_params.yaml',
               'printerAvatar': 'ava_printer_params.yaml'}

    nodes = []
    for name, params in screens.items():
        config = os.path.join(get_package_share_directory('avatar2_bridge'), 'config', params)
        print(config)
        interface_node = Node(
                 package='avatar2_bridge',
                 executable='ava_bridge',
                 name='ava_bridge',
                 output='screen',
                 namespace = '/' + name + '/avatar',
                 parameters=[config])
        nodes.append(interface_node)

    rosbridge_node = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            namespace="/owl")
    nodes.append(rosbridge_node)

    return LaunchDescription(nodes)
