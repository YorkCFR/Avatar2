import json
import rclpy
import queue
import asyncio
import threading
import websockets
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import TaggedString

#	ws<->ros command 
#
#       All messages have a cmd (command) - string
#       All messages have a dest (destination) one of the avatars - string
#       All messages have an arg (argument) - a string that parses as a valid json structure
#
#	Supported to ros messages (cmd)
#	    say - arg has at least the entry text whose value is what to utter
#           action - arg has at least the the entry act whose value is a string 
#          
#       Suppored from ros message (cmd)
#           heard - arg has at least the entry text whose value is what the llm heard
#


class AvaBridgeNode(Node):

    def __init__(self, ws_queue):
        super().__init__('ava_bridge')
        self.get_logger().info(f"{self.get_name()} node created.")
        self._ws_queue = ws_queue
        self._connected_clients = set()
        self._loop = None
        self._seq_number = 0
        
        self.declare_parameter('debug', True)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'Node created, debug is {self._debug}')

        self.declare_parameter('ipaddr', '127.0.0.1')
        self._ipaddr = self.get_parameter('ipaddr').get_parameter_value().string_value
        self.declare_parameter('port', 8765)
        self._port = self.get_parameter('port').get_parameter_value().integer_value

        self.declare_parameter('avatar_names', ['dummy'])
        self._avatar_names = self.get_parameter('avatar_names').get_parameter_value().string_array_value

        self.declare_parameter('in_message', '/avatar/in_message')
        self._in_topic = self.get_parameter('in_message').get_parameter_value().string_value
        
        self.declare_parameter('out_message', '/avatar/out_message')
        self._out_topic = self.get_parameter('out_message').get_parameter_value().string_value

        self.declare_parameter('out_command', '/avatar/out_command')
        self._out_command = self.get_parameter('out_command').get_parameter_value().string_value

        if self._debug:
            self.get_logger().info(f"ipaddr {self._ipaddr} port {self._port}")
            self.get_logger().info(f"avatar_names {self._avatar_names} in_topic {self._in_topic} out_topic {self._out_topic} out_command {self._out_command}")

        for name in self._avatar_names:
            topic = "/" + name + self._in_topic
            if self._debug:
                self.get_logger().info(f"creating callback for {topic}")
            self.create_subscription(TaggedString, topic, lambda msg: self._in_message_callback(msg, name), QoSProfile(depth=1))
            self.create_subscription(TaggedString, "/" + name + "/avatar/avatar_status", lambda msg: self._in_message_callback(msg, name), QoSProfile(depth=1))

        self._out_command_publisher = [None] * len(self._avatar_names)
        self._out_message_publisher = [None] * len(self._avatar_names)
        for idx, name in enumerate(self._avatar_names):
            topic = "/" + name + self._out_topic
            if self._debug:
                self.get_logger().info(f"creating publisher for {topic}")
            self._out_message_publisher[idx]  = self.create_publisher(TaggedString, topic, QoSProfile(depth=1))
            topic = "/" + name + self._out_command
            if self._debug:
                self.get_logger().info(f"creating publisher for {topic}")
            self._out_command_publisher[idx]  = self.create_publisher(TaggedString, topic, QoSProfile(depth=1))
            topic = name + self._out_command

    def _in_message_callback(self, msg, source):
        """ Process any of the text inputs"""
        if self._debug:
            self.get_logger().info(f"got input text {msg} from {source}")
        
        package = '{"cmd" : "heard", "dest" : "' 
        package = package + source + '", "arg" : '
        package = package + '{"text" : ' + json.dumps(str(msg.text.data)) + '}}'
        if self._loop:
            self.get_logger().info(f"Forwarding message from ROS->WS")
            asyncio.run_coroutine_threadsafe(
                self._broadcast(package),
                self._loop
            )
        else:
            self.get_logger().info(f"unable to forward message from ROS->WS")
            
        
    def ProcessMessage(self, msg, websocket):
        """ Process a message from the outsde world """
        try:
            package = json.loads(msg)
            cmd = package['cmd']
            dest = package['dest']
            arg = package['args']
        except Exception as e:
            self.get_logger().info(f"unable to parse message from remote llm {msg}")
            return
        who = self._avatar_names.index(dest)
        full_name = '/'  + dest + self._out_topic
            
        if self._debug:
            self.get_logger().info(f"Received message: command {cmd} dest {dest} index {who} argument {arg}")
            full_name = '/'  + dest + self._out_topic
            self.get_logger().info(f"going to publish to {full_name}")

        if cmd == 'say':
            self.get_logger().info(f"shoud emit a string message to {dest} with argument {arg['text']}")
            tagged_string = TaggedString()
            tagged_string.header.stamp = self.get_clock().now().to_msg()
            tagged_string.audio_sequence_number = self._seq_number
            tagged_string.text.data = arg['text']
            self._out_message_publisher[who].publish(tagged_string)
            self._seq_number = self._seq_number + 1
        elif cmd == 'action':
            self.get_logger().info(f"shoud tell avatar at {dest} to conduct {arg}")
        else:
            self.get_logger().info(f"no idea what {cmd} is")

    async def _broadcast(self, message):
        """
        Sends a message to all connected Unity clients.
        Uses asyncio.gather to send to all clients simultaneously.
        """
        if self._connected_clients:
            await asyncio.gather(
                *[client.send(message) for client in self._connected_clients]
            )

    def set_loop(self, loop):
        """
        Gives ROS node a reference to the asyncio event loop running in the WebSocket server thread.
        This allows to schedule broadcasts from ROS threads to the WebSocket clients.
        """
        self._loop = loop

    def register_client(self, websocket):
        """
        Adds a new WebSocket client to the set of connected clients.
        """
        self._connected_clients.add(websocket)
        self.get_logger().info(f"Client connected: {websocket.remote_address}")

    def unregister_client(self, websocket):
        """
        Removes a WebSocket client from the set of connected clients.
        """
        self._connected_clients.discard(websocket)
        self.get_logger().info(f"Client disconnected: {websocket.remote_address}")
            
def run_websocket_server(nows_queue, node, loop, ipaddr, port):
    """
    Runs in a background thread.
    Starts the asyncio event loop and WebSocket server.
    Accepts connections and registers them with the AvaBridgeNode.
    """

    if node._debug:
        node.get_logger().info(f"websocker_server thread....")

    asyncio.set_event_loop(loop)
    node.set_loop(loop)

    async def handler(websocket):
        node.register_client(websocket)
        try:
            while True:
                async for message in websocket:
                    if node._debug:
                        node.get_logger().info(f"Received message from client: {message}")
                    node.ProcessMessage(message, websocket)
        except websockets.exceptions.ConnectionClosed:
            if node._debug:
                node.get_logger().info(f"looping for more messages")
            pass
        finally:
            node.unregister_client(websocket)

    async def serve():
        async with websockets.serve(handler, ipaddr, port):
            node.get_logger().info(f"WebSocket server started on port ws://{ipaddr}:{port}")
            await asyncio.Future()  # run forever

    loop.run_until_complete(serve())

def main(args=None):
    rclpy.init(args=args)

    # Shared queue for thread-safe communication
    ws_queue = queue.Queue()
    node = AvaBridgeNode(ws_queue)

    # Start WebSocket server in a background thread BEFORE ROS spins, so it can register clients and set the event loop reference in the node.
    ws_loop = asyncio.new_event_loop()
    ws_thread = threading.Thread(
        target=run_websocket_server, 
        args=(ws_queue, node, ws_loop, node._ipaddr, node._port), 
        daemon=True
        )
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ws_thread.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
