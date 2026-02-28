import json
import rclpy
import queue
import asyncio
import threading
import websockets
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import TaggedString

class AvaBridgeNode(Node):

    def __init__(self, ws_queue):
        super().__init__('ava_bridge')
        self.get_logger().info(f"{self.get_name()} node created.")
        self._ws_queue = ws_queue
        self._connected_clients = set()
        self._loop = None
        
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'Node created, debug is {self._debug}')

        self.declare_parameter('ipaddr', '127.0.0.1')
        self._ipaddr = self.get_parameter('ipaddr').get_parameter_value().string_value
        self.declare_parameter('port', 8765)
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self.declare_parameter('out_message', '/avatar2/out_message')
        self._out_message = self.get_parameter('out_message').get_parameter_value().string_value
        self.declare_parameter('out_command', '/avatar2/out_command')
        self._out_command = self.get_parameter('out_command').get_parameter_value().string_value

#        self.create_subscription(TaggedString, self._out_topic, self._stt_callback, QoSProfile(depth=1))

        # Poll for WebSocket messages every 100ms
        # self.create_timer(0.1, self._process_ws_messages)

        if self._debug:
            self.get_logger().info(f'AvaBridge started, publish to {self._out_message} and {self._out_command}, WebSocket {self._ipaddr} port {self._port}')
        
    def ProcessMessage(self, msg, websocket):
        """ Process a message from the outsde world """
        package = json.loads(msg)
        self.get_logger().info(f"Received message: {msg} command {package['command']} argument {package['argument']}")
        websocket.send("got it")

        

    def _stt_callback(self, msg):
        """
        Callback for STT transcribed speech
        Packages it as JSON and forwards it to Unity Owl via via WebSocket
        """
        text = msg.text.data
        self.get_logger().info(f"Received message: {msg.text.data}")

        payload = json.dumps({
            "command": "say",
            "argument": text
        })

        if self._loop:
            asyncio.run_coroutine_threadsafe(
                self._broadcast(payload), 
                self._loop
            )

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
