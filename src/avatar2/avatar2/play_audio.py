import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import Audio
from avatar2_interfaces.srv import Listen
from rclpy.qos import QoSProfile
import whisper
import tempfile
import os

class PlayText(Node):
    def __init__(self):
        super().__init__('play_audio_syncd')
        self.declare_parameter('topic', '/avatar2/out_raw_audio')
        topic = self.get_parameter('topic').get_parameter_value().string_value


        self.declare_parameter('listen', '/avatar2/listen')
        self._listen = self.get_parameter('listen').get_parameter_value().string_value
        self.create_service(Listen, self._listen, self._listener_callback)

        self._msg_id = 0
        self._seq = 0

        self._publisher = self.create_publisher(Audio, topic, QoSProfile(depth=1))
        with open("msg.wav", "rb") as f:
            self._audio_data = f.read()

    def _listener_callback(self, msg, resp):
        """Deal with service call to set listening status"""
        self.get_logger().info(f"{self.get_name()} Set listening callback to {msg.listen}")
        resp.status = msg.listen
        if msg.listen:
            self._say_next()
        return resp

    def _say_next(self):
        self.get_logger().info(f'{self.get_name()} saying something')
        msg = Audio()
        msg.audio = self._audio_data.hex()
        msg.format = "WAV_1_44100"  # known magically
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = self._seq
        self._publisher.publish(msg)
        self._seq = self._seq + 1

        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlayText()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()
