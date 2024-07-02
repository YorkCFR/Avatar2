import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import Audio, TaggedString
from avatar2_interfaces.srv import Listen
from rclpy.qos import QoSProfile
import whisper
import tempfile
import os

class Audio2TextNode(Node):
    def __init__(self):
        super().__init__('audio_2_text_node')
        
        # debug param
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        self.declare_parameter('topic', '/avatar2/in_raw_audio')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('message', '/avatar2/in_message')
        message = self.get_parameter('message').get_parameter_value().string_value
        self.declare_parameter('device', 'cuda') # or cpu
        cuda = self.get_parameter('device').get_parameter_value().string_value
        self.declare_parameter('model', 'base') # or any valid whisper models
        model = self.get_parameter('model').get_parameter_value().string_value
        self.declare_parameter('listen', '/avatar2/listen')
        self._listen = self.get_parameter('listen').get_parameter_value().string_value

        self._model = whisper.load_model(model, device=cuda)

        self.create_subscription(Audio, topic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, message, QoSProfile(depth=1))

        self.create_service(Listen, self._listen, self._listener_callback)
        self._listening = True
        self._listening_time = self.get_clock().now().nanoseconds
        if self._debug:
            self.get_logger().info(f"{self.get_name()} Time {self._listening_time}")

    def _listener_callback(self, msg, resp):
        """Deal with service call to set listening status"""
        if self._debug:
            self.get_logger().info(f"{self.get_name()} Setting listening status to {msg.listen}")
        if msg.listen:
            self._listening_time = self.get_clock().now().nanoseconds
        self._listening = msg.listen
        resp.status = self._listening
        return resp

    def _callback(self, data):

        fd, path = tempfile.mkstemp(suffix=".wav")
        with os.fdopen(fd, 'wb') as f:
            f.write(bytes.fromhex(data.audio))
        result = self._model.transcribe(path, fp16=False)
        os.remove(path)

        if (not self._listening) or (self.get_clock().now().nanoseconds < self._listening_time + 4 * 1e9):
            if self._debug:
                self.get_logger().info(f"{self.get_name()} Not listening to message sequence number {data.seq} |{result['text']}|")
            return
        if self._debug:
            self.get_logger().info(f"{self.get_name()} Listening to message sequence number {data.seq} |{result['text']}|")
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = data.seq
        tagged_string.text.data = result['text']
        self._publisher.publish(tagged_string)

def main(args=None):
    rclpy.init(args=args)
    node = Audio2TextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()
