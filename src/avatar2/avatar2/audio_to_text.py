import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import AudioInput, TaggedString
from rclpy.qos import QoSProfile
import whisper
import tempfile
import os

class Audio2TextNode(Node):
    def __init__(self):
        super().__init__('audio_2_text_node')
        self.declare_parameter('topic', '/avatar2/audio_input')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('message', '/avatar2/message')
        message = self.get_parameter('message').get_parameter_value().string_value
        self.declare_parameter('device', 'cuda') # or cpu
        cuda = self.get_parameter('device').get_parameter_value().string_value
        self.declare_parameter('model', 'base') # or any valid whisper models
        model = self.get_parameter('model').get_parameter_value().string_value

        self._model = whisper.load_model(model, device=cuda)

        self.create_subscription(AudioInput, topic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, message, QoSProfile(depth=1))

    def _callback(self, data):
        fd, path = tempfile.mkstemp(suffix=".wav")
        with os.fdopen(fd, 'wb') as f:
            f.write(bytes.fromhex(data.audio))
        result = self._model.transcribe(path, fp16=False)
        os.remove(path)

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
