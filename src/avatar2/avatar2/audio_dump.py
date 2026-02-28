#
# Dump an Audio message stream as a collection of wav files.
# Possibly useful for debugging
#
import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import Audio

class DumpAudioNode(Node):
    def __init__(self):
        super().__init__('audio_dump')
        self.declare_parameter('topic', '/avatar2/in_raw_audio')
        self._topic = self.get_parameter('topic').get_parameter_value().string_value

        self._subscriber = self.create_subscription(Audio, self._topic, self._callback, 1)

    def _callback(self, data):
        with open(f"sound{data.seq}.wav", "wb") as f:
            f.write(bytes.fromhex(data.audio))

def main(args=None):
    rclpy.init(args=args)
    node = DumpAudioNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
