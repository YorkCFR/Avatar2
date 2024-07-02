import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import Audio
from rclpy.qos import QoSProfile
from glob import glob


class AudioInputWavNode(Node):
    def __init__(self):
        super().__init__('audio_input_wav')
        self._msg_id = 0
        # debug param
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        self.declare_parameter('topic', '/avatar2/audio_input')
        self._topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('source', "./*.wav")
        self._source = self.get_parameter('source').get_parameter_value().string_value
        self.declare_parameter('period', 1.0)
        self._period = self.get_parameter('period').get_parameter_value().double_value
        self._publisher = self.create_publisher(Audio, self._topic, QoSProfile(depth=1))

        self._files = glob(self._source)
        if self._debug:
            self.get_logger().info(f"{self.get_name()} will play {self._files}")
        self._msg_id = 0
        self._file_id = 0

        self.create_timer(self._period, self._timer_callback)

    def _timer_callback(self):
        if self._debug:
            self.get_logger().info(f"{self.get_name()} playing {self._files[self._file_id]}")
        with open(f"{self._files[self._file_id]}", "rb") as f:
            data = f.read()

            msg = Audio()
            msg.audio = data.hex()
            msg.format = "WAV_1_44100"  # great assumption here
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.seq = self._msg_id
            self._msg_id = self._msg_id + 1
            self._publisher.publish(msg)
            if self._debug:
                self.get_logger().info(f"Audio source sent {len(msg.audio)} samples")
            self._file_id = (self._file_id + 1) % len(self._files)

def main(args=None):
    rclpy.init(args=args)
    node = AudioInputWavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
