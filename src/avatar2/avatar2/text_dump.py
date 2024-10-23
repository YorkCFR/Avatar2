#
# Dump a TaggedString
#
import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import TaggedString
from rclpy.qos import QoSProfile

class TextDump(Node):
    def __init__(self):
        super().__init__('text_dump')
        
        # debug param

        self.declare_parameter('message', '/avatar2/in_message')
        message = self.get_parameter('message').get_parameter_value().string_value

        self.create_subscription(TaggedString, message, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, message, QoSProfile(depth=1))


    def _callback(self, data):
        self.get_logger().info(f"Got a TaggedString {data.text.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TextDump()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
