import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import TaggedString
from rclpy.qos import QoSProfile
from .llm import LLM
from .llm_dummy import LLMDummy
from .llm_langchain import LLMLangChain
import os


class LLMEngine(Node):
    def __init__(self):
        super().__init__('llm_engine_node')
        self.declare_parameter('out_topic', '/avatar2/out_message')
        outTopic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.declare_parameter('in_topic', '/avatar2/in_message')
        inTopic = self.get_parameter('in_topic').get_parameter_value().string_value
        
        self.declare_parameter('avatar', 'dummy')
        avatar_type = self.get_parameter('avatar').get_parameter_value().string_value

        self.get_logger().info(f'{self.get_name()} Firing up an avatar of type {avatar_type}')
        if avatar_type == 'dummy':
            self._llm = LLMDummy()
        elif avatar_type == 'langchain':
            self._llm = LLMLangChain()
        else:
            self.get_logger().info(f'{self.get_name()} {avatar_type} not known, using dummy')
            self._llm = LLMDummy()

        self.create_subscription(TaggedString, inTopic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, outTopic, QoSProfile(depth=1))


    def _callback(self, msg):
        """Deal with translation"""
        self.get_logger().info(f"{self.get_name()} listening got {msg.text.data}")

        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.audio_sequence_number
        tagged_string.text.data = "hello nurse"
        z = self._llm.response(text=msg.text.data)
        tagged_string.text.data = str(z)
        self._publisher.publish(tagged_string)
        self.get_logger().info(f"{self.get_name()} published {msg.text.data}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()
