import os
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import SpeakerInfo, TaggedString

class LLMCore(Node):
    def __init__(self):
        super().__init__('llm_core_node')
        self.declare_parameter('role_topics', {
            'visitor': '/visitor_topic',
            'student': '/student_topic',
            'professor': '/professor_topic',
            'dean': '/dean_topic'})
        
        self.role_topics = self.get_parameter('role_topics').get_parameter_value().string_value
        self.create_subscription(SpeakerInfo, 'face_recognition_topic', self.callback_face_recognition, QoSProfile(depth=1))
        
        self.publishers = {
            'visitor': self.create_publisher(TaggedString, self.role_topics['visitor'], QoSProfile(depth=1)),
            'student': self.create_publisher(TaggedString, self.role_topics['student'], QoSProfile(depth=1)),
            'professor': self.create_publisher(TaggedString, self.role_topics['professor'], QoSProfile(depth=1)),
            'dean': self.create_publisher(TaggedString, self.role_topics['dean'], QoSProfile(depth=1))}
        
    def callback_face_recognition(self, msg):
        try:
            info = json.loads(msg.info.data)
            role = info.get('role', 'visitor')
        except Exception as e:
            self.get_logger().error(f"Error parsing info: {e}")
            role = 'visitor'
        
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.seq
        tagged_string.text.data = json.dumps(info)  # Pass the whole info to LLM
        
        if role not in self.publishers:
            role = 'visitor'
        
        self.publishers[role].publish(tagged_string)
        self.get_logger().info(f"Published message to {role} topic")

def main(args=None):
    rclpy.init(args=args)
    node = LLMCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
