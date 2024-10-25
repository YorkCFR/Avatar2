import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import TaggedString
from avatar2_interfaces.srv import Listen
from rclpy.qos import QoSProfile
import whisper
import tempfile
import os

class PlayText(Node):
    def __init__(self):
        super().__init__('play_text_node')
        self.declare_parameter('topic', '/avatar2/out_message')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('period', 0.5)
        self._period = self.get_parameter('period').get_parameter_value().double_value
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('listen', '/avatar2/listen')
        self._listen = self.get_parameter('listen').get_parameter_value().string_value
        self.create_service(Listen, self._listen, self._listener_callback)

        self._messages = ["Welcome my son", "Welcome to the machine", "Where have you been?", "Its alright we know where youv been", 
                          "You ve been in the pipeline", "Filling in time", "Provided with toys and scouting for boys", 
                          "You bought a guitar to punish your ma.", "And you didn't like school", "And you know your nobodys fool", 
                          "So welcome to the machine", "Welcome my son", "Welcome to the machine", "What did you dream?", 
                          "Its alright we told you what to dream", "You dreamed of a big star", "He played a mean guitar", 
                          "He always ate in the Steak Bar", "He loved to drive in his Jaguar", "So welcome to the machine"]
        self._msg_id = 0
        self._seq = 0

        self._reset = True
        self._reset_count = 0


        self._publisher = self.create_publisher(TaggedString, topic, QoSProfile(depth=1))


    def _listener_callback(self, msg, resp):
        """Deal with service call to set listening status. The avatar tells us if it is listening or speaking. We don't
           send anything if it is speaking. But we want to wait for the avatar to have spoken. """
        self.get_logger().info(f"Listening with reset {self._reset} {self._reset_count} and am listening is {msg.listen}")
        if msg.listen: # avatar says it is listening
            if self._reset: # if we have reset
                self._say_next()
                self._reset = False
                resp.status = True
                self._reset_count = 0
            else:
                resp.status = False
                self._reset_count = self._reset_count + 1
        else:
            self._reset = True
            resp.status = False
            self._reset_count = self._reset_count + 1

        if self._reset_count > 100:
            self._reset_count = 0
            self._reset = True
        return resp

    def _say_next(self):
        self.get_logger().info(f'saying {self._messages[self._msg_id]}')
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = self._seq
        self._seq = self._seq + 1
        tagged_string.text.data = self._messages[self._msg_id]
        self._msg_id = (self._msg_id+1) % len(self._messages)
        self._publisher.publish(tagged_string)

def main(args=None):
    rclpy.init(args=args)
    node = PlayText()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
