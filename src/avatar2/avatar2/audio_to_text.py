#
# Convert audio input (wav) into text. This uses whisper to do the actual tts work.
# Beyond this, the only thing that this node # does that is marginally interesting 
# beyond that is that it can be told to # listen or to not listen. 
#
# This is critical so that that system does not listen
# to itself 
#
# Note that there should only be one service provider running in the entire system at 
# any one time. 
#

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
        self.declare_parameter('not_listen_timeout', 2.0) # in seconds
        self._not_listen_timeout = self.get_parameter('listen').get_parameter_value().double_value * 1e9 # convert to mano seconds

        self._model = whisper.load_model(model, device=cuda)

        self.create_subscription(Audio, topic, self._audio_callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, message, QoSProfile(depth=1))

        self.create_service(Listen, self._listen, self._listener_callback)
        self._listening = True
        self._not_listening_time = 0

        if self._debug:
            self.get_logger().info(f"{self.get_name()} Time {self._not_listening_time} publishing to {message}")

    def _listener_callback(self, msg, resp):
        """Deal with service call to set listening status. If listening, we listen. Otherwise ignore messages"""
        if self._debug:
            self.get_logger().info(f"{self.get_name()} Changing listening status from {self._listening} to {msg.listen}")
        self._listening = msg.listen
        resp.status = self._listening
        if not self._listening:
            self._not_listening_time = self.get_clock().now().nanoseconds 
        return resp

    def _audio_callback(self, data):
        """Deal with an audio message"""
        if self._debug:
            # Confirm that we are receiving the message and print the sequence number
            self.get_logger().info(f"Listening to message sequence number {data.seq}")

	    # timeout if _not_listening
        if (not self._listening) and (self.get_clock().now().nanoseconds > (self._not_listening_time + self._not_listening_timeout)):
            if self._debug:
                self.get_logger().info(f"Not listening timeout. Going to start listening again (starting now)")
            self._listening = True

        # if not listening, ignore the packet
        if not self._listening:
            if self._debug:
                self.get_logger().info(f"We are not listening, so ignore the packet")
            return

        # process the packet
        fd, path = tempfile.mkstemp(suffix=".wav")
        with os.fdopen(fd, 'wb') as f:
            f.write(bytes.fromhex(data.audio))
        result = self._model.transcribe(path, fp16=False)
        if self._debug:
            self.get_logger().info(f"Transcribed result |{result['text']} |")
        os.remove(path)
        if not result['text'].isascii():
            self.get_logger().info(f"{self.get_name()} Non-ascii characters detected in the result")
        # Replace the non-ascii characters with spaces
        result['text'] = ''.join([char if char.isascii() else ' ' for char in result['text']])

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
        node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
