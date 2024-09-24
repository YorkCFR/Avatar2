# 
# This does *only* the langchain part of things. Its only claim to fame is that it
# starts processing requests immediately, not just when the llm is loaded 
#

from threading import Thread
import random
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from langchain_community.llms import LlamaCpp
from avatar2_interfaces.msg import TaggedString

class LLMOnly(Node):
    def __init__(self):
        super().__init__('llm_only_node')
        
        # debug param
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        self.declare_parameter('in_topic', '/avatar2/in_raw_prompt')
        in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        self.declare_parameter('out_topic', '/avatar2/response')
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value

        self.declare_parameter('excuses', '["I am sorry, I cant do that right now", "My shift starts in a few minutes", "Ask me next week"]')
        self._excuses = json.loads(self.get_parameter('excuses').get_parameter_value().string_value)

        self.declare_parameter('model', 'model.file')
        self._model = self.get_parameter('model').get_parameter_value().string_value

        self._publisher = self.create_publisher(TaggedString, out_topic, QoSProfile(depth=1))
        self.create_subscription(TaggedString, in_topic, self._callback, QoSProfile(depth=1))


        self._loaded = False
        self._clean = False
        self._thread = Thread(target=self._thread, args=[None])
        self._thread.start()
        self.get_logger().info(f"{self.get_name()} publishing to {out_topic}")

    def _thread(self, args):
        max_vectors=2
        n_ctx=2048,
        temperature=0,
        n_gpu_layers=26
        self.get_logger().info(f"{self.get_name()} llm loading started {self._model}")
#        self._llm = LlamaCpp(model_path=self._model, temperature=0, n_ctx=2048, n_gpu_layers=26)
        self._llm = LlamaCpp(model_path=self._model, temperature=0)
        self.get_logger().info(f"{self.get_name()} llm loaded async")
        self._loaded = True

    def _callback(self, text):
        self.get_logger().info(f"{self.get_name()} processing  {text.text.data}")
        if self._loaded == False:
            self.get_logger().info(f"{self.get_name()} Not done")
        elif self._loaded:
            if self._clean == False:
                self._clean = True
                self.get_logger().info(f"{self.get_name()}  done first time {self._clean}")
                self._thread.join()
        
        if self._loaded == True:
            resp = self._llm(text.text.data)
        else:
            resp = random.choice(self._excuses)
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = text.audio_sequence_number
        tagged_string.text.data = resp
        self._publisher.publish(tagged_string)


def main(args=None):
    rclpy.init(args=args)
    node = LLMOnly()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

