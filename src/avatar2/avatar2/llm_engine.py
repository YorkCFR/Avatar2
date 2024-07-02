import os
import rclpy
import string
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import TaggedString
from .llm import LLM
from .llm_dummy import LLMDummy
from .llm_langchain import LLMLangChain
from .llm_withfaces import LLMWithFaces
from .llm_local_cache import LocalCache

class LLMEngine(Node):
    def __init__(self):
        super().__init__('llm_engine_node')
        
        # debug param
        self.declare_parameter('debug', True)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        self.declare_parameter('out_topic', '/avatar2/out_message')
        outTopic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.declare_parameter('in_topic', '/avatar2/in_message')
        inTopic = self.get_parameter('in_topic').get_parameter_value().string_value
        
        self.declare_parameter('avatar', 'dummy')
        avatar_type = self.get_parameter('avatar').get_parameter_value().string_value

        if self._debug:
            self.get_logger().info(f'{self.get_name()} Firing up an avatar of type {avatar_type}')
            
        if avatar_type == 'dummy':
            self._llm = LLMDummy()

        elif avatar_type == 'langchain':
            self.declare_parameter('root', './museum/')
            root = self.get_parameter('root').get_parameter_value().string_value

            self.declare_parameter('model', 'some.gguf')
            model = root + self.get_parameter('model').get_parameter_value().string_value

            self.declare_parameter('prompt', 'You are an AI assistant. Answer questions.')
            prompt = self.get_parameter('prompt').get_parameter_value().string_value

            self.declare_parameter('vectorstore', 'vectorstore.pkl')
            vectorstore = root + self.get_parameter('vectorstore').get_parameter_value().string_value

            self.declare_parameter('format', '\n###USER: {question}\n###ASSISTANT:')
            format = self.get_parameter('format').get_parameter_value().string_value
            
            self.declare_parameter('test_cache', 'test_cache.json')
            test_cache = root + self.get_parameter('test_cache').get_parameter_value().string_value

            self._llm = LLMLangChain(model=model, prompt=prompt, vectorstore=vectorstore, format=format)
            self.local_cache = LocalCache(node=self, filename=test_cache)
        elif avatar_type == 'faces':
            self.declare_parameter('root', './museum/')
            root = self.get_parameter('root').get_parameter_value().string_value

            self.declare_parameter('model', 'some.gguf')
            model = root + self.get_parameter('model').get_parameter_value().string_value

            self.declare_parameter('prompt', 'You are an AI assistant. Answer questions.')
            prompt = self.get_parameter('prompt').get_parameter_value().string_value

            self.declare_parameter('vectorstore', 'vectorstore.pkl')
            vectorstore = root + self.get_parameter('vectorstore').get_parameter_value().string_value

            self.declare_parameter('format', '\n###USER: {question}\n###ASSISTANT:')
            format = self.get_parameter('format').get_parameter_value().string_value
            
            self.declare_parameter('test_cache', 'test_cache.json')
            test_cache = root + self.get_parameter('test_cache').get_parameter_value().string_value

            self._llm = LLMWithFaces(model=model, prompt=prompt, vectorstore=vectorstore, format=format, node=self)
            self.local_cache = LocalCache(node=self, filename=test_cache)
        else:
            self._llm = LLMDummy()
            if self._debug:
                self.get_logger().info(f'{self.get_name()} {avatar_type} not known, using dummy')


        self.create_subscription(TaggedString, inTopic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, outTopic, QoSProfile(depth=1))
    
    def _callback(self, msg):
        """Deal with translation"""
        if self._debug:
            self.get_logger().info(f"{self.get_name()} listening got {msg.text.data}")
        if self._debug:
            self.get_logger().info(f"{self.get_name()} checking cache for {msg.text.data}")
        response, response_time = self.local_cache.get(msg.text.data)
        
        if response is None:
            start_time = self.get_clock().now()
            prompt, response = self._llm.response(text=msg.text.data)
            time_taken = (self.get_clock().now() - start_time).nanoseconds / 1e9
            # Add to cache
            self.local_cache.put(msg.text.data, response, time_taken)
            if self._debug:
                self.get_logger().info(f"{self.get_name()} response: {response}, {len(response)}")
        else:
            # Cache hit so update the cache with the response time
            self.local_cache.put(msg.text.data, response, response_time)
            
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.audio_sequence_number
        tagged_string.text.data = str(response)
        self._publisher.publish(tagged_string)

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
