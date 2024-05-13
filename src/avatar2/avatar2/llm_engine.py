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

            self._llm = LLMLangChain(model=model, prompt=prompt, vectorstore=vectorstore, format=format)
            self._local_cache = LocalCache()
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

            self._llm = LLMWithFaces(model=model, prompt=prompt, vectorstore=vectorstore, format=format, node=self)
            self._local_cache = LocalCache()
        else:
            self.get_logger().info(f'{self.get_name()} {avatar_type} not known, using dummy')
            self._llm = LLMDummy()

        self.create_subscription(TaggedString, inTopic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, outTopic, QoSProfile(depth=1))

    def _get_response(self, query):
        prompt, response = self._llm.response(text=query)
        
        return response
    
    def _evict_cache(self):
        self.get_logger().info(f"{self.get_name()} The current cache contains {self._local_cache._cache.keys()}")
        oldest = ((key for key in self._local_cache._cache if key not in self._local_cache._permanent_entries)).__next__()
        self.get_logger().info(f"{self.get_name()} Evicting {oldest}")
        if oldest:
            self._local_cache._cache.pop(oldest)
    
    def _process_query(self, query):
        query_normalized = query.lower().strip().translate(str.maketrans('', '', string.punctuation))
        self.get_logger().info(f"{self.get_name()} Normalized query: {query_normalized}")
        
        if query_normalized in self._local_cache._cache:
            self.get_logger().info(f"{self.get_name()} Cache hit for {query_normalized}")
            self._local_cache._cache.move_to_end(query_normalized)
            return self._local_cache._cache[query_normalized]
        
        self.get_logger().info(f"{self.get_name()} Cache miss for {query_normalized}")
        response = self._get_response(query)
        if len(self._local_cache._cache) >= self._local_cache._max_size:
            self.get_logger().info(f"{self.get_name()} Cache full, purging oldest entry")
            self._evict_cache()
        self._local_cache._cache[query_normalized] = response
        
        return response
    
    def _callback(self, msg):
        """Deal with translation"""
        self.get_logger().info(f"{self.get_name()} listening got {msg.text.data}")

        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.audio_sequence_number
        response = self._process_query(msg.text.data)
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
