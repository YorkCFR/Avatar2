import os
import rclpy
import json
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import TaggedString, SpeakerInfo
from .llm import LLM
from .llm_dummy import LLMDummy
from .llm_langchain import LLMLangChain
from .llm_withfaces import LLMWithFaces
from .llm_local_cache import LocalCache

class LLMEngine(Node):
    def __init__(self, root_dir='', scenario=''):
        super().__init__('llm_engine_node')
        config_file = os.path.join(root_dir, scenario, 'config.json')
        self.declare_parameter('config_file', config_file)
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        time_counter = 1
        self._role = ''

        self.get_logger().info(f'{self.get_name()} loading config form {config_file}')
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
        except:
            raise Exception(f"Could not open {config_file}")

        # self._debug = config.get('debug', True)
        self._debug = True
        if self._debug:
            self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')
       
        avatar_type = config.get('avatar', 'faces')
        if self._debug:
            self.get_logger().info(f'{self.get_name()} Firing up an avatar of type {avatar_type}')

        self.create_subscription(SpeakerInfo, '/avatar2/speaker_info', self._role_callback, QoSProfile(depth=1))
        self.create_subscription(SpeakerInfo, '/avatar2/speaker_info', self._role_callback, QoSProfile(depth=1))
        self.create_subscription(SpeakerInfo, '/avatar2/speaker_info', self._role_callback, QoSProfile(depth=1))
        self.create_subscription(SpeakerInfo, '/avatar2/speaker_info', self._role_callback, QoSProfile(depth=1))

        if avatar_type == 'dummy':
            self._llm = LLMDummy()
        elif avatar_type == 'langchain' or avatar_type == 'faces':
            try:
                root = config['root']
                vectorstore = config['vectorstore']
                model = config['model']
                prompt = config['prompt']
                cache = config['cache']
                format = config['format']
                outTopic = config['out_topic']
                inTopic = config['in_topic']
                avatar_type = config['avatar']
                scenario = config['scenario']
                log_dir = config['log_dir']

            except:
                self.get_logger().error(f'{self.get_name()} missing data in config')
                sys.exit(1)

            model = os.path.join(root, scenario, model)
            prompt = os.path.join(root, scenario, prompt)
            vectorstore = os.path.join(root, scenario, vectorstore)
            format = os.path.join(root, scenario, format)
            cache = os.path.join(root, scenario, cache)
            log_dir = os.path.join(root, scenario, log_dir)


            if avatar_type == 'langchain':
                self.get_logger().info(f'{self.get_name()} Loading LLM model {model} vectorestore {vectorstore}')
                self._llm = LLMLangChain(model=model, prompt=prompt, vectorstore=vectorstore, format=format)
                self.get_logger().info(f'{self.get_name()} LLM model Loaded {model}')
                self.local_cache = LocalCache(node=self, cache_file=cache, log_dir=log_dir, root=root)
            if avatar_type == 'faces':
                self.get_logger().info(f'{self.get_name()} Loading LLM model {model} vectorestore {vectorstore}')
                self._llm = LLMWithFaces(model=model, prompt=prompt, vectorstore=vectorstore, format=format, node=self)
                self.get_logger().info(f'{self.get_name()} LLM model Loaded {model}')
            self.local_cache = LocalCache(node=self, cache_file=cache, log_dir=log_dir, root=root)

        else:
            if self._debug:
                self.get_logger().info(f'{self.get_name()} {avatar_type} not known, using dummy')
            self._llm = LLMDummy()

        self.get_logger().info(f"{self.get_name()} LLM {model} is active!")
        self.create_subscription(TaggedString, inTopic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, outTopic, QoSProfile(depth=1))

    def _callback(self, msg):
        """Deal with translation"""
        if self._debug:
            self.get_logger().info(f"{self.get_name()} listening got {msg.text.data}")
        if self._debug:
            self.get_logger().info(f"{self.get_name()} checking cache for {msg.text.data}")        
        response, response_time = self.local_cache.get(msg.text.data)
        llm_input = msg.text.data

        if response is None:
            if (self._role == ''):
                self.get_logger().info(f"No role found")
            else:
                llm_input = llm_input + f"User has role {self._role}. Respond accordingly"
            start_time = self.get_clock().now()
            prompt, response = self._llm.response(text=llm_input)
            time_taken = (self.get_clock().now() - start_time).nanoseconds / 1e9
            # Add to cache
            self.local_cache.put(msg.text.data, response, time_taken)
            if self._debug:
                self.get_logger().info(f"{self.get_name()} response: {response}, {len(response)}")
        else:
            # Cache hit so update the cache with the response time
            self.local_cache._update_cache(msg.text.data, response, response_time)
            
        tagged_string = TaggedString()
        tagged_string.header.stamp = self.get_clock().now().to_msg()
        tagged_string.audio_sequence_number = msg.audio_sequence_number
        tagged_string.text.data = str(response)
        self._publisher.publish(tagged_string)

    def _role_callback(self, msg_role):
        """Deal with Roles from Face Recognizer"""
        data = msg_role.info.data
        self.get_logger().info(f"Got the data {data}")
        if self._debug:
            self.get_logger().info(f"{self.get_name()} listening got {data}")
        try:
            info = json.loads(data)
            self._role = info['role']
            # role = info.get('role', 'roomate')
            self.get_logger().info(f"{self.get_name()} got role {self._role}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")
            return

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
