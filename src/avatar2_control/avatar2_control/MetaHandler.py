import py_trees
from avatar2_interfaces.msg import TaggedString
import string
import re
import json

class MetaHandler(py_trees.behaviour.Behaviour):
    """Handle meta requests to the avatar. Note that this is not intended to replace langchain or similar
       rather its goal is to deal with high-level meta operations (shutdown, sleep, wakeup, others) that
       would be difficult (impossible?) to do within the langchain operation"""

    def __init__(self, name):
        super(MetaHandler,self).__init__(name)
        self._last_audio_sequence = -1

        self._handlers = {
            'awake' : self._handler_awake,
            'wakeup' : self._handler_awake,
            'wake' : self._handler_awake,
            'sleep' : self._handler_sleep,
            'gotosleep' : self._handler_sleep,
            'sleepnow' : self._handler_sleep,
            'shutdown' : self._handler_sleep,
            'die' : self._handler_sleep,
            'kill' : self._handler_sleep,
        }
        
        self._sleeping = False

    def _say_something_directly(self, s):
        say_this = TaggedString()
        say_this.header.stamp = self._node.get_clock().now().to_msg()
        sequence = self._blackboard.in_control_message.audio_sequence_number
        say_this.audio_sequence_number = sequence
        say_this.text.data = str(s)
        self._blackboard.out_message = say_this
        
    # handlers must return 
    def _handler_awake(self, words, speaker_name=None):
        """ Handler for the awake command """
        self._node.get_logger().warning(f"  {self.name} [awake handler called], sleeping is {self._sleeping}")
        if not self._sleeping:
            # if the avatar is already awake, then we should say something
            if speaker_name is not None:
                self._say_something_directly(f"But {speaker_name}, I am not sleeping now.")
            else:
                self._say_something_directly("But I am not sleeping now.")
        else:
            # if the avatar is sleeping, then we should wake it up
            self._sleeping = False
            self._say_something_directly("Ok. I am awake now.")
            self.get_logger().warning(f"  {self.name} [sleep handler called], sleeping is {self._sleeping}")

    def _handler_sleep(self, words, speaker_name=None):
        """ Handler for the sleep command """
        self._node.get_logger().warning(f"  {self.name} [sleep handler called], sleeping is {self._sleeping}")
        if not self._sleeping:
            # if the avatar is already awake, then we should put it to sleep
            self._sleeping = True
            if speaker_name is not None:
                self._say_something_directly(f"Ok {speaker_name}, I am going to sleep. Tell me to wake up if you want me to listen")
            else:
                self._say_something_directly("Ok. I am going to sleep. Tell me to wake up if you want me to listen")
        else :
            # if the avatar is already sleeping, then we do nothing
            self.get_logger().warning(f"  {self.name} [sleep handler called], sleeping is {self._sleeping}")
        

    def setup(self, **kwargs):
        #kwargs['node'] is our ros node
        self._node = kwargs['node']
        self._node.get_logger().warning('meta handler is alive')
        self.logger.debug(f"  {self.name} [MetaHandler:setup()]")

    def initialise(self):
        self.logger.debug(f"  {self.name} [MetaHandler:initialise()]")
        self._blackboard = py_trees.blackboard.Client(name="Meta Handler")
        self._blackboard.register_key(key="/in_control_message", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_control_message.text", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_control_message.text.data", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/avatar_name", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/avatar_name_pattern", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/out_message", access=py_trees.common.Access.WRITE)
        self._blackboard.register_key(key="/in_message", access=py_trees.common.Access.WRITE)
        self._blackboard.register_key(key="/speaker_info", access=py_trees.common.Access.READ)

    def update(self):
        self.logger.debug(f"  {self.name} [MetaHandler:update()]")
        self._node.get_logger().warning(f'meta handler update text is {self._blackboard.in_control_message.text.data}')
        self._node.get_logger().warning(f'meta handler update number is {self._blackboard.in_control_message.audio_sequence_number}')

        speaker_info = json.loads(self._blackboard.speaker_info.info.data)
        self._node.get_logger().warning(f'speaker info is {speaker_info}')
        sequence = self._blackboard.in_control_message.audio_sequence_number
        in_text = self._blackboard.in_control_message.text.data.lower()
        speaker_first_name = speaker_info['first_name']
        speaker_last_name = speaker_info['last_name']
        speaker_role = speaker_info['role']
        
        avatar_name = self._blackboard.avatar_name.lower()
        self._node.get_logger().warning(f'Avatars name is {avatar_name}')
        avatar_name_pattern = self._blackboard.avatar_name_pattern
        self._node.get_logger().warning(f'Avatars name pattern is {avatar_name_pattern}')

        # if the avatar is sleeping, ignore all commands and return "I am sleeping"
        if self._sleeping:
            self._node.get_logger().warning(f'avatar is sleeping')
            # Check if the speaker is trying to wake the avatar
            if self.check_awake_command(in_text, avatar_name_pattern):
                self._handler_awake([], speaker_name=speaker_first_name)
            # Try to see if we have speaker info to say something
            if speaker_first_name is not None:
                self._say_something_directly(f"Sorry {speaker_first_name}, I am sleeping right now.")
            else:
                self._say_something_directly("Sorry, I am sleeping right now.")
            return py_trees.common.Status.SUCCESS
        else:
            self._node.get_logger().warning(f'avatar is awake')
        
        # if we have processed this sentence, ignore it
        if sequence == self._last_audio_sequence:
            return py_trees.common.Status.SUCCESS
        self._last_audio_sequence = sequence

        self._node.get_logger().warning(f'meta handler processing a new bit of text')

        in_text = in_text.translate(str.maketrans('', '', string.punctuation))
        self._node.get_logger().warning(f'text is now {in_text}')
        words = in_text.split()
        self._node.get_logger().warning(f'text is now {words}')
        # if the first word is the avatar name, then we should process this
        if (len(words) > 1) and (re.match(avatar_name_pattern, words[0]) is not None):
            key = "".join(words[1:])
            self._node.get_logger().warning(f'We should process this {key}')
            try:
                self._handlers[key](words, speaker_name=speaker_first_name)
                self._node.get_logger().warning(f'{key} has been processed. Not sending to the llm')                
                return py_trees.common.Status.SUCCESS
            except KeyError:
                self._node.get_logger().warning(f'dont know {key}')
        
        user_input = TaggedString()
        user_input.header.stamp = self._node.get_clock().now().to_msg()
        user_input.audio_sequence_number = sequence
        user_input.text.data = str(in_text)
        
        self._blackboard.in_message = user_input
        self._node.get_logger().warning(f'passing on to LLM {in_text}')
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [MetaHandler:terminate()][{self.status}->{new_status}]")
        
    def check_awake_command(self, in_text, avatar_name_pattern):
        in_text = in_text.translate(str.maketrans('', '', string.punctuation))
        words = in_text.split()
        if (len(words) > 1) and (re.match(avatar_name_pattern, words[0]) is not None):
            key = "".join(words[1:])
            if key == 'awake' or key == 'wake' or key == 'wakeup':
                return True
