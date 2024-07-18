import py_trees
from avatar2_interfaces.msg import TaggedString
import string

class MetaHandler(py_trees.behaviour.Behaviour):
    """Handle meta requests to the avatar. Note that this is not intended to replace langchain or similar
       rather its goal is to deal with high-level meta operations (shutdown, sleep, wakeup, others) that
       would be difficult (impossible?) to do within the langchain operation"""

    def __init__(self, name):
        super(MetaHandler,self).__init__(name)
        self._last_audio_sequence = -1

        self._handlers = {
            'awake' : self._handler_awake,
            'sleep' : self._handler_sleep
        }

    # handlers must return 
    def _handler_awake(words):
        self.logger.debug(f"  {self.name} [awake handler called]")

    def _handler_sleep(words):
        self.logger.debug(f"  {self.name} [sleep handler called]")

    def setup(self, **kwargs):
        #kwargs['node'] is our ros node
        self._node = kwargs['node']
        self._node.get_logger().warning('meta handler is alive')
        self.logger.debug(f"  {self.name} [MetaHandler:setup()]")

    def initialise(self):
        self.logger.debug(f"  {self.name} [MetaHandler:initialise()]")
        self._blackboard = py_trees.blackboard.Client(name="Meta Handler")
        self._blackboard.register_key(key="/in_message", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.text", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.text.data", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/in_message.audio_sequence_number", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/avatar_name", access=py_trees.common.Access.READ)
        self._blackboard.register_key(key="/out_message", access=py_trees.common.Access.WRITE)

    def update(self):
        self.logger.debug(f"  {self.name} [MetaHandler:update()]")
        self._node.get_logger().warning(f'meta handler update text is {self._blackboard.in_message.text.data}')
        self._node.get_logger().warning(f'meta handler update number is {self._blackboard.in_message.audio_sequence_number}')

        sequence = self._blackboard.in_message.audio_sequence_number
        in_text = self._blackboard.in_message.text.data.lower()
        
        avatar_name = self._blackboard.avatar_name.lower()
        self._node.get_logger().warning(f'Avatars name is {avatar_name}')

        # if we have processed this sentence, ignore it
        if sequence == self._last_audio_sequence:
            return py_trees.common.Status.SUCCESS
        self._last_audio_sequence = sequence

        self._node.get_logger().warning(f'meta handler processing a new bit of text')

        in_text = in_text.translate(str.maketrans('', '', string.punctuation))
        self._node.get_logger().warning(f'text is now {in_text}')
        words = in_text.split()
        self._node.get_logger().warning(f'text is now {words}')
        if (len(words) > 1) and (words[0] == avatar_name):
            self._node.get_logger().warning(f'We should process this')
            try:
                self._handlers.get(words[1])(words)
                response = "Meta response to " + in_text
                tagged_string = TaggedString()
                tagged_string.header.stamp = self._node.get_clock().now().to_msg()
                tagged_string.audio_sequence_number = sequence
                tagged_string.text.data = str(response)
                self._blackboard.out_message = tagged_string


                return py_trees.common.Status.SUCCESS
            except:
                self._node.get_logger().warning(f'dont know {key}')
        else:
            user_input = TaggedString()
            user_input.header.stamp = self._node.get_clock().now().to_msg()
            user_input.audio_sequence_number = sequence
            user_input.text.data = str(in_text)
            
            self._blackboard.in_message = user_input
            self._node.get_logger().warning(f'passing on to LLM {in_text}')
            
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"  {self.name} [MetaHandler:terminate()][{self.status}->{new_status}]")
        
        
