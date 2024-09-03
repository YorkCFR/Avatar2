import py_trees
import py_trees_ros
import py_trees_ros.trees
import py_trees.console as console
import rclpy

from .MetaHandler import MetaHandler
from .AvatarHandler import AvatarHandler
from avatar2_interfaces.msg import TaggedString, SpeakerInfo



def controller_create_root() -> py_trees.behaviour.Behaviour:
    """Create the entire tree"""

    # High to low priority and restart on each tick
    root = py_trees.composites.Sequence(name = "Simple controller", memory = False)

    # Map interesting ros topics to the blackboard
    topics2bb = py_trees.composites.Sequence(name = "Topics2BB", memory = True)

    in_control_message = py_trees_ros.subscribers.ToBlackboard(name="in_control_message",
         qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
         topic_name="/avatar2/in_control_message", topic_type=TaggedString, blackboard_variables = {'in_control_message' : None})
    
    speaker_info = py_trees_ros.subscribers.ToBlackboard(name="speaker_info",
          qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
          topic_name="/avatar2/speaker_info", topic_type=SpeakerInfo, blackboard_variables = {'speaker_info' : None})

    # automatically publish interesting ros tops from the blackboard
    out_message = py_trees_ros.publishers.FromBlackboard(name="out_mesage",
        topic_name = "/avatar2/out_message",
        topic_type = TaggedString,
        qos_profile = py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable = "/out_message")
    
    in_message = py_trees_ros.publishers.FromBlackboard(name="in_message",
        topic_name = "/avatar2/in_message",
        topic_type = TaggedString,
        qos_profile = py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variable = "/in_message")
    
    # Deal with meta commands to the avatar
    meta = MetaHandler("meta handler")

    # High to low priority and restart on each tick
    bits = py_trees.composites.Selector(name = "Behaviour types", memory = False)

    # Deal with regular avatar commands
    avatar = AvatarHandler("avatar handler")

    # an always running behaviour
    idle = py_trees.behaviours.Running(name="Idle")

    # build the tree
    bits.add_child(meta)
    bits.add_child(avatar)
    topics2bb.add_child(in_control_message)
    topics2bb.add_child(speaker_info)
    root.add_child(topics2bb)
    root.add_child(bits)
    root.add_child(out_message)
    root.add_child(in_message)
    root.add_child(idle)

    return root

        

def main(args=None):
    rclpy.init(args=args)
    root = controller_create_root()
    tree = py_trees_ros.trees.BehaviourTree(root = root, unicode_tree_debug = True)
    provided = py_trees.blackboard.Client(name="Provided")
    provided.register_key(key='avatar_name', access=py_trees.common.Access.WRITE)
    provided.avatar_name = 'Mary'
    provided.register_key(key='avatar_name_pattern', access=py_trees.common.Access.WRITE)
    provided.avatar_name_pattern = '^mary|merry$|^merry$'
    print(provided)

    try:
        tree.setup(node_name = "basic_control", timeout = 15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + f"failed tosetup the tree, aborting [{e}]" + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror(console.red + "tree setup interrupted" + console.reset)
        tree.shutdown()

    print("Tree setup complete. Starting tick tock...")
    tree.tick_tock(period_ms = 1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
