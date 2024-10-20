import os, sys
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import SpeakerInfo, TaggedString

class UserMonitorNode(Node):
    def __init__(self):
        super().__init__("user_monitor_node")
        self.get_logger().info(f'{self.get_name()} created')

        self._subscriber = self.create_subscription(TaggedString, "/avatar2/user_info", self._callback, QoSProfile(depth=1))

        self._state = "Startup"
        self._nlooking = 0
        self._ndisrupt = 0

    def _callback(self, msg):
        j = json.loads(msg.text.data)
#        self.get_logger().info(f'{self.get_name()} got message {j}')

        if self._state == "Startup":
            if j['state'] == "Looking":
                self.get_logger().info(f"{self.get_name()} started up looking at {j['first_name']} {j['last_name']} duration {j['time']} {j['proxemics']}")
                self._watching = j
                self._state = "Continuing"
            elif j['state'] == "Idle":
                self.get_logger().info(f"Started up in Idle")
                self._watching = None
                self._state = 'Idle'
            elif j['state'] == "Starting":
                self.get_logger().info(f"Started up starting at {j['first_name']} {j['last_name']} duration {j['time']} {j['proxemics']}")
                self._watching = j
                self._state = "Continuing"
            elif j['state'] == "Continuing":
                self.get_logger().info(f"Started up disrupted at {j['first_name']} {j['last_name']} duration {j['time']} {j['proxemics']}")
                self._watching = j
                self._state = "Continuing"
            elif j['state'] == "Disrupted":
                self.get_logger().info(f"Started up disrupted at {j['first_name']} {j['last_name']} duration {j['time']} {j['proxemics']}")
                self._watching = j
                self._state = "Continuing"
            elif j['state'] == "Terminated":
                self.get_logger().info(f"Started up disrupted at {j['first_name']} {j['last_name']} duration {j['time']}")
                self._watching = None
                self._state = "Idle"
        else:
            if j['state'] == "Idle":
                pass
            elif j['state'] == "Starting":
                self.get_logger().info(f"Starting to look at {j['first_name']} {j['last_name']} duration {j['time']} {j['proxemics']}")
                self._watching = j
                self._state = "Continuing"
                self._ndisrupt = 0
                self._nlooking = 0
            elif j['state'] == "Continuing":
                if j['proxemics'] != self._watching['proxemics']:
                    self.get_logger().info(f"Looking at {j['first_name']} {j['last_name']} duration {j['time']} {j['proxemics']} {self._ndisrupt} {self._nlooking}")
                    self._watching = j
            elif j['state'] == "Disrupted":
                self._ndisrupt = self._ndisrupt + 1
                pass
            elif j['state'] == "Looking":
                self._nlooking = self._nlooking + 1
                pass
            elif j['state'] == "Terminated":
                self.get_logger().info(f"Finished looking at {j['first_name']} {j['last_name']} duration {j['time']}")
            else:
                self.get_logger().info(f"{self.get_name()} strange {j}")

def main(args=None):
    rclpy.init(args=args)
    node = UserMonitorNode()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

