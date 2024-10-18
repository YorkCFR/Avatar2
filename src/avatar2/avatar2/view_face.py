#
# A bit of debugging code to show what the face recognizer got back
#
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
import json
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from avatar2_interfaces.msg import SpeakerInfo
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

class ViewCamera(Node):
    def __init__(self):
        super().__init__('view_face')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('face', "/avatar2/speaker_info")

        self._face_topic = self.get_parameter('face').get_parameter_value().string_value
        self.create_subscription(SpeakerInfo, self._face_topic, self._face_callback, QoSProfile(depth=1))
        self._bridge = CvBridge()

    def _face_callback(self, msg):
        face = self._bridge.imgmsg_to_cv2(msg.face, "bgr8")
   
        height = face.shape[0]
        width = face.shape[1]
        first_name = "Unknown"
        last_name = "Unknown"
        role = "Unknown"
        try :
            data = json.loads(msg.info.data)
            first_name = data['first_name']
            last_name = data['last_name']
            role = data['role']
        except Exception as e:
            pass
        cv2.rectangle(face, (int(width*(msg.col-msg.width/2)), int(height*(msg.row-msg.height/2))), (int(width*(msg.col+msg.width/2)), int(height*(msg.row+msg.height/2))), (0,0,255), 2)
        cv2.putText(face, first_name + " " + last_name, (int(width*(msg.col-msg.width/2) - 10), int(height*(msg.row+msg.height/2)+25)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, 2)
        cv2.imshow('face', face)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ViewCamera()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

