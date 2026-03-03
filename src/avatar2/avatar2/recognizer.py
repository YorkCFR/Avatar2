#
# Do face recognition, and publish face information as detected
#
# This now ignores the scenario file information, and evertything comes
# from the config file structure
#
import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import math
import json
import os
from pathlib import Path
from cv_bridge import CvBridge
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from avatar2_interfaces.msg import SpeakerInfo
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from .FaceRecognizer import FaceRecognizer


class Recognizer(Node):
    def __init__(self):
        super().__init__('recognizer_node')
        self.get_logger().info(f'{self.get_name()} node created')
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('camera_topic', '/avatar2/image_raw')
        self._camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.declare_parameter('face_topic', '/avatar2/speaker_info')
        self._face_topic = self.get_parameter('face_topic').get_parameter_value().string_value
        self.declare_parameter('known_faces', 'faces_path')
        self._known_faces = self.get_parameter('known_faces').get_parameter_value().string_value

        encodings = Path(os.path.join(self._known_faces, "faces.pkl"))
        database = Path(os.path.join(self._known_faces, "faces.json"))

        self._msg_id = 0
        if self._debug:
            self.get_logger().info(f'debug is {self._debug}')
            self.get_logger().info(f'camera_topic is {self._camera_topic}')
            self.get_logger().info(f'face_topic is {self._face_topic}')
            self.get_logger().info(f'known_faces is {self._known_faces}')

    
        self._bridge = CvBridge()

        # subs
        self._sub = self.create_subscription(Image, self._camera_topic, self._camera_callback, 1) 

        # pubs
        self._publisher = self.create_publisher(SpeakerInfo, self._face_topic, QoSProfile(depth=1))

        self._face_recognizer = FaceRecognizer(encodings=encodings, database=database, debug=self._debug)
        self._face_recognizer.load_encodings()
        
    
    def _camera_callback(self, data):
        """Deal with camera message"""
        img = self._bridge.imgmsg_to_cv2(data)
        bb, name, middle_row, middle_col = self._face_recognizer.recognize_faces(img)

        if bb is not None:
            sub = img[bb[0]:bb[2], bb[3]:bb[1],:]
            if self._debug:
                cv2.imshow('face', sub)
                cv2.waitKey(3)


            sp = SpeakerInfo()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.seq = self._msg_id
            self._msg_id = self._msg_id + 1
            sp.row = float(middle_row) / float(img.shape[0])
            sp.col = float(middle_col) / float(img.shape[1])
            sp.width = float(sub.shape[1]) / float(img.shape[1])
            sp.height = float(sub.shape[0]) / float(img.shape[0])
            sp.face = self._bridge.cv2_to_imgmsg(img, "bgr8")
            sp.info = String()
            sp.info.data = json.dumps(name)
            self._publisher.publish(sp)
    
def main(args=None):
    rclpy.init(args=args)
    node = Recognizer()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
