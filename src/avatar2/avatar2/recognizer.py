import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import math
import json
from pathlib import Path
from cv_bridge import CvBridge
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from avatar2_interfaces.msg import SpeakerInfo
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from .FaceRecognizer import FaceRecognizer
from .FaceRecognizer import FaceRecognizer


class Recognizer(Node):
    def __init__(self, config_file = 'config.json'):
        super().__init__('recognizer_node')
        self.get_logger().info(f'{self.get_name()} node created')
        self.declare_parameter('config_file', config_file)
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
        except:
            self.get_logger().error(f'{self.get_name()} unable to open config_file {config_file}')
            sys.exit(1)
            
        try:
            self._debug = config['debug']
            self._face_topic = config['face_topic']   
            self._camera_topic = config['camera_topic']
            root = config['root']
            scenario = config['scenario']
        
        except:
            self.get_logger().error(f'{self.get_name()} unable to get params from {config_file}')
            sys.exit(1)

        self._msg_id = 0
        if self._debug:
            self.get_logger().info(f'{self.get_name()} root is {root}')

        encodings = Path(os.path.join(root, scenario, 'faces', "faces.pkl"))
        database = Path(os.path.join(root, scenario, 'faces', "faces.json"))
    
        self._bridge = CvBridge()

        # subs
        self._sub = self.create_subscription(Image, self._camera_topic, self._camera_callback, 1) 

        # pubs
        self._publisher = self.create_publisher(SpeakerInfo, self._face_topic, QoSProfile(depth=1))

        self._face_recognizer = FaceRecognizer(encodings=encodings, database=database)
        self._face_recognizer.load_encodings()
        
    
    def _camera_callback(self, data):
        img = self._bridge.imgmsg_to_cv2(data)

        bb, name, middle_row, middle_col = self._face_recognizer.recognize_faces(img)
        #if self._debug:
            #self.get_logger().info(f'{self.get_name()} bounding_box {bb} name is {name}')

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

    

    


