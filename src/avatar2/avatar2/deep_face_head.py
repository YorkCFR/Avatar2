import os
import sys
import rclpy
import cv2
import datetime
import numpy as np
import pandas as pd
import math
from ultralytics import YOLO
from ultralytics.engine.results import Results, Keypoints
from cv_bridge import CvBridge
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from avatar2_interfaces.msg import SpeakerInfo
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


class YOLO_Pose(Node):
    _BODY_PARTS = ["NOSE", "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR", "LEFT_SHOULDER", "RIGHT_SHOULDER",
                   "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE",
                   "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE"]
    def __init__(self):
        super().__init__('pose_node')

        # debug param
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')

        # params
        self._model_file = os.path.join(get_package_share_directory('avatar2'), 'yolov8n-pose.pt') 
        self.declare_parameter("model", self._model_file) 
        model = self.get_parameter("model").get_parameter_value().string_value

        self.declare_parameter("device", "cpu")
        self._device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self._threshold = self.get_parameter("threshold").get_parameter_value().double_value

        self._msg_id = 0
        self.declare_parameter('topic', '/avatar2/speaker_info')
        self._topic = self.get_parameter('topic').get_parameter_value().string_value


        self.declare_parameter("camera_topic", "/mycamera/image_raw")
        self._camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        
        self._move_flag = False
        self._bridge = CvBridge()
        self._model = YOLO(model)
        self._model.fuse()

        # subs
        self._sub = self.create_subscription(Image, self._camera_topic, self._camera_callback, 1) 

        # pubs
        self._publisher = self.create_publisher(SpeakerInfo, self._topic, QoSProfile(depth=1))
        
    def parse_keypoints(self, results: Results):

        keypoints_list = []

        for points in results.keypoints:        
            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):
                if conf >= self._threshold:
                    keypoints_list.append([kp_id, p[0], p[1], conf])

        return keypoints_list
    
    def _camera_callback(self, data):
#        self.get_logger().info(f'{self.get_name()} camera callback')
        img = self._bridge.imgmsg_to_cv2(data)
        results = self._model.predict(
                source = img,
                verbose = False,
                stream = False,
                conf = self._threshold,
                device = self._device
        )

        if len(results) != 1:
            if self._debug:
                self.get_logger().info(f'{self.get_name()}  Nothing to see here or too much {len(results)}')
            return
            
        results = results[0].cpu()
        if len(results.boxes.data) == 0:
#            self.get_logger().info(f'{self.get_name()}  boxes are too small')
            return

        
#        self.get_logger().info(f'{self.get_name()}  {results.boxes.data[0]}')
        if results.keypoints:
            keypoints = self.parse_keypoints(results)
            left_shoulder = None
            right_shoulder = None
            nose = None
            if len(keypoints) > 0:
                for i in range(len(keypoints)):
                    if YOLO_Pose._BODY_PARTS[keypoints[i][0]] == 'LEFT_SHOULDER':
                        left_shoulder = keypoints[i]
                    if YOLO_Pose._BODY_PARTS[keypoints[i][0]] == 'RIGHT_SHOULDER':
                        right_shoulder = keypoints[i]
                    if YOLO_Pose._BODY_PARTS[keypoints[i][0]] == 'NOSE':
                        nose = keypoints[i]

#                self.get_logger().info(f'{self.get_name()}  left shoulder {left_shoulder}')
#                self.get_logger().info(f'{self.get_name()}  right shoulder {right_shoulder}')
#                self.get_logger().info(f'{self.get_name()}  nose {nose}')

                if (left_shoulder is not None) and (right_shoulder is not None) and (nose is not None):

                    lsu = int(left_shoulder[1])
                    lsv = int(left_shoulder[2])
                    rsu = int(right_shoulder[1])
                    rsv = int(right_shoulder[2])
                    pu1 = int(results.boxes.data[0][0])
                    pv1 = int(results.boxes.data[0][1])
                    pu2 = int(results.boxes.data[0][2])
                    pv2 = int(results.boxes.data[0][3])
                    v2 = int((lsv+rsv)/2)

                
#                    self.get_logger().info(f'{self.get_name()}  {pv1}:{v2} {rsu}:{lsu}')

                    if lsu > rsu:
                        sub = img[pv1:v2, rsu:lsu,:]
                        sp = SpeakerInfo()
                        sp.header.stamp = self.get_clock().now().to_msg()
                        sp.seq = self._msg_id
                        self._msg_id = self._msg_id + 1
                        sp.row = float(nose[2]) / 480 
                        sp.col = float(nose[1]) / 640 
                        sp.face = self._bridge.cv2_to_imgmsg(sub, "bgr8")
                        self._publisher.publish(sp)


#                        self.get_logger().info(f'{self.get_name()}  {sub.shape}')
                        # Visualize results on frame        
#                        annotated_frame = results[0].plot()
#                        cv2.imshow('Results', annotated_frame)
#                        cv2.imshow('sub', sub)
#                        cv2.waitKey(1)
    
def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Pose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


