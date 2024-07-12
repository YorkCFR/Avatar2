import sys
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class OpenCVCamera(Node):

    def __init__(self, rostopic='/mycamera/image_raw', port=2):
        super().__init__('avatar_camera')

        self.declare_parameter('topic', rostopic)
        rostopic = self.get_parameter('topic').get_parameter_value().string_value

        self.declare_parameter('port', str(port))
        port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f'{self.get_name()} publishing from camera {port} on {rostopic}')

        try:
            self._camera =  cv2.VideoCapture(port, cv2.CAP_V4L2)
        except Exception as e:
            self.get_logger().error(f'{self.get_name()} Unable to open camera {port}')
            sys.exit(1)

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, rostopic, 1)

    def stream(self):
        try:
            while True:
                _, frame = self._camera.read()
                image_message = self._bridge.cv2_to_imgmsg(frame, "bgr8")

                self._publisher.publish(image_message)

                rclpy.spin_once(self, timeout_sec=0)
        except Exception as e:
            self.get_logger().error(f'{self.get_name()} capture error {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCamera()
    try:
        node.stream()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
