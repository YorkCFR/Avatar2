import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import Audio
from avatar2_interfaces.srv import Listen
import simpleaudio as sa
import cv2
import time
import math

class ROSAvatar(Node):
    def __init__(self):
        super().__init__('ros_avatar')
        self.declare_parameter('topic', '/avatar2/out_raw_audio')
        self._topic = self.get_parameter('topic').get_parameter_value().string_value

        #debug param    
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} node created, debug is {self._debug}')


        self.declare_parameter('listen', '/avatar2/listen')
        self._listen = self.get_parameter('listen').get_parameter_value().string_value
        self._cli = self.create_client(Listen, self._listen)
        while not self._cli.wait_for_service(timeout_sec=1.0):
            if self._debug:
                self.get_logger().info(f'{self.get_name()} waiting for service {self._listen}')
        self._req =  Listen.Request()


        self.declare_parameter('imagery', './ros_avatar')
        imagery = self.get_parameter('imagery').get_parameter_value().string_value

        self.declare_parameter('nframes', 64)
        self._nframes = self.get_parameter('nframes').get_parameter_value().integer_value

        self._im_talk = [None] * self._nframes
        self._im_sleep = [None] * self._nframes
        for i in range(self._nframes):
            self._im_talk[i] = cv2.imread(imagery +  f'/talk/frame_{i:>02}_delay-0.03s.jpg')
            self._im_sleep[i] = cv2.imread(imagery + f'/sleep/frame_{i:>02}_delay-0.03s.jpg')
        self._frameid = 0
        cv2.namedWindow('Avatar')
        cv2.imshow('Avatar', self._im_sleep[self._frameid])
        cv2.waitKey(1)

        self._play = None
        self._req.listen = True
        self._cli.call_async(self._req)  # ignore return value

        self._subscriber = self.create_subscription(Audio, self._topic, self._callback, 1)
        self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        if self._play is not None:
            if not self._play.is_playing():
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} sound is finished')
                self._play = None
                self._req.listen = True
                self._cli.call_async(self._req)  # ignore return value

        self._frameid = (self._frameid + 1) % self._nframes
        if self._play is None:
            cv2.imshow('Avatar', self._im_sleep[self._frameid])
            if self._debug:
                self.get_logger().info(f'{self.get_name()} Not playing')
        else:
            t = time.time() - self._playtime
            alpha = 0.5 * (1 - math.cos(t * 2 * math.pi / 2))
            mix = cv2.addWeighted(self._im_talk[self._frameid], alpha, self._im_sleep[self._frameid], 1-alpha, 0)
            cv2.imshow('Avatar', mix)
            if self._debug:
                self.get_logger().info(f'{self.get_name()} playing')
        cv2.waitKey(1)


    def _callback(self, data):
        if self._play is not None:
            if self._play.is_playing():
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} currently playing, ignoring new sound')
                return  
        audio = bytes.fromhex(data.audio)
        if data.format == 'WAV_1_44100':
            sound = sa.WaveObject(audio, num_channels=1, bytes_per_sample=2, sample_rate=44100)
        elif data.format == 'WAV_1_22050':
            sound = sa.WaveObject(audio, num_channels=1, bytes_per_sample=2, sample_rate=22050)
        else:
            if self._debug:
                self.get_logger().info(f'{self.get_name()} got an unknown sound format {data.format}')
            return
        self._req.listen = False
        self._cli.call_async(self._req)  # ignore return value
        self._play = sound.play()
        self._playtime = time.time()
        


        

def main(args=None):
    rclpy.init(args=args)
    node = ROSAvatar()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
