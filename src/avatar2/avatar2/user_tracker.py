import os
import sys
import json
import rclpy
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import SpeakerInfo, TaggedString

class ConversationTrackerNode(Node):
    def __init__(self, config_file = 'config.json'):
        super().__init__('conversation_tracker_node')
        self.get_logger().info(f'{self.get_name()} node created')
        self.declare_parameter('config_file', config_file)
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
        except:
            self.get_logger().error(f'{self.get_name()} unable to open config_file {config_file}')
            sys.exit(1)
        
        self._debug = True
        try:
            # self._debug = config['debug']
            self._face_topic = config['face_topic']
            self._tracker_topic = config['tracker_topic']
            self._conversation_timeout = config['conversation_timeout']
            self._new_person_threshold = config['new_person_threshold']
        except:
            self.get_logger().error(f'{self.get_name()} unable to get params from {config_file}')
            sys.exit(1)
            
        self._current_speaker = None
        self._face_reappeared = False
        self._detection_start_time = None
        self._last_detection_time = None
        self._detection_duration = None
        self._msg_id = 0
        if self._debug:
            self.get_logger().info(f'{self.get_name()} initialized with conversation timeout: {self._conversation_timeout} seconds')

        self._no_face_timer = self.create_timer(1.0, self._no_face_callback)  # Check every second            
        self._subscriber = self.create_subscription(SpeakerInfo, self._face_topic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, self._tracker_topic, QoSProfile(depth=1))
        
    def _no_face_callback(self):
        # Check if the face detection is lost
        current_time = self.get_clock().now()
        convo_info = TaggedString()
        convo_info.header.stamp = current_time.to_msg()
        convo_info.audio_sequence_number = self._msg_id+1
        if self._last_detection_time is not None:
            time_diff = (current_time - self._last_detection_time).nanoseconds / 1e9
            if time_diff > self._new_person_threshold:
                # Face detection lost
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} face detection lost')
                self._current_speaker = None
                self._detection_start_time = None
                self._detection_duration = None
                self._face_reappeared = False
                convo_info.text.data = 'No face detected for ' + str(round(time_diff, 2)) + ' seconds.'
            elif time_diff > self._conversation_timeout:
                # Conversation timeout
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} conversation timeout')
                self._detection_start_time = None
                self._face_reappeared = True
                since_last_seen = (current_time - self._last_detection_time).nanoseconds / 1e9
                convo_info.text.data = 'Conversation timeout. No face detected for ' + str(round(time_diff, 2)) + ' seconds.'
            else:
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} no face detected for {time_diff} seconds')
                convo_info.text.data = 'No face detected for ' + str(round(time_diff, 2)) + ' seconds.'
            self._publisher.publish(convo_info)
        else:
            # First time we have no face detected
            convo_info.text.data = 'No face detected. Waiting for face detection.'
            self._publisher.publish(convo_info)
            
    def _callback(self, msg):
        # Got a face detection message, check if it's the same speaker
        speaker_info = json.loads(msg.info.data)
        speaker_id = speaker_info['ID']
        current_time = self.get_clock().now()
        convo_info = TaggedString()
        convo_info.header.stamp = current_time.to_msg()
        convo_info.audio_sequence_number = msg.seq # match the sequence number of face detection
        self._msg_id = msg.seq
        
        if speaker_id == -1:
            # Unknown person detected
            # Check if it is a new person or the same person
            if self._current_speaker is None:
                # New person detected
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} detected new person')
                self._current_speaker = 'Unknown'
                self._detection_start_time = current_time
                self._last_detection_time = current_time
                self._detection_duration = 0
                self._face_reappeared = False
                # Publish new person detected
                convo_info.text.data = 'New unknown person detected.'
            else:
                # Conversation with the same person
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} detected same person')
                self._face_reappeared = True
                self._last_detection_time = current_time
                if self._detection_start_time is None:
                    if self._debug:
                        self.get_logger().info(f'{self.get_name()} conversation_start_time is None. New conversation.')
                    # New conversation with the same person
                    self._detection_start_time = current_time
                    self._detection_duration = 0
                else:
                    # Continuing conversation with the same person
                    if self._debug:
                        self.get_logger().info(f'{self.get_name()} continuing conversation with the same person')
                    self._detection_duration += (current_time - self._detection_start_time).nanoseconds / 1e9
                # Publish same person detected
                convo_info.text.data = 'Continuing conversation. Conversation duration: ' + str(round(self._detection_duration, 2)) + ' seconds.'
            self._publisher.publish(convo_info)
        else:
            # Known person detected
            if self._current_speaker is None:
                # First time detection of known person
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} detected new known person')
                self._current_speaker = speaker_info['first_name'] + ' ' + speaker_info['last_name']
                self._detection_start_time = current_time
                self._last_detection_time = current_time
                self._detection_duration = 0
                self._face_reappeared = False
                convo_info.text.data = 'New known person detected: ' + self._current_speaker
            else:
                # Conversation with the same person
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} detected same known person')
                self._last_detection_time = current_time
                self._face_reappeared = True
                self._detection_duration += (current_time - self._detection_start_time).nanoseconds / 1e9
                convo_info.text.data = 'Continuing conversation with ' + self._current_speaker + '. Conversation duration: ' + str(round(self._detection_duration, 2)) + ' seconds.'
            self._publisher.publish(convo_info)
                
def main(args=None):
    rclpy.init(args=args)
    node = ConversationTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    
if __name__ == '__main__':
    main()