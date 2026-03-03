#
# Given face recognition, track the user we are talking to
#
# This now returns a json structure (not a string)
#
# state ::= "Looking" | "Idle" | "Starting" | "Continuing" | "Disrupted" | "Terminated"
# time ::= string representation of a time in seconds
# id ::= integer (-1 if unknown person) (only valid for Starting)
# last_name ::= last name as a string (only valid for Starting)
# first_name ::= last name as a string (only valid for Starting)
# last_name ::= last name as a string (only valid for Starting)
# role ::= role of the person (only valid for starting)
# proxemics ::= "intimate" | "personal" | "social" |  "public" 
#
# This now uses the more standard configuration structure
# 
#
import os
import sys
import json
import rclpy
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile
from avatar2_interfaces.msg import SpeakerInfo, TaggedString

class ConversationTrackerNode(Node):
    def __init__(self):
        super().__init__('conversation_tracker_node')
        self.get_logger().info(f'node created')

        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('face_topic', '/avatar2/face_topic')
        self._face_topic = self.get_parameter('face_topic').get_parameter_value().string_value
        self.declare_parameter('tracker_topic', '/avatar2/tracker_topic')
        self._tracker_topic = self.get_parameter('tracker_topic').get_parameter_value().string_value
        self.declare_parameter('conversation_timeout', 1.0)
        self._conversation_timeout = self.get_parameter('conversation_timeout').get_parameter_value().double_value
        self.declare_parameter('new_person_threshold', 1.0)
        self._new_person_threshold = self.get_parameter('new_person_threshold').get_parameter_value().double_value
        self.declare_parameter('intimate', 200.0)
        self._intimate = self.get_parameter('intimate').get_parameter_value().double_value
        self.declare_parameter('personal', 100.0)
        self._personal = self.get_parameter('personal').get_parameter_value().double_value
        self.declare_parameter('social', 50.0)
        self._social = self.get_parameter('social').get_parameter_value().double_value
            
        self._current_speaker = None
        self._detection_start_time = 0
        self._last_detection_time = 0
        self._detection_duration = None

        self._msg_id = 0
        self.get_logger().info(f'initialized with conversation timeout: {self._conversation_timeout} seconds')

        self._no_face_timer = self.create_timer(1.0, self._no_face_callback)  # Check every second            
        self._subscriber = self.create_subscription(SpeakerInfo, self._face_topic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, self._tracker_topic, QoSProfile(depth=1))
        
    def _no_face_callback(self):
        """ Routinely check to see if a face has been seen"""
        current_time = self.get_clock().now()
        convo_info = TaggedString()
        message = {}
        convo_info.header.stamp = current_time.to_msg()
        convo_info.audio_sequence_number = self._msg_id+1

        if self._current_speaker is not None: # we are currently talking to someone 
            time_diff = (current_time - self._last_detection_time).nanoseconds / 1e9
            if time_diff > self._conversation_timeout:
                # Face detection lost
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} face detection lost')

                message['state'] = 'Terminated'
                message['first_name'] = self._current_speaker['first_name']
                message['last_name'] = self._current_speaker['last_name']
                message['role']= self._current_speaker['role']
                message['ID'] = self._current_speaker['ID']
                message['time'] = str(round(self._detection_duration, 2))

                self._current_speaker = None
                self._detection_start_time = 0.0
                self._detection_duration = 0.0
                self._face_reappeared = False
            else:
                if self._debug:
                    self.get_logger().info(f'{self.get_name()} no face detected for {time_diff} seconds')
                message['state'] = "Looking"
                message['first_name'] = self._current_speaker['first_name']
                message['last_name'] = self._current_speaker['last_name']
                message['role']= self._current_speaker['role']
                message['ID'] = self._current_speaker['ID']
                message['time'] = str(round(time_diff, 2))
            convo_info.text.data = json.dumps(message)
            self._publisher.publish(convo_info)
        else:
            # not talking to anyone
            message['state'] = "Idle"
            convo_info.text.data = json.dumps(message)
            self._publisher.publish(convo_info)
            

    def _callback(self, msg):
        """ Got a face detection message. Update conversation status"""
        speaker_info = json.loads(msg.info.data)
        current_time = self.get_clock().now()
        convo_info = TaggedString()
        convo_info.header.stamp = current_time.to_msg()
        convo_info.audio_sequence_number = msg.seq # match the sequence number of face detection
        self._msg_id = msg.seq
        message = {}
        area = msg.width * msg.height
        message['area'] = area
        message['target'] =[msg.row, msg.col]
        if area >= self._intimate:
            proxemics = "intimate"
        elif area >= self._personal:
            proxemics = "personal"
        elif area >= self._social:
            proxemics = "social"
        else:
            proxemics = "public"
        
        if self._current_speaker is None: # We have no curent speaker to consider
            if self._debug:
                self.get_logger().info(f"{self.get_name()} starting conversation with new person {speaker_info['ID']}")
            self._current_speaker = speaker_info
            self._detection_start_time = current_time
            self._last_detection_time = current_time

            message['state'] = 'Starting'
            message['first_name'] = speaker_info['first_name']
            message['last_name'] = speaker_info['last_name']
            message['role']= speaker_info['role']
            message['ID'] = speaker_info['ID']
            message['time'] = "0.00"
            message['proxemics'] = proxemics
            convo_info.text.data = json.dumps(message)
        elif self._current_speaker['ID'] == speaker_info['ID']: # detected the person we are talking too (all unknown are the same!)
            if self._debug:
                self.get_logger().info(f"{self.get_name()} detected same known person {speaker_info['ID']}")
            self._last_detection_time = current_time
            self._detection_duration = (current_time - self._detection_start_time).nanoseconds / 1e9

            message['state'] = 'Continuing'
            message['first_name'] = speaker_info['first_name']
            message['last_name'] = speaker_info['last_name']
            message['role']= speaker_info['role']
            message['ID'] = speaker_info['ID']
            message['time'] = str(round(self._detection_duration, 2))
            message['proxemics'] = proxemics
            convo_info.text.data = json.dumps(message)
        else: # detected someone else
            if self._debug:
                self.get_logger().info(f"{self.get_name()} detected different person old {self._current_speaker['ID']} new {speaker_info['ID']}")
            if (current_time - self._detection_start_time).nanoseconds / 1e9 > self._new_person_threshold: # time to forget this conversation
                message['state'] = 'Terminated'
                message['first_name'] = self._current_speaker['first_name']
                message['last_name'] = self._current_speaker['last_name']
                message['role']= self._current_speaker['role']
                message['ID'] = self._current_speaker['ID']
                message['time'] = str(round(self._detection_duration, 2))
                convo_info.text.data = json.dumps(message)
                self._current_speaker = None
            else: # ignore this disruption
                self._detection_duration = (current_time - self._detection_start_time).nanoseconds / 1e9

                message['state'] = 'Disrupted'
                message['first_name'] = self._current_speaker['first_name']
                message['last_name'] = self._current_speaker['last_name']
                message['role']= self._current_speaker['role']
                message['ID'] = self._current_speaker['ID']
                message['disrupter_ID'] = self._current_speaker['ID']
                message['time'] = str(round(self._detection_duration, 2))
                convo_info.text.data = json.dumps(message)
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
