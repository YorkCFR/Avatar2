#
# Convert text to audio using LOCAL TTS piper engine.
# This version just saves the wav file and send along the address of the wav
# file to the ROS world. The files are all created in a directory that is
# created when run, so file collision is reduced. The recipient is responsible for deleting
# the message when they get it. As an added bonus we get to see what wasn't spoken.
#
#
# piper is installed through pip as pip3 install piper-tts
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from avatar2_interfaces.msg import TaggedString
from rclpy.qos import QoSProfile
from pydub import AudioSegment
from pathlib import Path
import tempfile
import os
import io
import subprocess



class Text2WavNode(Node):
    def __init__(self):
        super().__init__('text_to_wav_node')
        self.declare_parameter('audio_dir', '/avatar2/out_raw_audio_dir')
        self._audio_dir = self.get_parameter('audio_dir').get_parameter_value().string_value
        self._audio_dir = self._audio_dir + "/" + "avatar-" + str(os.getpid())

        self.declare_parameter('audio_raw', '/avatar2/out_raw_audio_name')
        self._audio_raw = self.get_parameter('audio_raw').get_parameter_value().string_value

        self.declare_parameter('message', '/avatar2/out_message')
        message = self.get_parameter('message').get_parameter_value().string_value

        self.declare_parameter('debug', True)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('voice_model', '/home/walleed/Avatar2/tts_models/en_US-lessac-high.onnx')
        self._voice_model = self.get_parameter('voice_model').get_parameter_value().string_value
        self.get_logger().info(f'Voice model: {self._voice_model}')

        self.create_subscription(TaggedString, message, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(TaggedString, self._audio_raw, QoSProfile(depth=1))

        path = Path(self._audio_dir)
        if path.exists():
            self.get_logger().error(f'The Path directory {self._audio_dir} exists. Will not overwrite')
            exit()
        try:
            os.mkdir(self._audio_dir)
            os.chmod(self._audio_dir, 0o777)
        except Exception as e:
            self.get_logger().error(f'Unable to create/chmod {self._audio_dir} {e}')
            exit()

        self._utteranceNo = 0
        self.get_logger().info(f'Running with debug {self._debug}, using Piper TTS. Output directory is {self._audio_dir}')

    def _callback(self, data):
        if self._debug:
            self.get_logger().info(f'{self.get_name()} about to convert |{data.text.data}|')
        
        try:
            self.get_logger().warn(f'string is {data.text.data}')
            wav_container = self._generate_local_audio(data.text.data)
            self.get_logger().info(f'{self.get_name()} output is in {wav_container}')
                
            # Publish the tagged string
            msg = TaggedString()
            current_time = self.get_clock().now()
            msg.header.stamp = current_time.to_msg()
            msg.audio_sequence_number = data.audio_sequence_number
            msg.header.frame_id = ""
            msg.text = String()
            msg.text.data = str(wav_container)
            self._publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'TTS generation failed: {e}')

    def _generate_local_audio(self, text):
        """Generate high-quality audio using Piper TTS"""
        try:
            # Create temporary WAV file
            temp_wav_file = self._audio_dir + "/" + "utterance-" + str(self._utteranceNo) + ".wav"
            self._utteranceNo = self._utteranceNo + 1
            
            # Run Piper TTS command with optimized quality settings
            cmd = [
                'piper',
                '--model', self._voice_model,
                '--output-file', temp_wav_file,
                '--length-scale', '1.0',
                '--noise-scale', '0.333',
                '--noise-w-scale', '0.333',
                '--sentence-silence', '0.2'
            ]
            self.get_logger().warn(f'TTS generation : {cmd}')
            
            # Execute Piper with text input
            process = subprocess.run(
                cmd,
                input=text,
                text=True,
                capture_output=True,
                check=True
            )
            
            return temp_wav_file
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Piper TTS command failed: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Piper TTS error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = Text2WavNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
