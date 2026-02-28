#
# Convert text to audio using LOCAL TTS engines
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from avatar2_interfaces.msg import Audio, TaggedString
from rclpy.qos import QoSProfile
from pydub import AudioSegment
import tempfile
import os
import io
import subprocess



class Text2AudioNode(Node):
    def __init__(self):
        super().__init__('text_to_audio_node')
        self.declare_parameter('audio', '/avatar2/out_raw_audio')
        audio = self.get_parameter('audio').get_parameter_value().string_value
        self.declare_parameter('message', '/avatar2/out_message')
        message = self.get_parameter('message').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('voice_model', '/home/walleed/Avatar2/tts_models/en_US-lessac-high.onnx')
        self._voice_model = self.get_parameter('voice_model').get_parameter_value().string_value
        self.get_logger().info(f'Voice model: {self._voice_model}')

        self.create_subscription(TaggedString, message, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(Audio, audio, QoSProfile(depth=1))
        self.get_logger().info(f'Running with debug {self._debug}, using Piper TTS')

    def _callback(self, data):
        if self._debug:
            self.get_logger().info(f'{self.get_name()} about to say |{data.text.data}|')
        
        try:
            wav_data = self._generate_local_audio(data.text.data)
                
            if wav_data:
                # Publish the WAV audio data
                msg = Audio()
                msg.audio = wav_data.hex()
                msg.format = "WAV_1_22050"  # Match ros_avatar supported format
                msg.text = String()
                msg.text.data = data.text.data
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.seq = data.audio_sequence_number
                self._publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'TTS generation failed: {e}')

    def _generate_local_audio(self, text):
        """Generate high-quality audio using Piper TTS"""
        try:
            # Create temporary WAV file
            temp_wav_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
            temp_wav_file.close()
            
            # Run Piper TTS command with optimized quality settings
            cmd = [
                'piper',
                '--model', self._voice_model,
                '--output-file', temp_wav_file.name,
                '--length-scale', '1.0',
                '--noise-scale', '0.333',
                '--noise-w-scale', '0.333',
                '--sentence-silence', '0.2'
            ]
            
            # Execute Piper with text input
            process = subprocess.run(
                cmd,
                input=text,
                text=True,
                capture_output=True,
                check=True
            )
            
            # Load and convert audio to target format (mono, 22050Hz, 16-bit)
            audio = AudioSegment.from_wav(temp_wav_file.name)
            audio = audio.set_channels(1).set_frame_rate(22050).set_sample_width(2)
            
            # Export to WAV format in memory
            wav_buffer = io.BytesIO()
            audio.export(wav_buffer, format="wav")
            wav_data = wav_buffer.getvalue()
            
            # Clean up
            os.remove(temp_wav_file.name)
            return wav_data
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Piper TTS command failed: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Piper TTS error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = Text2AudioNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
