#
# Convert text to audio
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from avatar2_interfaces.msg import Audio, TaggedString
from rclpy.qos import QoSProfile
from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer
import tempfile
import os



class Text2AudioNode(Node):
    def __init__(self):
        super().__init__('text_to_audio_node')
        self.declare_parameter('audio', '/avatar2/out_raw_audio')
        audio = self.get_parameter('audio').get_parameter_value().string_value
        self.declare_parameter('message', '/avatar2/out_message')
        message = self.get_parameter('message').get_parameter_value().string_value
        self.declare_parameter('device', 'cuda') # or cpu
        cuda = self.get_parameter('device').get_parameter_value().string_value == 'cuda'
        self.declare_parameter('site_path', "~/.local/lib/python3.10/site-packages/TTS/.models.json") # where local TTS models live
        site_path = self.get_parameter('site_path').get_parameter_value().string_value
        self.declare_parameter('model', "tts_models/en/ljspeech/tacotron2-DDC") # voice model to use
        model = self.get_parameter('model').get_parameter_value().string_value
        self.declare_parameter('debug', False)
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value

        # do all that is needed to fire up TTS
        model_manager = ModelManager(site_path)
        model_path, config_path, model_item = model_manager.download_model(model)
        voc_path, voc_config_path, _ = model_manager.download_model(model_item["default_vocoder"])
        self._syn = Synthesizer(tts_checkpoint=model_path, tts_config_path=config_path, 
                                vocoder_checkpoint=voc_path, vocoder_config=voc_config_path, use_cuda=cuda)

        self.create_subscription(TaggedString, message, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(Audio, audio, QoSProfile(depth=1))
        self.get_logger().info(f'Running with debug {self._debug}')

    def _callback(self, data):
        if self._debug:
            self.get_logger().info(f'{self.get_name()} about to say |{data.text.data}|')
        out = self._syn.tts(data.text.data)
        fp = tempfile.NamedTemporaryFile(delete=False,suffix=".wav")
        fp.close()
        if self._debug:
            self.get_logger().info(f'{self.get_name()} saving to {fp.name}')
        self._syn.save_wav(out, fp.name)
        with open(fp.name, "rb") as f:
            audio_data = f.read()

            msg = Audio()
            msg.audio = audio_data.hex()
            msg.format = "WAV_1_22050"  # known magically
            msg.text = String()
            msg.text.data = data.text.data
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.seq = data.audio_sequence_number
            self._publisher.publish(msg)
        os.remove(fp.name)

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
