# Provide a generic wrapper for simple langchain LLM interface with a promp and a vectorstore and faces.
from langchain.prompts.prompt import PromptTemplate
from langchain_community.llms import LlamaCpp
from .llm import LLM
from rclpy.parameter import Parameter
from rclpy.duration import Duration
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from avatar2_interfaces.msg import SpeakerInfo
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from avatar2_interfaces.msg import TaggedString

import pickle

class LLMWithFaces(LLM):
    _NO_FACE_RESET = 10.0
    def __init__(self, model, prompt, format, vectorstore, node, max_vectors=2, n_ctx=2048, temperature=0, verbose=False, n_gpu_layers=-1, debug = False):
        with open(vectorstore, "rb") as f:
            self.vectorstore = pickle.load(f)
        self._llm = LlamaCpp(model_path=model, temperature=temperature, n_ctx=n_ctx, verbose=verbose,  n_gpu_layers=n_gpu_layers)
        self._prompt = prompt
        self._format = format
        self._max_vectors = max_vectors
        self._node = node
        self._debug = debug
        self._node.get_logger().info(f'{self._node.get_name()} LLMWithFaces alive')

        self._node.declare_parameter('face', "/avatar2/speaker_info")
        self._face_topic = self._node.get_parameter('face').get_parameter_value().string_value
        self._last_face = None
        self._last_audio = None

        self._node.create_subscription(SpeakerInfo, self._face_topic, self._face_callback, QoSProfile(depth=1))
        self._bridge = CvBridge()

    def _face_callback(self, msg):
        now = self._node.get_clock().now()
        if self._last_face is None:
            welcome = True
        else:
            dur = now - self._last_face
            if dur > Duration(seconds=LLMWithFaces._NO_FACE_RESET):
                welcome = True
            else:
                welcome = False
        self._last_face = self._node.get_clock().now()
        if self._debug:
            self._node.get_logger().info(f'{self._node.get_name()} recovered face {msg.row} {msg.col} {welcome}')
        face = self._bridge.imgmsg_to_cv2(msg.face, "bgr8")

#        cv2.imshow('face', face)
#        cv2.waitKey(3)
        return

        if welcome:
            prompt = self._prompt + "\n"
            text = "Someone has just arrived at the museum. Welcome them to the museum and ask them if you can be of any help.\n"
            prompt = prompt + self._format.format(question=text)
    
            self._node.get_logger().info(f'{self._node.get_name()} want to ask {prompt}')
            resp = self._llm(prompt)
            tagged_string = TaggedString()
            tagged_string.header.stamp = self._node.get_clock().now().to_msg()
            tagged_string.audio_sequence_number = 0
            tagged_string.text.data = str(resp)
            self._node.get_logger().info(f'{self._node.get_name()} want to say {resp}')
            self._node._publisher.publish(tagged_string)


    def response(self, text):
        self._last_audio = self._node.get_clock().now()
        docs = self.vectorstore.as_retriever(search_kwargs={"k" : self._max_vectors}).get_relevant_documents(query=text)

        prompt = self._prompt
        for doc in docs:
            prompt = prompt + "\n" + doc.page_content
        prompt = prompt + self._format.format(question=text)
    
        resp = self._llm(prompt)
        return prompt, resp
