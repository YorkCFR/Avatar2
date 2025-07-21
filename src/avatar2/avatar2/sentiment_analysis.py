import rclpy
from rclpy.node import Node
from avatar2_interfaces.msg import Audio, SentimentAnalysis
from rclpy.qos import QoSProfile
from .sentimentModule import Sentiment

class SentimentAnalysisNode(Node):
    def __init__(self):
        super().__init__('sentiment_analysis_node')
        self.declare_parameter('topic', '/avatar2/in_raw_audio')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('sentiment', '/avatar2/sentiment_analysis')
        sentiment = self.get_parameter('sentiment').get_parameter_value().string_value
        self.declare_parameter('sentiments', ['anger', 'excited', 'neutral', 'sad', 'happy', 'fear', 'surprised'])
        self._sentiments = self.get_parameter('sentiments').get_parameter_value().string_array_value
        self.declare_parameter('classification_model', 'classification_model')
        classification_model = self.get_parameter('sentiment').get_parameter_value().string_value

        self._sentimentAnalysis = Sentiment(model=classification_model)

        self.create_subscription(Audio, topic, self._callback, QoSProfile(depth=1))
        self._publisher = self.create_publisher(SentimentAnalysis, sentiment, QoSProfile(depth=1))

    def _callback(self, data):
        self.get_logger().info(f"Sentiment analysis for {data.seq}")
        score = self._sentimentAnalysis.analyze(data.audio, format=data.format)
        # sentiments = ['anger', 'excited', 'neutral', 'sad', 'happy', 'fear', 'surprised']
        # self.get_logger().info(f"Sentiment score type {type(score)}")
        max_index = score.argmax()
        max_confidence = score[max_index]
        max_feeling = self._sentiments[max_index]

        sentiment_analysis = SentimentAnalysis()
        sentiment_analysis.header.stamp = self.get_clock().now().to_msg()
        sentiment_analysis.audio_sequence_number = data.seq
        sentiment_analysis.result = score.tolist()
        sentiment_analysis.confidence = max_confidence.item()
        sentiment_analysis.emotion = max_feeling
        self._publisher.publish(sentiment_analysis)

def main(args=None):
    rclpy.init(args=args)
    node = SentimentAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
