import io
from .features import *
import ament_index_python.packages
from .functions import predict_on_model

class Sentiment():
    def __init__(self, model='/home/jenkin/Documents/avatar/Avatar2/src/avatar2/classification_model/model'):
        self._model_path = model

    def analyze(self, data, format='WAV_1_220050'):
        wav_file = io.BytesIO(bytes.fromhex(data))
        with wav_file as source:
            score = predict_on_model(source, self._model_path)[0]
        return score
