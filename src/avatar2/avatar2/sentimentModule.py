import io
from .features import *
#import ament_index_python.packages
from .functions import predict_on_model

class Sentiment():
    def __init__(self, model='/home/walleed/Avatar2/src/avatar2/classification_model/model/saved_model.pb'):
        self._model_path = model

    def analyze(self, data, format='WAV_1_220050'):
        wav_file = io.BytesIO(bytes.fromhex(data))
        with wav_file as source:
            result = predict_on_model(source, self._model_path)
            # print(f"Model prediction result: {result}")
            
            # Handle different result formats
            if isinstance(result, dict):
                # Handle dictionary output (e.g., {'output_1': array(...)})
                if 'output_1' in result:
                    score = result['output_1'][0]  # Take first prediction from the array
                else:
                    # Take the first value from the dictionary
                    score = list(result.values())[0][0]
            elif hasattr(result, 'shape'):
                # Handle numpy array results
                if len(result.shape) > 1 and result.shape[0] > 0:
                    score = result[0]  # Take first prediction if batch
                elif len(result.shape) == 1:
                    score = result
                else:
                    score = result
            elif hasattr(result, '__len__') and len(result) > 0:
                score = result[0] if isinstance(result[0], (list, tuple)) else result
            else:
                score = result
        return score
