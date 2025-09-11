import wave
from keras.models import load_model
import tensorflow as tf
from keras.layers import TFSMLayer
# from tf.keras.models import load_model
from .helper import pad_sequence_into_array
from .features import *
# from graph_formating import *
# import matplotlib.pyplot as plt
from collections import deque
import warnings
import numpy as np

# Method to get audio parameters
def get_audio(filename):
    wav = wave.open(filename, mode="rb")
    (nchannels, sampwidth, framerate, nframes, comptype, compname) = wav.getparams()
    content = wav.readframes(nframes)
    samples = np.fromstring(content, dtype=np.int16)
    return (nchannels, sampwidth, framerate, nframes, comptype, compname), samples

# Method to load the audio in the buffer. Also checks if framerate != 44100
def load_into_buffer(file_name, window_size = 0.5):
    RATE = 44100
    audio_data = get_audio(file_name)
    (nchannels, sampwidth, framerate, nframes, comptype, compname), sample_wav = audio_data

    if (framerate != RATE):
        warnings.warn("The audio framerate is assumed to be " + RATE + ". Recorded framerate is " + framerate)

    audio_length = nframes / framerate # length of wav in seconds
    # buffer_windows = nframes * len_wav # number of windows we need
    load_buffer = deque()

    # split audio into chunks of window_size and feed them into the buffer
    left = sample_wav[0::nchannels]
    # for t_start in range(0, int(len_wav), int(window_size)):

    t_start = 0
    while True:
        if t_start == 0: #first sample
            start = t_start
            end = t_start + window_size
        else:
            start = t_start
            end = t_start + window_size

        if audio_length < end:
            end = audio_length

        sample_left = left[int(start * framerate):int(end * framerate)]
        load_buffer.append(np.array(sample_left).astype('int16'))

        if audio_length == end: break
        else: t_start = end
    return load_buffer

# Method used to return results from model. Currently only works with audio only feature
# Can be amended to work with the multimodal model
def predict_on_model(file_name, model_path):
    WINDOW_SIZE = 0.5
    WINDOW_N = 10
    process_buffer = deque(maxlen=WINDOW_N)
    RATE = 44100
    file_len = 5
    CHUNK = int(RATE / 2)

    # set up model
    # model = load_model(model_path)
    # model = load_model(model_path, custom_objects={'TFSMLayer': TFSMLayer})
    model = tf.keras.models.Sequential([
                TFSMLayer(model_path, call_endpoint='serving_default')
            ])

    process_buffer = deque(maxlen=WINDOW_N)
    # create empty buffer for holding in file data
    feeder_buffer = deque()
    # load into buffer checks for framerate requirements
    # raises a warning if framerate != RATE
    feeder_buffer = load_into_buffer(file_name, WINDOW_SIZE)
    
    # if len(feeder_buffer) == 0: # if the feeder buffer is empty, read in a new audio file
    #     #print("loading to feeder")
    #     feeder_buffer = load_file_into_buffer(file_path, raw_audio, WINDOW_SIZE)

        # if multi_modal:
        #     # When we load in a new file, run deepspeech
        #     print("trigger deepspeech"); start_ds_time = time.time()
        #     text, sentiment = get_deepspeech_predictions(feeder_buffer, deepspeech_model_8)
        #     print("deepspeech runtime: " + str(time.time() - start_ds_time))

    # Step 2: Move 1 window of the Feeder Buffer into the Process Buffer
    process_buffer.append(feeder_buffer.popleft())

    data_int = []
    for sample in process_buffer:
        data_int += list(sample)

    data_int = np.array(data_int).astype('int16')


    # Calculate Audio Features
    st_features = calculate_features(data_int, RATE)
    st_features, _ = pad_sequence_into_array(st_features, maxlen=100)
    st_features = np.array([st_features.T])

    # Predict on model
    wav_test_results = model.predict(st_features)
    return wav_test_results

# Just a simple function to print graph of outputs
# Not used with ROS system
# def graph_output(wav_test_results):
#     cols = ['ang', 'exc', 'neu', 'sad', 'hap', 'fea', 'sur']
#     df_pred_wav = pd.DataFrame([np.zeros(7)], columns=cols)
#     fig, ax = plt.subplots(3)

#     # basic formatting for the axes
#     ax[0].set_title('Emotion Prediction')
#     ax[0].set_xlabel('Time')
#     ax[0].set_ylabel('Confidence')

#     ax[0] = plot_line_graph(ax[0], df_pred_wav)

#     predicted_values = pd.DataFrame({cols[0]: wav_test_results[0][0],
#                                      cols[1]: wav_test_results[0][1],
#                                      cols[2]: wav_test_results[0][2],
#                                      cols[3]: wav_test_results[0][3],
#                                      cols[4]: wav_test_results[0][4],
#                                      cols[5]: wav_test_results[0][5],
#                                      cols[6]: wav_test_results[0][6]
#                                      }, index=[1])

#     print(predicted_values.shape)
#     # pass previous values to filter function
#     # predicted_values = noise_filter(df_pred_wav.tail(1),
#     # predicted_values)

#     df_pred_wav = df_pred_wav.append(predicted_values,
#                                      ignore_index=True)

#     ax[0] = plot_line_graph(ax[0], df_pred_wav)

#     df_pred_wav.plot(kind='bar')
#     plt.ylabel('Emotion Prediction')
#     fig.canvas.draw_idle()
#     plt.show()

# Used to calculate features from audio input
# Taken from the works of Samarth Tripathi 
def calculate_features(frames, freq):
    window_sec = 0.1
    window_n = int(freq * window_sec)

    st_f = stFeatureExtraction(frames, freq, window_n, window_n / 2)

    if st_f.shape[1] > 2:
        i0 = 1
        i1 = st_f.shape[1] - 1
        if i1 - i0 < 1:
            i1 = i0 + 1

        deriv_st_f = np.zeros((st_f.shape[0], i1 - i0), dtype=float)
        for i in range(i0, i1):
            i_left = i - 1
            i_right = i + 1
            deriv_st_f[:st_f.shape[0], i - i0] = st_f[:, i]
        return deriv_st_f
    elif st_f.shape[1] == 2:
        deriv_st_f = np.zeros((st_f.shape[0], 1), dtype=float)
        deriv_st_f[:st_f.shape[0], 0] = st_f[:, 0]
        return deriv_st_f
    else:
        deriv_st_f = np.zeros((st_f.shape[0], 1), dtype=float)
        deriv_st_f[:st_f.shape[0], 0] = st_f[:, 0]
        return deriv_st_f

