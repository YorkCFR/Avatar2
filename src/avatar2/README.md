Low Level Launch Files
* avatar_audio_launch.py - just launches sound to text/text to sound
* avatar_face_detect.launch.py - launches face recognition nodes
* avatar_microphone.launch.py - launches sound capture node
* ros_avatar.launch.py - launch the ros avatar 

* all_control.launch.py - launches everything including control tool
* all_launch.py - launchces everything except control tool

Installation Notes

This requires 
* libasound-dev (sudo apt-get install libasound-dev)
* python3-pyaudio (sudo apt-get install python3-pyaudio)
* pyaudio (pip3 install pyaudio)
* speechrecognition (pip3 install speechrecognition)
* Keras (pip3 install keras)
* Tensorflow (python3 -m pip install tensorflow[and-cuda])

## Updated TTS Requirements (2025)

### New TTS System (Piper Neural TTS)
* piper-tts==1.3.0 (pip install piper-tts)
* pydub (pip install pydub) - Audio processing
* simpleaudio (pip install simpleaudio) - Audio playback

### Voice Models Required
Voice models must be downloaded separately (not included in repository due to size):

**Required Models Directory:** `/Avatar2/tts_models/`

**Models to Download:**
* **Primary Voice**: `en_US-lessac-high.onnx` + `en_US-lessac-high.onnx.json`
* **Alternative**: `en_US-ljspeech-high.onnx` + `en_US-ljspeech-high.onnx.json`  
* **Backup**: `en_US-amy-medium.onnx` + `en_US-amy-medium.onnx.json`

**Download Instructions:**
```bash
# Create TTS models directory
mkdir -p /home/walleed/Avatar2/tts_models

# Download Lessac voice (primary, high quality)
wget -O /home/walleed/Avatar2/tts_models/en_US-lessac-high.onnx \
  "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/high/en_US-lessac-high.onnx"
wget -O /home/walleed/Avatar2/tts_models/en_US-lessac-high.onnx.json \
  "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/high/en_US-lessac-high.onnx.json"

# Download LJSpeech voice (alternative, high quality)
wget -O /home/walleed/Avatar2/tts_models/en_US-ljspeech-high.onnx \
  "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ljspeech/high/en_US-ljspeech-high.onnx"
wget -O /home/walleed/Avatar2/tts_models/en_US-ljspeech-high.onnx.json \
  "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/ljspeech/high/en_US-ljspeech-high.onnx.json"

# Download Amy voice (backup, medium quality, smaller file)
wget -O /home/walleed/Avatar2/tts_models/en_US-amy-medium.onnx \
  "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/amy/medium/en_US-amy-medium.onnx"
wget -O /home/walleed/Avatar2/tts_models/en_US-amy-medium.onnx.json \
  "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/amy/medium/en_US-amy-medium.onnx.json"
```

**Note:** These models are large files (60-108 MB each) and are excluded from the repository to comply with GitHub file size limits.

### TTS Quality Settings (Optimized)
```bash
--length-scale 1.0        # Natural speech speed
--noise-scale 0.333       # Reduce robotic artifacts
--noise-w-scale 0.333     # Smoother pronunciation
--sentence-silence 0.2    # Natural pauses
```

### Deprecated/Removed Dependencies
* ~~TTS (Coqui)~~ - Heavy dependency conflicts with Tensorflow versions
* ~~festival~~ - Low quality output
* ~~espeak-ng~~ - Basic quality, not needed
* ~~gTTS~~ - Google cloud dependency

### Environment Notes
* CUDA 12.2 + CuDNN 8907 for GPU acceleration

## Quick Setup Guide

### 1. Install System Dependencies
```bash
sudo apt-get install libasound-dev python3-pyaudio
```

### 2. Install Python Packages
```bash
pip3 install pyaudio speechrecognition keras piper-tts pydub simpleaudio
python3 -m pip install tensorflow[and-cuda]
```

### 3. Download Voice Models
Voice models must be downloaded separately due to their large size:

```bash
# Create TTS models directory
mkdir -p /home/walleed/Avatar2/tts_models

### 4. Build and Run
```bash
ros2 launch avatar2 avatar_debug_full_pipeline.launch.py debug:=true
```
