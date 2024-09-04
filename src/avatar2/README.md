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
