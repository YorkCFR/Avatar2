# Avatar2
The updated avatar. This is a generic README.md that describes the in-flux state of the avatar as of June 2026
 
There have been some major changes since the version that was targeted at the Hearing Clinic. 

## Lower level tools
To run/test various lower level pieces, the following launch files are quite useful

- avatar_debug_video.launch.py - This launches the camera and a camera viewer
- avatar_debug_recognizer.launch.py - This launches the camera and the face recognizer
- avatar_microphone.launch.py - This launches the microphone monitoring code
- ros2 run avatar2 sound_capture --ros-args -p  debug:=True - this will let you check that your audio is capturing things
- ros2 launch avatar2 avatar_debug_play_text.launch.py  - this will play text to audio with a simple avatar display
- avatar_audio.launch.py - This launches the speech to text and text to speech code


