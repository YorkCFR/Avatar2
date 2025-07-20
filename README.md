# Avatar2
The updated avatar. This is a generic README.md that describes the in-flux state of the avatar as of October 2024.
 
There have been some major changes since the version in the summer. Perhaps the most critical of these is that the data from the face recognition system now includes much more information.

The avatar uses ROS2 for message passing. The passing system is set up so that all messages are sent/received. So far this has not been an issue

## Basic form
You can run the entire system (assuming you have all the right hardware on one machine) using

- ros2 launch avatar2 all.launch.py root:=<where the scenario folder lives> Scenario:=<scenario to run>

This will run the most basic version of the avatar. There are a number of other parameters that you can/should set here. But this is key to make it work

We have a number of scenarios for the avatar. The details for a given scenario should be in the scenario folder. Within a given scenario there should be a faces folder which includes the pkl and json representation of the known people of the scenario.
- hearing_clinic The hearing clinic
- presentation An avatar for presentations about the project
- regimental_museum An avatar for the museum
- robot_interface An avatar for a ROS robot

## Lower level tools
To run/test various lower level pieces, the following launch files are quite useful

- avatar_debug_video.launch.py - This launches the camera and a camera viewer
- avatar_debug_recognizer.launch.py - This launches the camera and the face recognizer
- avatar_microphone.launch.py - This launches the microphone monitoring code
- ros2 run avatar2 sound_capture --ros-args -p  debug:=True - this will let you check that your audio is capturing things
- ros2 launch avatar2 avatar_debug_play_text.launch.py  - this will play text to audio with a simple avatar display
- avatar_audio.launch.py - This launches the speech to text and text to speech code



The debris folder is a place for things that were important once, but are no longer part of the main branch of the avatar.


The following python libraries need to be installed

- face_recognition


