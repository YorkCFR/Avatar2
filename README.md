# Avatar2
The updated avatar. This is a generic README.md that describes the in-flux state of the avatar as of July 2024.

The avatar uses ROS2 for message passing. The passing system is set up so that all messages are sent/received. So far this has not been an issue

## Basic form
You can run the entire system (assuming you have all the right hardware on one machine) using

- ros2 launch avatar2 all.launch.py root:=<where the scenario folder lives> Scenario:=<scenario to run>

This will run the most basic version of the avatar. There are a number of other parameters that you can/should set here. But this is key to make it work

We have a number of scenarios for the avatar. The details for a given scenario should be in the scenario folder. Within a given scenario there should be a faces folder which includes the pkl and json representation of the known people of the scenario.
- hearing_clinic The hearing clinic
- presentation An avatar for presentations about the project
- regimental_museum An avatar for the museum

## Lower level tools
To run/start various lower level pieces, the following launch files are quite useful

- avatar_video.launch.py - This launches the camera and the head tracking/identification system
- avatar_microphone.launch.py - This launches the microphone monitoring code
- avata_audio.launch.py - This launches the speech to text and text to speech code


The debris folder is a place for things that were important once, but are no longer part of the main branch of the avatar.



