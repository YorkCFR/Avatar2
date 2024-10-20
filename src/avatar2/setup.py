import os
from glob import glob
from setuptools import setup

package_name = 'avatar2'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('/classification_model/*')),
        (os.path.join('share', package_name), glob('/classifications/models/*')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walleed',
    maintainer_email='walleed@yorku.ca',
    description='A ROS-based avatar for HRI',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avatar_camera = avatar2.opencv_camera:main',               # capture video
            'avatar_camera_view = avatar2.view_camera:main',            # display raw video (debug)
            'head_detect = avatar2.recognizer:main',                    # do head detection
            'view_head_info = avatar2.view_face:main',                  # display head detection output (debug)
            'user_tracker = avatar2.user_tracker:main',                 # do user tracking
            'user_monitor = avatar2.user_monitor:main',                 # report results of user monitoring (debug)
            'sound_capture = avatar2.audio_input:main',			# capture audio from a microphone
            'sound_play = avatar2.audio_input_wav:main',		# play a wav file (debug)
            'sound_dump = avatar2.audio_dump:main',			# dump audio file (debug)
            'sound_to_text = avatar2.audio_to_text:main',		# convert audio to text
            'play_text = avatar2.play_text:main',			# convert text to wav
            'text_to_sound = avatar2.text_to_audio:main',
            'play_text_syncd = avatar2.play_text_syncd:main',
            'play_audio_syncd = avatar2.play_audio_syncd:main',
            'llm_engine = avatar2.llm_engine:main',
            'ros_avatar = avatar2.ros_avatar:main',
            'llm_only = avatar2.llm_only:main',
            'sentiment_analysis = avatar2.sentiment_analysis:main',
        ],
    },
)
