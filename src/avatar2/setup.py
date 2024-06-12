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
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walleed',
    maintainer_email='walleed@todo.todo',
    description='TODO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_capture = avatar2.audio_input:main',
            'sound_play = avatar2.audio_input_wav:main',
            'sound_dump = avatar2.audio_dump:main',
            'sound_to_text = avatar2.audio_to_text:main',
            'avatar_camera = avatar2.opencv_camera:main',
            'sentiment_analysis = avatar2.sentiment_analysis:main',
            'head_info = avatar2.yolo_head:main',
            'head_detect = avatar2.recognizer:main',
            'view_head_info = avatar2.view_face:main',
            'text_to_sound = avatar2.text_to_audio:main',
            'play_text = avatar2.play_text:main',
            'play_text_syncd = avatar2.play_text_syncd:main',
            'llm_engine = avatar2.llm_engine:main',
            'ros_avatar = avatar2.ros_avatar:main',
        ],
    },
)
