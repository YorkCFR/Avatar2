from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'avatar2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walleed',
    maintainer_email='walleed.e.khan@gmail.com',
    description='Interface betweem ROS-based system and AVA',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ava_bridge = avatar2_bridge.ava_bridge:main',
        ],
    },
)
