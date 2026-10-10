from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'avatar2_llm_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'websockets', 'llama-cpp-python'],
    zip_safe=True,
    maintainer='Walleed',
    maintainer_email='walleedk@yorku.ca',
    description='Local LLM client for Avatar2 - generates "say" responses to "heard" events',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'avatar2_llm_bridge_node = avatar2_llm_bridge.avatar2_llm_bridge_node:main',
        ],
    },
)
