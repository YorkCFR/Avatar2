import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'avatar2_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenkin',
    maintainer_email='jenkin@yorku.ca',
    description='High level avatar control',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'basic_control = avatar2_control.basic_control:main',
        ],
    },
)
