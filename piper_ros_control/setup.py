from setuptools import setup
import os
from glob import glob
import sys

package_name = 'piper_ros_control'

python_version = f'{sys.version_info.major}.{sys.version_info.minor}'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes',  package_name + '.demos'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('piper_ros_control/launch/*.launch.py')),
        # (os.path.join('share', package_name, 'demos'),
        #     glob('piper_ros_control/demos/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CodeHesed',
    maintainer_email='code.hesed@gmail.com',
    description='ROS2 Humble control package for Piper robot arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'piper_single_custom_ctrl = piper_ros_control.nodes.piper_ctrl_single_custom_node:main',
            'keyboard_control = piper_ros_control.demos.keyboard_control:main'
        ],
    },
)