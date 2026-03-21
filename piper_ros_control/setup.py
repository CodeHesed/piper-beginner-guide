from setuptools import setup

package_name = 'piper_ros_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CodeHesed',
    maintainer_email='code.hesed@gmail.com',
    description='ROS2 Humble control package for Piper robot arm',
    license='Apache License 2.0',
    tests_require=['pytest'],
)