from setuptools import setup
import os
from glob import glob

package_name = 'odrive_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 package for controlling ODrive motor controllers in a tracked robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_node = odrive_control.odrive_node:main',
            'teleop_node = odrive_control.teleop_node:main',
            'odometry_node = odrive_control.odometry_node:main',
        ],
    },
)
