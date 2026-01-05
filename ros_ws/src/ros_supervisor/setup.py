from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pratyush',
    maintainer_email='dev@local',
    description='ROS 2 launch wrapper for ROS Supervisor backend',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros_supervisor_ping = ros_supervisor.cli:main',
        ],
    },
)
