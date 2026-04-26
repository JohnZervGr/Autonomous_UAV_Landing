from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zerv',
    maintainer_email='zervas1999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_control_node = drone_control.controller:main',
            'camera_tracker_node = drone_control.camera_tracker:main',
            'mission_controller_node = drone_control.mission_controller:main',
        ],
    },
)
