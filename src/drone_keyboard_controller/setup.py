from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_keyboard_controller'

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
    install_requires=['setuptools','keyboard'],
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
            'twist_controller = drone_keyboard_controller.twist_mavros_bridge:main',
            'keyboard_controller = drone_keyboard_controller.controller:main',
        ],
    },
)
