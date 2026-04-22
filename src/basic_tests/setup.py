from setuptools import find_packages, setup

package_name = 'basic_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zerv',
    maintainer_email='zervas1999@gmail.com',
    description='This is a pakage greated to test the basic functions of mavros and PX4 on the gazebo simulator',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'disarm = basic_tests.disarm:main',
            'takeoff = basic_tests.takeoff_offboard:main'
        ],
    },
)
