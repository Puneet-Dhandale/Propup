import os
from glob import glob
from setuptools import setup

package_name = 'go2_simple_joint_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujal',
    maintainer_email='sujal@example.com',
    description='Simple joint position control template for Go2 robot in Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_joint_controller = go2_simple_joint_control.simple_joint_controller:main',
            'joint_command_publisher = go2_simple_joint_control.joint_command_publisher:main',
            'keyboard_teleop = go2_simple_joint_control.keyboard_teleop:main',
            'gait_generator = go2_simple_joint_control.gait_generator:main',
            'imu_balance_controller = go2_simple_joint_control.imu_balance_controller:main',
            'stance_force_controller = go2_simple_joint_control.stance_force_controller:main'
        ],
    },
)
