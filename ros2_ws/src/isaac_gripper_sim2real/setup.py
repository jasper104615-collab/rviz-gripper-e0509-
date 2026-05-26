import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'isaac_gripper_sim2real'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cube_team',
    maintainer_email='cube@todo.com',
    description='Isaac gripper sim2real bridge (rad to stroke/pulse)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_bridge = isaac_gripper_sim2real.isaac_gripper_bridge_node:main',
            'rviz_joint_state_merger = isaac_gripper_sim2real.rviz_joint_state_merger_node:main',
        ],
    },
)
