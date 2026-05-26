from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rviz_gripper_e0509"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cube_team",
    maintainer_email="cube@todo.com",
    description="RViz RH-P12 gripper only with real robot pulse feedback",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gripper_joint_state = rviz_gripper_e0509.gripper_joint_state_node:main",
        ],
    },
)
