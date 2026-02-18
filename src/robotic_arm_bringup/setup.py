import glob
import os

from setuptools import find_packages, setup

package_name = "robotic_arm_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="daniel_iacob",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "scene_manager = robotic_arm_bringup.scene_manager:main",
            "move_to_cube = robotic_arm_bringup.move_to_cube:main",
        ],
    },
)
