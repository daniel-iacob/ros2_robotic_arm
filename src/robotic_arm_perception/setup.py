from setuptools import find_packages, setup

package_name = "robotic_arm_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="todo@todo.com",
    description="Camera and vision pipeline for robotic arm object detection",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "camera_node = robotic_arm_perception.camera_node:main",
            "vision_node = robotic_arm_perception.vision_node:main",
        ],
    },
)
