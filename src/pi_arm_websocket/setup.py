from glob import glob
from setuptools import find_packages, setup

package_name = "pi_arm_websocket"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "websockets"],
    zip_safe=True,
    maintainer="Pi Arm Maintainer",
    maintainer_email="maintainer@example.com",
    description="JSON WebSocket facade for the Pi Arm ROS manager.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["pi_arm_websocket = pi_arm_websocket.gateway:main"]},
)
