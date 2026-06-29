from setuptools import setup
import os
from glob import glob

package_name = "tower_light_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeonbotdae",
    maintainer_email="vkrtp2@gmail.com",
    description="Tower light color detector for quadruped robot competition",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detector  = tower_light_detector.detector_node:main",
            "mission   = tower_light_detector.mission_node:main",
            "calibrate = tower_light_detector.calibrate_hsv:main",
        ],
    },
)
