from glob import glob
import os

from setuptools import setup

package_name = "auv_mission_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "tasks"),
         glob("config/tasks/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "keyboard_node = auv_mission_control.keyboard_node:main",
            "web_keyboard = auv_mission_control.web_keyboard:main",
            "hold_node = auv_mission_control.hold_node:main",
            "autonomy_node = auv_mission_control.autonomy_node:main",
        ],
    },
)
