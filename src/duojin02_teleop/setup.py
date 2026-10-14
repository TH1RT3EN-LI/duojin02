from setuptools import setup
import os

package_name = "duojin02_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=["duojin02_teleop"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            [
                "config/joy_launcher.yaml",
                "config/joy_axis_selector.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="李天顺",
    maintainer_email="litianshun.cn@gmail.com",
    description="Keyboard and joystick teleoperation for the Duojin mobile base.",
    license="GPL-3.0-only",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "joy_launcher_node = duojin02_teleop.joy_launcher_node:main",
            "joy_axis_selector_node = duojin02_teleop.joy_axis_selector_node:main",
            "keyboard_teleop_node = duojin02_teleop.keyboard_teleop_node:main",
        ],
    },
)
