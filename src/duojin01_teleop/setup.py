from setuptools import find_packages, setup
import os
package_name = 'duojin01_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=['duojin01_teleop'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_launcher_node = duojin01_teleop.joy_launcher_node:main',
            'joy_axis_selector_node = duojin01_teleop.joy_axis_selector_node:main',
            'keyboard_teleop_node = duojin01_teleop.keyboard_teleop_node:main',
        ],
    },
)
