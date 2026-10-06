from setuptools import setup
import os
from glob import glob
package_name = 'duojin01_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=['duojin01_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'lattice_primitives'),
            glob('config/lattice_primitives/*.json') + glob('config/lattice_primitives/*.md')),
        
        (os.path.join('share', package_name, 'config', 'rviz'),
            glob('config/rviz/*.rviz')),
        
        (os.path.join('share', package_name, 'config', 'foxglove'),
            glob('config/foxglove/*.yaml') + glob('config/foxglove/*.json')),

        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='Duojin01 real-hardware bringup',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': ['localization_readiness = duojin01_bringup.localization_readiness:main'],
    },
)
