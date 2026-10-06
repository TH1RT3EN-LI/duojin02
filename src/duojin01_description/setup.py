from setuptools import setup
from glob import glob
import os

package_name = 'duojin01_description'

# Keep the model hierarchy intact so SDF and package:// URIs resolve after install.
model_data_files = [
    (os.path.join('share', package_name, directory),
     [os.path.join(directory, name) for name in sorted(files)])
    for directory, _, files in sorted(os.walk('models'))
    if files
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ] + model_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='duojin01 robot description',
    license='GPL-3.0-only',
    entry_points={'console_scripts': []},
)
