from setuptools import setup
from glob import glob
import os

package_name = "duojin02_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="李天顺",
    maintainer_email="litianshun.cn@gmail.com",
    description="duojin02 robot description",
    license="GPL-3.0-only",
    entry_points={"console_scripts": []},
)
