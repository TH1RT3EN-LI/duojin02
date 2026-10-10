from setuptools import setup
from glob import glob

setup(name='duojin02_sim', version='0.1.0', packages=['duojin02_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/duojin02_sim']),
                  ('share/duojin02_sim', ['package.xml']),
                  ('share/duojin02_sim/launch', glob('launch/*.py')),
                  ('share/duojin02_sim/config', glob('config/*.yaml') + glob('config/*.xml') + glob('config/*.config') + glob('config/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='litianshun', maintainer_email='litianshun.cn@gmail.com',
      description='Gazebo hardware interface adapter', license='GPL-3.0-only',
      entry_points={'console_scripts': ['device_adapter = duojin02_sim.adapter:main',
                                       'navigation_readiness = duojin02_sim.readiness:main']})
