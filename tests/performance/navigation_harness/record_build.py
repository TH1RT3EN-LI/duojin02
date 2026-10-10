"""Record and later verify exact sources and compiled navigation binaries."""
import hashlib
import json
import platform
import os
import subprocess
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root=Path('/hardware').resolve();build=Path('/tmp/navbuild').resolve()
    sources=list((root/'src/duojin01_navigation').rglob('*'))
    sources += list((root/'src/duojin01_bringup').rglob('*'))
    sources += list(Path('/candidate/tests/performance/navigation_harness/sim_fixture').rglob('*'))
    sources += list(Path('/candidate/tests/performance/navigation_harness').glob('*.py'))
    sources=[p for p in sources if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
    binaries=list((build/'install/duojin01_navigation/lib').glob('*.so'))
    binaries += [build/'install/duojin01_navigation/lib/duojin01_navigation/velocity_guard']
    manifest=dict(sources={str(p.resolve()):digest(p) for p in sources},binaries={str(p.resolve()):digest(p) for p in binaries})
    manifest['slam_backend']=json.loads(Path('/candidate/.cache/slam-install/share/slam_toolbox/hardware_build.json').read_text())
    manifest['platform']=dict(architecture=platform.machine(),kernel=platform.release(),rmw_implementation=os.environ.get('RMW_IMPLEMENTATION','default'),
      packages=subprocess.run(['dpkg-query','-W','ros-humble-nav2-smac-planner','ros-humble-nav2-mppi-controller',
                              'ros-humble-nav2-amcl','libgz-sim8','ros-humble-rmw-cyclonedds-cpp'],capture_output=True,text=True,check=True).stdout)
    (build/'navigation-build.json').write_text(json.dumps(manifest,indent=2))
    print('Saved compiled navigation manifest')


if __name__=='__main__':main()
