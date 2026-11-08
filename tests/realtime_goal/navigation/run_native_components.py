#!/usr/bin/env python3
"""Build current native components using frozen existing Humble build flags.

Invoke in the existing container: python3 /work/tests/.../run_native_components.py
--output /work/output/<new epoch>/navigation/native-A01. No launch or Action.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--root', type=Path, default=Path('/work'))
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(a.output/'ros_logs'),
           AMENT_PREFIX_PATH='/work/install/cuda-local-v3:/opt/ros/humble',
           LD_LIBRARY_PATH='/work/install/cuda-local-v3/lib:/opt/ros/humble/lib:' + env.get('LD_LIBRARY_PATH',''))
records = []
hashes = {}
for target in ('goal_settling_checks', 'precise_mppi_limit_checks'):
    build = a.root/'build/graph-update-cuda/duojin02_navigation/CMakeFiles'/f'{target}.dir'
    flags = []
    text = (build/'flags.make').read_text()
    for key in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS'):
        flags += shlex.split(re.search(r'^'+key+r' = (.*)$',text,re.M).group(1))
    flags += ['-isystem','/opt/ros/humble/include/diagnostic_msgs']
    link = shlex.split((build/'link.txt').read_text())
    libs = link[link.index('-o')+2:]
    libs = [str(build.parents[1]/x) if not x.startswith(('-', '/')) and (build.parents[1]/x).is_file() else x for x in libs]
    if target == 'precise_mppi_limit_checks':
        libs += ['/opt/ros/humble/lib/libdiagnostic_msgs__rosidl_typesupport_cpp.so']
    source = a.root/'src/duojin02_navigation/test'/f'{target}.cpp'
    obj = a.output/f'{target}.o'
    exe = a.output/target
    commands = [['/usr/bin/c++',*flags,'-c',str(source),'-o',str(obj)],
                ['/usr/bin/c++',str(obj),'-o',str(exe),*libs], [str(exe)]]
    for command in commands:
        index = len(records)
        start = time.monotonic()
        result = subprocess.run(command, env=env, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=180)
        log = f'command-{index}.log'
        (a.output/log).write_text(result.stdout)
        records.append(dict(argv=command,returncode=result.returncode,wall_s=time.monotonic()-start,
                            log=log,environment={k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','LD_LIBRARY_PATH')}))
        (a.output/'commands.json').write_text(json.dumps(records,indent=2))
        if result.returncode:
            raise SystemExit(result.returncode)
    hashes[str(exe)] = hashlib.sha256(exe.read_bytes()).hexdigest()
(a.output/'binary-sha256.json').write_text(json.dumps(hashes,indent=2))
(a.output/'result.json').write_text(json.dumps(dict(passed=True, fresh_native_components=True,
    controller='actual_current_PreciseMPPI', robot_action_sent=False,
    full_navigation=False, physical_arrival_evidence=False),indent=2))
