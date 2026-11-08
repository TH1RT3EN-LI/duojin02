#!/usr/bin/env python3
"""Compile/run actual current native controller/checker with declared plant only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--root',type=Path,default=Path('/work'))
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
build=a.root/'build/graph-update-cuda/duojin02_navigation/CMakeFiles/precise_mppi_limit_checks.dir'
flags=[]
for key in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS'):
    flags+=shlex.split(re.search(r'^'+key+r' = (.*)$',(build/'flags.make').read_text(),re.M).group(1))
flags+=['-isystem','/opt/ros/humble/include/diagnostic_msgs']
link=shlex.split((build/'link.txt').read_text())
libs=[str(build.parents[1]/x) if not x.startswith(('-', '/')) and (build.parents[1]/x).is_file() else x for x in link[link.index('-o')+2:]]+['/opt/ros/humble/lib/libdiagnostic_msgs__rosidl_typesupport_cpp.so']
source=a.root/'tests/realtime_goal/navigation/native_task_loop_probe.cpp'
production=[a.root/'src/duojin02_navigation/src/precise_mppi.cpp',
            a.root/'src/duojin02_navigation/include/duojin02_navigation/stable_goal_checker.hpp',
            a.root/'src/duojin02_navigation/include/duojin02_navigation/task_goal_contract.hpp',
            a.root/'src/duojin02_bringup/duojin02_bringup/task_goal_evaluator.py',source]
manifest={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in production}
(a.output/'source-sha256-before.json').write_text(json.dumps(manifest,indent=2))
exe=a.output/'native_task_loop_probe'
commands=[['/usr/bin/c++',*flags,str(source),'-o',str(exe),*libs],
          [str(exe),str(a.output)],
          ['/usr/bin/python3','-B',str(a.root/'src/duojin02_bringup/duojin02_bringup/task_goal_evaluator.py'),
           '--protocol',str(a.output/'protocol.json'),'--tasks',str(a.output/'tasks.json'),
           '--samples',str(a.output/'samples.jsonl'),'--output',str(a.output/'evaluation.json')]]
env=os.environ.copy()
env.update(ROS_DOMAIN_ID='219',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
           ROS_LOG_DIR=str(a.output/'ros_logs'),AMENT_PREFIX_PATH='/work/install/cuda-local-v3:/opt/ros/humble',
           LD_LIBRARY_PATH='/work/install/cuda-local-v3/lib:/opt/ros/humble/lib:'+env.get('LD_LIBRARY_PATH',''))
records=[]
for command in commands:
    start=time.monotonic()
    result=subprocess.run(command,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=180)
    log=f'command-{len(records)}.log'
    (a.output/log).write_text(result.stdout)
    records.append(dict(argv=command,returncode=result.returncode,wall_s=time.monotonic()-start,log=log))
    (a.output/'commands.json').write_text(json.dumps(records,indent=2))
    if result.returncode:
        raise SystemExit(result.returncode)
after={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in production}
(a.output/'source-sha256-after.json').write_text(json.dumps(after,indent=2))
if after!=manifest:
    raise RuntimeError('source changed during compilation/run; mixed source artifact rejected')
(a.output/'validation.json').write_text(json.dumps(dict(passed=True,
    evidence='native_controller_and_goal_checker_declared_synthetic_plant',
    external_smoother_mux_guard=False, full_navigation=False,physical_arrival_guarantee=False),indent=2))
