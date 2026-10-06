#!/usr/bin/env python3
"""Read-only native-board profiling during a separately commanded navigation run."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def percentile(values,q):
    if not values:return None
    values=sorted(values);return values[min(len(values)-1,int((len(values)-1)*q))]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--output',default='navigation-profile.json');args=parser.parse_args()
    if args.seconds<=0:parser.error('--seconds must be positive')
    rclpy.init();node=Node('navigation_native_profile')
    stamps={name:[] for name in ('/cmd_vel_nav','/cmd_vel_base','/scan')};compute=[];reasons=Counter();resources={}
    subscriptions=[]
    for topic in ('/cmd_vel_nav','/cmd_vel_base'):
        subscriptions.append(node.create_subscription(Twist,topic,lambda m,t=topic:stamps[t].append(time.monotonic()),10))
    subscriptions.append(node.create_subscription(LaserScan,'/scan',lambda m:stamps['/scan'].append(time.monotonic()),qos_profile_sensor_data))
    def guard(message):
        value=json.loads(message.data);compute.append(value['compute_ms']);reasons[value['reason']]+=1
    subscriptions.append(node.create_subscription(String,'/navigation/guard_status',guard,10))
    start=time.monotonic();last_sample=0
    while time.monotonic()-start<args.seconds:
        rclpy.spin_once(node,timeout_sec=.02);wall=time.monotonic()
        if wall-last_sample<.5:continue
        last_sample=wall
        for directory in Path('/proc').iterdir():
            if not directory.name.isdigit():continue
            try:
                name=Path((directory/'cmdline').read_bytes().split(b'\0')[0].decode()).name
                if name not in {'controller_server','planner_server','amcl','async_slam_toolbox_node','velocity_guard','collision_monitor','ekf_node'}:continue
                fields=(directory/'stat').read_text().split(') ',1)[1].split()
                cpu=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
                rss=int(fields[21])*os.sysconf('SC_PAGE_SIZE')/2**20
                resources.setdefault((name,directory.name),[]).append((wall,cpu,rss))
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    board_power=None
    if shutil.which('nvpmodel'):
        try:
            power=subprocess.run(['nvpmodel','-q'],capture_output=True,text=True,timeout=3)
            board_power=dict(returncode=power.returncode,output=(power.stdout+power.stderr).strip())
        except subprocess.TimeoutExpired:board_power=dict(error='nvpmodel query timed out')
    frequencies={str(p.parent.parent.name):int(p.read_text().strip()) for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq') if p.is_file()}
    data=dict(rmw_implementation=get_rmw_implementation_identifier(),cpu_online=Path('/sys/devices/system/cpu/online').read_text().strip(),cpu_frequency_khz=frequencies,power_mode=board_power,architecture=platform.machine(),kernel=platform.release(),duration_s=time.monotonic()-start,
              memory_total_kib=int(Path('/proc/meminfo').read_text().splitlines()[0].split()[1]),
              l4t_release=Path('/etc/nv_tegra_release').read_text().strip() if Path('/etc/nv_tegra_release').exists() else None,
              guard_compute_p95_ms=percentile(compute,.95),guard_reasons=dict(reasons),topics={},nodes={})
    for topic,values in stamps.items():
        gaps=[(b-a)*1000 for a,b in zip(values,values[1:])]
        data['topics'][topic]=dict(messages=len(values),period_p95_ms=percentile(gaps,.95),period_max_ms=max(gaps) if gaps else None)
    for (name,pid),values in resources.items():
        data['nodes'][f'{name}:{pid}']=dict(cpu_percent_one_core=100*(values[-1][1]-values[0][1])/max(.001,values[-1][0]-values[0][0]),
                                         peak_rss_mib=max(v[2] for v in values))
    data['navigation_stream_observed']=len(stamps['/cmd_vel_nav'])>=20
    Path(args.output).write_text(json.dumps(data,indent=2));print(json.dumps(data,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
