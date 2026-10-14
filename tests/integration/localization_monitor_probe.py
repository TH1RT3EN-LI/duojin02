"""Verify advisory localization quality cannot interrupt the command chain.

Uses installed Humble APIs, real DDS and compiled mux/guard/serial driver.
The circular map is a controlled sensor fixture, not simulator ground truth.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tty
import yaml

import rclpy

from navigation_probe_fixture import ROOT, SensorFixture
from duojin02_bringup.safety_profile import mux_parameters
from ros_isolation import configure_isolation


def run(args):
    domain = configure_isolation()
    rclpy.init()
    fixture = SensorFixture()
    processes, logs = [], []
    result = {'pass': False, 'checks': [], 'cleanup': [], 'ros_domain_id': domain,
              'scope': 'DDS, controlled scan/map, compiled mux/guard, PTY driver; no physical robot'}

    def start(name, command):
        log = (args.output_dir/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process

    def check(name, condition):
        assert condition, {'case': name, 'status': fixture.status[-1:] ,
                           'output': fixture.outputs[-3:], 'wire': fixture.wire_packets[-3:],
                           'guard': fixture.guard_status[-1:]}
        result['checks'].append(name)

    args.output_dir.mkdir(parents=True, exist_ok=False)
    mux_file = args.output_dir/'mux.yaml'
    mux_file.write_text(yaml.safe_dump({'twist_mux': {'ros__parameters': mux_parameters(
        ROOT/'src/duojin02_bringup/config/twist_mux.yaml')}}))
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    fixture.master = master
    safety = str(ROOT/'src/duojin02_bringup/config/safety.yaml')
    try:
        monitor = start('monitor', ['ros2', 'run', 'duojin02_bringup', 'localization_monitor',
            '--ros-args', '--params-file', str(ROOT/'src/duojin02_bringup/config/localization_monitor.yaml'),
            '-p', 'use_sim_time:=true'])
        start('mux', ['ros2','run','twist_mux','twist_mux','--ros-args',
            '--params-file',str(mux_file),
            '-r','/cmd_vel_out:=/cmd_vel_requested'])
        # Node-scoped override wins over the hardware YAML clock setting.
        start('guard', [str(args.guard_binary),'--ros-args','--params-file', safety, '-p','velocity_guard:use_sim_time:=true'])
        start('driver', [str(args.driver_binary),'--ros-args','--params-file', safety,
            '-p',f'usart_port_name:={os.ttyname(slave)}', '-p','serial_baud_rate:=57600',
            '-p','use_sim_time:=true','-r','/cmd_vel:=/cmd_vel_base'])
        fixture.pump(.8, scan=False, nav=.24)
        check('guard_stops_without_initial_scan', bool(fixture.wire_packets) and fixture.wire_packets[-1][1] == (0,0,0))
        fixture.pump(2.0, nav=.24)
        check('consistent_scan_map_diagnostic', bool(fixture.status) and fixture.status[-1][1]['quality'] == 'consistent')
        check('direct_navigation_reaches_serial', fixture.wire_packets[-1][1] == (240,0,0))
        fixture.pump(.8, range_m=.9, nav=.24)
        check('inconsistent_quality_is_advisory', fixture.status[-1][1]['quality'] == 'inconsistent' and
              fixture.status[-1][1]['advisory_only'] is True and fixture.wire_packets[-1][1] == (240,0,0))
        fixture.pump(.7, nav=.24)
        check('quality_recovers_without_service_or_dwell', fixture.status[-1][1]['quality'] == 'consistent')
        os.killpg(monitor.pid, signal.SIGSTOP)
        fixture.pump(.7, nav=.24)
        check('monitor_pause_does_not_control_motion', fixture.wire_packets[-1][1] == (240,0,0))
        os.killpg(monitor.pid, signal.SIGCONT)
        fixture.pump(.7, nav=.24)
        os.killpg(monitor.pid, signal.SIGINT)
        monitor.wait(timeout=8)
        fixture.pump(.7, nav=.24)
        check('monitor_exit_does_not_control_motion', monitor.returncode == 0 and fixture.wire_packets[-1][1] == (240,0,0))
        fixture.pump(.7, scan=False, nav=.24)
        check('actual_scan_loss_still_stops', fixture.wire_packets[-1][1] == (0,0,0))
        fixture.pump(.7, nav=.24)
        check('fresh_sensor_data_resumes_without_latched_quality', fixture.wire_packets[-1][1] == (240,0,0))
        fixture.pump(.7, invalid=True, nav=.24)
        check('invalid_scan_still_stops', fixture.wire_packets[-1][1] == (0,0,0))
        fixture.pump(.7, nav=.24)
        check('valid_scan_resumes_after_malformed_scan', fixture.wire_packets[-1][1] == (240,0,0))
        fixture.pump(.7)
        check('direct_command_loss_still_stops', fixture.wire_packets[-1][1] == (0,0,0))
        count = len(fixture.outputs)
        fixture.pump(.7, manual=.12)
        check('idle_navigation_has_no_cached_republisher', len(fixture.outputs) == count)
        check('manual_takeover_reaches_serial', fixture.wire_packets[-1][1] == (120,0,0))
        result['pass'] = True
    finally:
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGCONT)
                os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=8)
            result['cleanup'].append({'name': name, 'returncode': returncode, 'forced': False})
            result['pass'] = result['pass'] and returncode == 0
        result['published'] = {'clock':fixture.clock_count, 'scans':fixture.scan_count, 'commands':fixture.command_count}
        result['observed'] = {'diagnostics':len(fixture.status), 'commands':len(fixture.outputs),
                              'guard':len(fixture.base), 'serial_packets':len(fixture.wire_packets)}
        result['final_status'] = fixture.status[-1][1] if fixture.status else None
        (args.output_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        fixture.close()
        rclpy.try_shutdown()
        os.close(master)
        os.close(slave)
        for log in logs:
            log.close()
    assert result['pass'], result['cleanup']
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--guard-binary', type=Path, required=True)
    parser.add_argument('--driver-binary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    run(parser.parse_args())
