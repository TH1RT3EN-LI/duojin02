#!/usr/bin/env python3
"""Require zero output for invalid grids and recovery on the original valid grid."""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

from ros_isolation import configure_isolation
from scan_fault_probe import CaseProbe, clear_command, require, zero


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard-binary', type=Path, required=True)
    parser.add_argument('--params-file', type=Path, required=True)
    parser.add_argument('--json-out', type=Path, required=True)
    parser.add_argument('--log-dir', type=Path, required=True)
    args = parser.parse_args()
    args.discovery_timeout_s = 8.0
    args.log_dir.mkdir(parents=True, exist_ok=True)
    report = {'test': 'guard_map_input_validation', 'result': 'running',
              'ros_domain_id': configure_isolation(), 'cases': []}
    probe = None
    exit_code = 0
    rclpy.init(args=[])
    try:
        probe = CaseProbe('grid_validation', args)
        probe.wait_ready()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publishers = {
            'map': probe.create_publisher(OccupancyGrid, probe.topic('unused_map'), qos),
            'costmap': probe.create_publisher(OccupancyGrid, probe.topic('unused_costmap'), qos),
        }
        tf_publisher = probe.create_publisher(TFMessage, probe.topic('unused_tf'), 10)

        def pump(duration):
            deadline = time.monotonic() + duration
            next_sample = 0.0
            while time.monotonic() < deadline:
                if time.monotonic() >= next_sample:
                    transform = TransformStamped()
                    transform.header.stamp = probe.get_clock().now().to_msg()
                    transform.header.frame_id, transform.child_frame_id = 'odom', 'map'
                    transform.transform.rotation.w = 1.0
                    tf_publisher.publish(TFMessage(transforms=[transform]))
                    probe.scan_pub.publish(probe.scan())
                    next_sample = time.monotonic() + .02
                probe.spin()

        deadline = time.monotonic() + args.discovery_timeout_s
        while not all(publisher.get_subscription_count() for publisher in
                      [*publishers.values(), tf_publisher]):
            require(time.monotonic() < deadline, 'Grid/TF discovery timed out', **probe.snapshot())
            pump(.02)

        def observe(name, publisher, message, reason, stopped):
            message.header.stamp = probe.get_clock().now().to_msg()
            publisher.publish(message)
            pump(.15)
            command_valid = zero if stopped else clear_command
            require(probe.statuses[-1][1]['reason'] == reason and
                    command_valid(probe.outputs[-1][1]),
                    'Grid transition did not produce the expected guard state',
                    case=name, expected_reason=reason, **probe.snapshot())
            report['cases'].append({'case': name, 'result': 'passed', **probe.snapshot()})

        for kind, publisher in publishers.items():
            valid = OccupancyGrid()
            valid.header.frame_id = 'map' if kind == 'map' else 'odom'
            valid.info.width = valid.info.height = 1
            valid.info.resolution = .01
            valid.info.origin.orientation.w = 1.0
            valid.data = [0]
            invalid = deepcopy(valid)
            invalid.info.origin.position.x = math.nan
            observe(kind + '_valid_before_fault', publisher, valid, 'clear', False)
            observe(kind + '_invalid_origin_stops', publisher, invalid, kind + '_invalid', True)
            observe(kind + '_original_valid_grid_recovers', publisher, valid, 'clear', False)
        report['result'] = 'passed'
    except BaseException as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        report['result'] = 'failed'
        report['failure'] = {'type': type(error).__name__, 'message': str(error),
                             'evidence': getattr(error, 'evidence', {})}
    finally:
        if probe is not None:
            probe.close()
            report['cleanup'] = {'guard_returncode': probe.process.returncode}
            if probe.process.returncode != 0:
                report['result'] = 'failed'
                exit_code = 1
        rclpy.try_shutdown()
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps(report, indent=2, allow_nan=False) + '\n'
    args.json_out.write_text(body)
    print(body)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
