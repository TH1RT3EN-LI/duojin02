#!/usr/bin/env python3
"""Real guard epoch-cache checks using remapped synthetic inputs, no hardware."""
import json
import math
import os
import sys
import time
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from scan_fault_probe import arguments, CaseProbe, require, zero
import rclpy


def main():
    args = arguments()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.log_dir.mkdir(parents=True, exist_ok=True)
    report = {'test': 'guard_prior_association_epochs', 'result': 'running', 'cases': []}
    probe = None
    exit_code = 0
    rclpy.init(args=[])
    try:
        probe = CaseProbe('association', args)
        probe.wait_ready()
        tf_pub = probe.create_publisher(TFMessage, probe.topic('unused_tf'), 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = probe.create_publisher(OccupancyGrid, probe.topic('unused_map'), qos)
        covariance_pub = probe.create_publisher(PoseWithCovarianceStamped,
                                                probe.topic('unused_localization'), qos)
        yaw = 0.0

        def pump(duration):
            end = time.monotonic() + duration
            next_tf = 0.0
            while time.monotonic() < end:
                if time.monotonic() >= next_tf:
                    tf = TransformStamped()
                    tf.header.stamp = probe.get_clock().now().to_msg()
                    tf.header.frame_id = 'odom'
                    tf.child_frame_id = 'map'
                    tf.transform.rotation.z = math.sin(yaw / 2)
                    tf.transform.rotation.w = math.cos(yaw / 2)
                    tf_pub.publish(TFMessage(transforms=[tf]))
                    next_tf = time.monotonic() + .010
                probe.spin()

        def fresh_scan():
            scan = probe.scan([math.inf] * 33)
            tf = TransformStamped()
            tf.header.stamp = scan.header.stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'map'
            tf.transform.rotation.z = math.sin(yaw / 2)
            tf.transform.rotation.w = math.cos(yaw / 2)
            tf_pub.publish(TFMessage(transforms=[tf]))
            # The existing guard also requires source-time map anchoring. An
            # absent future TF is a fixture failure, not cache invalidation.
            probe.pump(.006)
            probe.scan_pub.publish(scan)

        def prior(width=1):
            m = OccupancyGrid()
            m.header.frame_id = 'map'
            m.header.stamp = probe.get_clock().now().to_msg()
            m.info.width, m.info.height = width, 1
            m.info.resolution = .01
            m.info.origin.position.x = .345
            m.info.origin.orientation.w = 1.0
            m.data = [100] * width
            return m

        def uncertainty(margin):
            m = PoseWithCovarianceStamped()
            m.pose.pose.orientation.w = 1.0
            m.pose.covariance[0] = m.pose.covariance[7] = (margin / 2) ** 2
            covariance_pub.publish(m)

        def observe(name, change, expect_miss=True):
            # Freshness belongs to the original scan. Keep this observation
            # below 300 ms; do not send another scan inside the cache window.
            fresh_scan()
            before = time.monotonic()
            pump(.115)
            ready = [v for received, v in probe.statuses if received >= before]
            if name not in ('map_first_then_same_epoch', 'association_reenabled'):
                require(any(v['prior_association_hits'] > 0 for v in ready),
                        'Pre-change association epoch was not stably cached: ' + name,
                        statuses=ready)
            since = time.monotonic()
            change()
            pump(.125)
            values = [v for received, v in probe.statuses if received >= since
                      and v['reason'] not in ('scan_or_tf_stale', 'command_stale')]
            require(values, 'No active guard status for ' + name, **probe.snapshot())
            if expect_miss:
                require(any(v['prior_association_misses'] > 0 for v in values),
                        'Changed association context reused old results: ' + name,
                        statuses=values)
                require(any(v['prior_association_hits'] > 0 for v in values),
                        'Stable context did not reuse results: ' + name, statuses=values)
            else:
                require(any(v['prior_association_hits'] == 0
                            and v['prior_association_misses'] == 0 for v in values),
                        'High uncertainty must bypass association', statuses=values)
            report['cases'].append({'case': name, 'result': 'passed', 'statuses': values})

        deadline = time.monotonic() + args.discovery_timeout_s
        while not all(pub.get_subscription_count() for pub in (tf_pub, map_pub, covariance_pub)):
            require(time.monotonic() < deadline, 'Synthetic prior/TF discovery timed out')
            pump(.02)
        pump(.05)
        observe('map_first_then_same_epoch', lambda: map_pub.publish(prior()))
        observe('new_scan_epoch', fresh_scan)
        observe('same_size_map_replacement', lambda: map_pub.publish(prior()))
        def rotate():
            nonlocal yaw
            yaw = .73
        observe('map_transform_change', rotate)
        observe('localization_uncertainty_change', lambda: uncertainty(.02))
        observe('prior_size_change', lambda: map_pub.publish(prior(2)))
        observe('association_disabled', lambda: uncertainty(.16), expect_miss=False)
        observe('association_reenabled', lambda: uncertainty(.02))
        pump(.35)
        require(probe.statuses[-1][1]['reason'] == 'scan_or_tf_stale'
                and zero(probe.outputs[-1][1]), 'Cache extended stale scan validity', **probe.snapshot())
        report['cases'].append({'case': 'cache_cannot_refresh_scan_heartbeat', 'result': 'passed'})
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
                report['failure'] = {'type': 'GuardCleanupFailure',
                                     'message': 'Guard did not exit normally',
                                     'returncode': probe.process.returncode}
                exit_code = 1
        rclpy.shutdown()
    body = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(body)
    print(body)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
