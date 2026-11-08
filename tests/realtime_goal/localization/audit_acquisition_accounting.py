#!/usr/bin/env python3
"""Conditional source accounting for acquisition; never opens ground truth.

The native capture decision is treated as an event, not independently certified.
Original TF and thresholds determine every subsequent required native pose.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from audit_motion_selection import interpolate, wrap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--replay-dir', type=Path, required=True)
    parser.add_argument('--output-name', default='acquisition_accounting.json')
    args = parser.parse_args()
    report = json.loads((args.replay_dir / 'replay.json').read_text())
    sources = json.loads((args.replay_dir / 'scan_sources.json').read_text())
    poses = json.loads((args.replay_dir / 'poses.json').read_text())
    parameters = report['parameters']
    assert parameters.get('global_acquisition_enabled'), 'Only acquisition-enabled runs'
    rows = report['acquisition_diagnostics']
    captured = [r for r in rows if r['state'] == 'acquired_engineering_gate']
    checks = {'at_most_one_capture': len(captured) <= 1}
    by_raw = {s['raw_stamp_ns']: s for s in sources}
    checks['acquisition_logs_bound_to_source_identity_and_reference'] = all(
        int(r['raw_stamp_ns']) in by_raw and
        int(r['reference_stamp_ns']) == by_raw[int(r['raw_stamp_ns'])]['reference_stamp_ns'] and
        by_raw[int(r['raw_stamp_ns'])]['support_valid'] and
        r['source_frame'].lstrip('/') + ':' + r['raw_stamp_ns'] == by_raw[int(r['raw_stamp_ns'])]['source_scan_id']
        for r in rows)
    capture_ns = int(captured[0]['reference_stamp_ns']) if captured else None
    expected_acquisition = [s for s in sources if s['support_valid'] and
                            (capture_ns is None or s['reference_stamp_ns'] <= capture_ns)]
    checks['all_pending_inputs_acknowledged_once_in_order'] = (
        [int(r['raw_stamp_ns']) for r in rows] == [s['raw_stamp_ns'] for s in expected_acquisition])
    checks['no_acquisition_pose_or_pf_likelihood'] = all(
        r['pose_published'] == '0' and r['scan_pf_likelihood_applied'] == '0' for r in rows)
    checks['no_pose_for_consumed_capture_or_pending_scan'] = all(
        capture_ns is not None and p['reference_stamp_ns'] > capture_ns for p in poses)
    edge = (parameters['odom_frame_id'], parameters['base_frame_id'])
    samples = {}
    connection = sqlite3.connect(f'file:{args.bag}?mode=ro', uri=True)
    for data, in connection.execute("SELECT messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name='/tf' ORDER BY messages.timestamp,messages.id"):
        for t in deserialize_message(bytes(data), TFMessage).transforms:
            if (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/')) != edge:
                continue
            q = t.transform.rotation
            assert abs(q.x) < 1e-10 and abs(q.y) < 1e-10, 'Nonplanar odometry'
            v = t.transform.translation
            samples[t.header.stamp.sec * 10**9 + t.header.stamp.nanosec] = (
                v.x, v.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z))
    connection.close()
    stamps = sorted(samples)
    expected = []
    last = None
    for source in sources:
        ns = source['reference_stamp_ns']
        if capture_ns is None or ns <= capture_ns or not source['support_valid']:
            continue
        index = bisect.bisect_left(stamps, ns)
        if index < len(stamps) and stamps[index] == ns:
            pose = samples[ns]
        else:
            assert 0 < index < len(stamps), 'Missing post-capture odometry bracket'
            left, right = stamps[index-1], stamps[index]
            pose = interpolate(samples[left], samples[right], (ns-left)/(right-left),
                               parameters['scan_motion_compensation'])
        selected = (last is None or abs(pose[0]-last[0]) > parameters['update_min_d'] or
                    abs(pose[1]-last[1]) > parameters['update_min_d'] or
                    abs(wrap(pose[2]-last[2])) > parameters['update_min_a'])
        if selected:
            expected.append(source['source_scan_id'])
            last = pose
    actual = [p['source_scan_id'] for p in poses]
    checks['post_capture_motion_source_order_exact'] = expected == actual
    checks['all_scans_published'] = report['published_scans'] == len(sources)
    checks['no_unexpected_geometry_rejection'] = not report['unexpected_rejected_raw_stamps']
    result = {'scope': 'conditional source accounting; capture event from native log; geometry correctness needs independent evaluation',
              'status': 'passed' if all(checks.values()) else 'failed', 'checks': checks,
              'input_count': len(sources), 'acquisition_input_count': len(rows),
              'capture_reference_stamp_ns': capture_ns, 'native_pose_count': len(poses),
              'expected_post_capture_pose_count': len(expected),
              'expected_post_capture_source_ids': expected,
              'missing_pose_source_ids': sorted(set(expected)-set(actual)),
              'unexpected_pose_source_ids': sorted(set(actual)-set(expected)),
              'full_input_without_native_pose': len(sources)-len(set(actual)),
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    assert Path(args.output_name).name == args.output_name, 'Output must be a basename'
    output = args.replay_dir / args.output_name
    assert not output.exists(), 'Use a new run; accounting outputs are immutable'
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['status', 'checks', 'input_count', 'acquisition_input_count', 'native_pose_count']}))
    assert result['status'] == 'passed', 'Acquisition accounting failed'


if __name__ == '__main__':
    main()
