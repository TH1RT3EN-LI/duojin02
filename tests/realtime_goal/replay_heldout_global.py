#!/usr/bin/env python3
"""Held-out full-bag global-relocalization stress with uniform free-map initialization.

Never opens ground-truth data or reads/publishes a pose prior. Frozen known-map
geometry and original adapted scans/odom/TF are reused with exact source holdout.
It is a global-relocalization stress, not normal tracking or an accuracy claim.
"""
import argparse
import csv
from decimal import Decimal, ROUND_HALF_UP
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import sqlite3
import subprocess
import time

import yaml
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Empty


def require(value, message):
    if not value:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_pgm(path):
    # nav2/slam_toolbox map_saver writes an 8-bit P5 raster. Avoid a plotting
    # dependency in the isolated ROS replay container.
    blob = Path(path).read_bytes()
    cursor = 0
    tokens = []
    while len(tokens) < 4:
        while blob[cursor] in b" \t\r\n":
            cursor += 1
        if blob[cursor] == 35:
            cursor = blob.index(b"\n", cursor) + 1
            continue
        end = cursor
        while blob[end] not in b" \t\r\n":
            end += 1
        tokens.append(blob[cursor:end])
        cursor = end
    require(tokens[0] == b"P5" and int(tokens[3]) == 255, 'Only native 8-bit P5 maps supported')
    cursor += 2 if blob[cursor:cursor + 2] == b"\r\n" else 1
    width, height = map(int, tokens[1:3])
    require(len(blob) - cursor == width * height, 'Invalid native map raster size')
    return np.frombuffer(blob, dtype=np.uint8, offset=cursor).reshape(height, width)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['cuda'], default='cuda')
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--adaptation', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--training-source-roster', type=Path, required=True)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--arm', choices=['T0D0', 'T0D1'], required=True)
    parser.add_argument('--expected-prefix', type=Path, required=True)
    parser.add_argument('--executable', type=Path, default=Path('/work/install/cuda-local-v3/lib/duojin02_amcl_cuda/amcl'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--max-scans', type=int, default=0)
    parser.add_argument('--acquisition-aware', action='store_true',
                        help='Allow explicit pending acquisition instead of fabricating an initial pose; preserve all input scans')
    parser.add_argument('--preflight-only', action='store_true', help='CPU-only source/TF/map contract checks; never starts a node')
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False, parents=True)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    require(args.rate > 0, 'Replay rate must be positive')
    require(args.max_scans >= 0, 'max-scans must be nonnegative')
    report_path = args.output / 'replay.json'
    report = {'status': 'running', 'backend': args.backend, 'rate': args.rate,
              'arm': args.arm, 'scope': 'unseen full-bag uniform-free-map global-relocalization stress; no closed-loop control or GT accuracy',
              'initialization_contract': 'existing AMCL reinitialize_global_localization service; uniform free-map pose distribution; no GT or pose prior',
              'input_contract': 'identical original adapted CDR records, original timestamps and scan fields',
              'timing_scope': 'raw scan publish call to matched physical-reference-stamp AMCL pose receipt; sparse native motion-selected poses',
              'started_unix_s': time.time(), 'ros_domain_id': os.environ['ROS_DOMAIN_ID']}
    report['acquisition_aware'] = args.acquisition_aware
    if args.acquisition_aware:
        report['scope'] = 'unseen known-map global acquisition stress; full input denominator; no closed-loop control or GT accuracy'
        report['initialization_contract'] = ('existing AMCL reinitialize_global_localization service; explicit pending acquisition '
            'allowed according to effective parameters; no GT or pose prior')

    def save():
        report_path.write_text(json.dumps(report, indent=2) + '\n')

    bag = args.bag
    report['transport_adapter'] = json.loads(args.adaptation.read_text())
    report['timing_scope'] += '; original TF records sent 200 ms earlier in source time to avoid native TF asynchronous wait races'
    conn = sqlite3.connect(f'file:{bag}?mode=ro', uri=True)
    topics = dict(conn.execute('SELECT id,name FROM topics'))
    records = [(topics[tid], ts, bytes(data)) for tid, ts, data in
               conn.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id')]
    conn.close()
    report['bag_sha256'] = sha(bag)
    report['requested_max_scans'] = args.max_scans
    report['complete_bag_scan_count'] = sum(topic == '/scan' for topic, _, _ in records)
    if args.max_scans:
        scan_indices = [i for i, (topic, _, _) in enumerate(records) if topic == '/scan']
        last_scan_index = scan_indices[min(args.max_scans, len(scan_indices)) - 1]
        cutoff = records[last_scan_index][1] + 500000000
        records = [row for i, row in enumerate(records) if row[1] <= cutoff and (row[0] != '/scan' or i <= last_scan_index)]
    report['input_records'] = len(records)
    report['input_scans'] = sum(row[0] == '/scan' for row in records)
    message_hash = hashlib.sha256()
    for topic, ts, data in records:
        message_hash.update(topic.encode() + b'\0' + ts.to_bytes(8, 'little') + data)
    report['published_record_contract_sha256'] = message_hash.hexdigest()
    report['source_span_s'] = (records[-1][1] - records[0][1]) / 1e9

    map_path = args.map
    metadata = yaml.safe_load(map_path.read_text())
    pixels = read_pgm(map_path.parent / metadata['image'])
    occupancy = (255 - pixels.astype(float)) / 255.
    if metadata.get('negate', 0):
        occupancy = 1. - occupancy
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[occupancy > metadata['occupied_thresh']] = 100
    cells[occupancy < metadata['free_thresh']] = 0
    report['map_sha256'] = sha(map_path.parent / metadata['image'])

    parameters = yaml.safe_load(args.parameters.read_text())['amcl']['ros__parameters']
    parameters.update(use_sim_time=True, observation_backend=args.backend,
                      observation_cuda_min_particles=0, observation_log_every=1, filter_diagnostics_log_every=1, set_initial_pose=False,
                      likelihood_temperature_beta=1.0, resample_ess_ratio=0.0)
    expected_deskew = args.arm == 'T0D1'
    require(parameters['scan_motion_compensation'] == expected_deskew, 'Wrong arm deskew setting')
    require(parameters['capture_time_offset_sec'] == 0.0, 'T0 requires zero offset; uncalibrated offsets are prohibited')
    require(parameters['scan_reference_fraction'] == (.5 if expected_deskew else 0.), 'Wrong reference fraction')
    import duojin02_bringup.scan_geometry as contract_module
    report['geometry_contract_module'] = {'path': contract_module.__file__, 'sha256': sha(Path(contract_module.__file__))}
    from duojin02_bringup.scan_geometry import validate_map_contract, contract
    validate_map_contract(map_path, parameters)
    map_sidecar = map_path.with_suffix('.scan_geometry.json')
    require(map_sidecar.is_file(), 'New map must have native scan geometry sidecar')
    report['scan_geometry_contract'] = contract(parameters)
    map_artifacts = [map_path, map_path.parent / metadata['image'], map_sidecar]
    map_artifacts += [p for suffix in ('.data', '.posegraph', '.snapshot.json') if (p := map_path.with_suffix(suffix)).is_file()]
    report['map_artifact_sha256_before'] = {str(p): sha(p) for p in map_artifacts}
    report['training_source_roster_sha256'] = sha(args.training_source_roster)
    report['runtime_environment'] = {key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'CUDA_VISIBLE_DEVICES')}
    report['latency_experiment_scope'] = 'One active held-out GPU query; shared host remains externally loaded. Descriptive paced runtime, not isolated throughput.'
    require(float(metadata['origin'][2]) == 0., 'Rotated raster origins are unsupported')
    def seconds_to_ns(value):
        return int((Decimal.from_float(value) * Decimal(1000000000)).to_integral_value(rounding=ROUND_HALF_UP))
    dynamic_samples = {}
    for topic, _, cdr in records:
        if topic == '/tf':
            for transform in deserialize_message(cdr, TFMessage).transforms:
                edge = (transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/'))
                dynamic_samples.setdefault(edge, []).append(transform.header.stamp.sec * 1000000000 + transform.header.stamp.nanosec)
    target_edge = (parameters['odom_frame_id'], parameters['base_frame_id'])
    require(target_edge in dynamic_samples, 'Recorded direct odometry TF edge absent')
    tf_stamps = np.array(sorted(set(dynamic_samples[target_edge])), dtype=np.int64)
    report['dynamic_tf_support'] = {'edge': target_edge, 'samples': len(tf_stamps),
        'first_ns': int(tf_stamps[0]), 'last_ns': int(tf_stamps[-1]),
        'maximum_gap_ns': int(np.max(np.diff(tf_stamps)))}
    beam_offsets = {}
    scan_sources = []
    for record_index, (topic, record_ns, cdr) in enumerate(records):
        if topic != '/scan':
            continue
        scan = deserialize_message(cdr, LaserScan)
        raw_ns = scan.header.stamp.sec * 1000000000 + scan.header.stamp.nanosec
        increment = float(scan.time_increment)
        count = len(scan.ranges)
        require(count > 0 and math.isfinite(increment) and increment >= 0., 'Invalid scan timing')
        require(not expected_deskew or increment > 0., 'D1 requires positive beam increment')
        shift_ns = seconds_to_ns(parameters['scan_reference_fraction'] * (count - 1) * increment)
        support_reason = None
        if expected_deskew:
            key = (count, increment)
            if key not in beam_offsets:
                beam_offsets[key] = np.array([seconds_to_ns(i * increment) for i in range(count)] + [shift_ns], dtype=np.int64)
            queries = raw_ns + beam_offsets[key]
            indices = np.searchsorted(tf_stamps, queries)
            if np.any(queries < tf_stamps[0]):
                support_reason = 'missing_history'
            elif np.any(indices == len(tf_stamps)):
                support_reason = 'missing_future'
            else:
                exact = tf_stamps[indices] == queries
                lower = np.maximum(0, indices - 1)
                if np.any((~exact) & (tf_stamps[indices] - tf_stamps[lower] > seconds_to_ns(parameters['scan_max_tf_gap_sec']))):
                    support_reason = 'internal_tf_gap'
        scan_sources.append({'support_valid': support_reason is None, 'support_failure': support_reason, 'source_scan_id': f"{scan.header.frame_id.lstrip('/')}:{raw_ns}", 'source_scan_index': len(scan_sources), 'record_index': record_index,
            'record_stamp_ns': record_ns, 'raw_stamp_ns': raw_ns, 'reference_stamp_ns': raw_ns + shift_ns,
            'last_beam_stamp_ns': raw_ns + seconds_to_ns((count - 1) * increment),
            'time_increment': increment, 'beam_count': count, 'cdr_sha256': hashlib.sha256(cdr).hexdigest()})
    require(len({s['raw_stamp_ns'] for s in scan_sources}) == len(scan_sources), 'Duplicate raw scan stamps')
    require(len({s['reference_stamp_ns'] for s in scan_sources}) == len(scan_sources), 'Duplicate reference stamps')
    source_by_ref = {s['reference_stamp_ns']: s for s in scan_sources}
    source_by_index = {s['record_index']: s for s in scan_sources}
    (args.output / 'scan_sources.json').write_text(json.dumps(scan_sources) + '\n')
    training_sources = {row['source_scan_id'] for row in csv.DictReader(args.training_source_roster.open())}
    query_sources = {row['source_scan_id'] for row in scan_sources}
    overlap = sorted(training_sources.intersection(query_sources))
    report['holdout_contract'] = dict(training_sources=len(training_sources), query_sources=len(query_sources),
        shared_source_ids=overlap, source_roster=str(args.training_source_roster),
        interpretation='exact recorded scan identity holdout; not a guarantee of unseen physical scene')
    require(not overlap, 'Held-out query contains map-training source IDs')
    report['expected_geometry_unsupported_source_scan_ids'] = [s['source_scan_id'] for s in scan_sources if not s['support_valid']]
    report['expected_geometry_unsupported_raw_stamps'] = [s['raw_stamp_ns'] for s in scan_sources if not s['support_valid']]
    report['runner_sha256'] = sha(Path(__file__))
    report['scan_sources_sha256'] = sha(args.output / 'scan_sources.json')
    report['scan_cdr_contract_sha256'] = hashlib.sha256(b''.join(bytes.fromhex(s['cdr_sha256']) for s in scan_sources)).hexdigest()
    parameter_path = args.output / 'parameters.yaml'
    parameter_path.write_text(yaml.safe_dump({'amcl': {'ros__parameters': parameters}}))
    report['parameters'] = parameters
    save()

    # Independent wheel-motion support is frozen before the estimator exists.
    # This oracle reads only source scans/TF/thresholds, never map poses or GT.
    subprocess.run(['python3', str(Path(__file__).parent/'localization/audit_motion_selection.py'),
        '--bag', str(args.bag), '--replay-dir', str(args.output), '--expected-only'], check=True)
    predeclared = args.output.parent/'predeclared-motion'/args.arm
    predeclared.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.output/'motion_selection_expected.json', predeclared/'motion_selection_expected.json')

    if args.preflight_only:
        report.update(status='preflight_complete', estimator_started=False, gpu_started=False)
        save()
        print(json.dumps({'status': report['status'], 'input_scans': report['input_scans'],
                          'holdout_contract': report['holdout_contract']}))
        return

    rclpy.init(args=[])
    node = Node('lilocbench_amcl_replay')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {
        '/scan': node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data),
        '/odom': node.create_publisher(Odometry, '/odom', 100),
        '/tf': node.create_publisher(TFMessage, '/tf', 100),
        '/tf_static': node.create_publisher(TFMessage, '/tf_static', durable)}
    actual_publication_hash = hashlib.sha256()
    actual_publications = []

    def publish_record(index):
        topic, ns, data = records[index]
        actual_publication_hash.update(topic.encode() + b'\0' + ns.to_bytes(8, 'little') + data)
        actual_publications.append(index)
        publishers[topic].publish(data)
    map_pub = node.create_publisher(OccupancyGrid, '/map', durable)
    global_localization = node.create_client(Empty, '/reinitialize_global_localization')
    clock_pub = node.create_publisher(Clock, '/clock', 100)
    lifecycle = node.create_client(ChangeState, '/amcl/change_state')
    sent = {}
    received = []

    def on_pose(message):
        ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        p, q = message.pose.pose.position, message.pose.pose.orientation
        receipt = time.monotonic_ns()
        require(all(math.isfinite(v) for v in [p.x, p.y, q.z, q.w] + list(message.pose.covariance)), 'Nonfinite pose')
        require(ns in source_by_ref, f'AMCL pose stamp not matched to physical scan reference: {ns}')
        source = source_by_ref[ns]
        if not received:
            report['first_native_pose_timing'] = {
                'receipt_monotonic_ns': receipt,
                'after_first_scan_publish_wall_s':
                    (receipt - report['first_scan_publish_monotonic_ns']) / 1e9,
                'after_estimator_start_wall_s':
                    (receipt - report['estimator_start_monotonic_ns']) / 1e9,
                'after_global_request_wall_s':
                    (receipt - report['global_request_monotonic_ns']) / 1e9,
                'reference_after_first_input_source_s':
                    (ns - scan_sources[0]['reference_stamp_ns']) / 1e9,
                'scope': 'First received native pose, correctness assessed only after trial closure; '
                         'includes pending acquisition; estimator-start interval includes harness setup waits',
            }
        received.append({'stamp_ns': ns, 'reference_stamp_ns': ns, 'raw_stamp_ns': source['raw_stamp_ns'],
                         'source_scan_id': source['source_scan_id'], 'source_scan_index': source['source_scan_index'], 'receipt_monotonic_ns': receipt,
                         'pose': [p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)],
                         'covariance': list(message.pose.covariance),
                         'publish_to_receipt_ms': (receipt - sent[ns]) / 1e6 if ns in sent else None})
        with (args.output / 'trajectory.tum').open('a') as stream:
            sec, nano = divmod(ns, 1000000000)
            stream.write(f'{sec}.{nano:09d} {p.x:.15g} {p.y:.15g} 0 0 0 {q.z:.15g} {q.w:.15g}\n')

    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_pose, 10000)
    from diagnostic_msgs.msg import DiagnosticArray
    def on_contract(message):
        with (args.output / 'estimate-contract.jsonl').open('a') as stream:
            stream.write(json.dumps({'receipt_monotonic_ns': time.monotonic_ns(),
                'statuses': [{'name': s.name, 'fields': {v.key: v.value for v in s.values}}
                             for s in message.status]}) + '\n')
    node.create_subscription(DiagnosticArray, '/amcl_estimate_contract', on_contract, 10000)

    def spin(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(.005, max(0., until-time.monotonic())))

    def stamp(ns):
        return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

    def state(code):
        request = ChangeState.Request()
        request.transition.id = code
        future = lifecycle.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.)
        require(future.done() and future.result() and future.result().success, f'Lifecycle {code} failed')

    executable = str(args.executable)
    process = None
    try:
        with (args.output / 'node.log').open('w') as log:
            report['estimator_start_unix_s'] = time.time()
            save()
            report['estimator_start_monotonic_ns'] = time.monotonic_ns()
            process = subprocess.Popen([executable, '--ros-args', '--params-file', str(parameter_path)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            require(lifecycle.wait_for_service(timeout_sec=20.), 'AMCL service unavailable')
            clock_pub.publish(Clock(clock=stamp(records[0][1])))
            state(Transition.TRANSITION_CONFIGURE)
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width, grid.info.height = pixels.shape[1], pixels.shape[0]
            grid.info.resolution = float(metadata['resolution'])
            grid.info.origin.position.x, grid.info.origin.position.y = metadata['origin'][:2]
            grid.info.origin.orientation.w = 1.
            grid.data = np.flipud(cells).ravel().tolist()
            map_pub.publish(grid)
            for index,(topic, _, data) in enumerate(records):
                if topic == '/tf_static':
                    publish_record(index)
            spin(1.)
            state(Transition.TRANSITION_ACTIVATE)
            require(global_localization.wait_for_service(timeout_sec=10.), 'Global localization service absent')
            report['global_request_monotonic_ns'] = time.monotonic_ns()
            global_future = global_localization.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, global_future, timeout_sec=30.)
            require(global_future.done() and global_future.result() is not None,
                    'Uniform free-map global initialization failed')
            report['global_initialization'] = dict(service='/reinitialize_global_localization',
                succeeded=True, pose_prior_published=False, ground_truth_read=False,
                particle_policy=('finite geometric search followed by engineering capture gate and Gaussian tracking seed'
                    if parameters.get('global_acquisition_enabled', False)
                    else 'unchanged configured AMCL particle counts and uniformPoseGenerator'))
            spin(.3)
            # Freeze actual loaded library provenance after lifecycle activation.
            paths = sorted({line.split()[-1] for line in Path(f'/proc/{process.pid}/maps').read_text().splitlines()
                            if '/' in line and 'amcl' in line.lower()})
            report['loaded_amcl_paths_before_prefix_check'] = paths
            save()
            require(paths and all(Path(path).is_file() and Path(path).resolve().is_relative_to(
                args.expected_prefix.resolve()) for path in paths), 'Wrong AMCL library')
            report['loaded_libraries'] = {path: sha(path) for path in paths}
            report['executable_sha256'] = sha(executable)
            parameter_client = node.create_client(GetParameters, '/amcl/get_parameters')
            request = GetParameters.Request()
            request.names = sorted(parameters)
            future = parameter_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
            require(future.done() and future.result(), 'Effective parameter query failed')
            from rclpy.parameter import parameter_value_to_python
            report['effective_parameters'] = dict(zip(request.names, [parameter_value_to_python(v) for v in future.result().values]))
            save()
            # Wait for first-use distance-field construction and the first estimate
            # with complete odometry support to prevent unequal CPU/CUDA startup drops.
            first_index = next(i for i,row in enumerate(records) if row[0]=='/scan')
            first_scan = deserialize_message(records[first_index][2], LaserScan)
            first_ns = source_by_index[first_index]['reference_stamp_ns']
            prepublished = set()
            pre_roll_end = source_by_index[first_index]['last_beam_stamp_ns'] + 100000000
            # The bag advances original TF records by 200 ms. Preserve all
            # support needed for the final beam, without changing any CDR.
            for index,(topic,ns,data) in enumerate(records):
                if ns > pre_roll_end:
                    break
                if topic == '/tf':
                    publish_record(index)
                    prepublished.add(index)
            spin(.05)
            sent[first_ns] = time.monotonic_ns()
            report['first_scan_publish_monotonic_ns'] = sent[first_ns]
            publish_record(first_index)
            prepublished.add(first_index)
            startup_wait_started = time.monotonic()
            until = time.monotonic()+30.
            def first_processed():
                if any(row['stamp_ns'] == first_ns for row in received):
                    return True
                if not args.acquisition_aware:
                    return False
                # A pending acquisition is an acknowledgement, not localization.
                # No synthetic pose and no additional scan are sent while waiting.
                text = (args.output / 'node.log').read_text()
                raw = source_by_index[first_index]['raw_stamp_ns']
                return any('AMCL_ACQUISITION ' in line and
                           f'raw_stamp_ns={raw} ' in line for line in text.splitlines())
            while not first_processed() and time.monotonic()<until:
                spin(.01)
            report['startup_callback_wait_wall_s'] = time.monotonic() - startup_wait_started
            require(first_processed(), 'First scan callback acknowledgement failed')
            mapped = sorted({line.split()[-1] for line in Path(f'/proc/{process.pid}/maps').read_text().splitlines() if '/' in line})
            report['loaded_computation_libraries_after_first_scan'] = {path: sha(path) for path in mapped
                if Path(path).is_file() and any(token in path.lower() for token in ('amcl', 'cuda', 'nvidia', 'scan_geometry'))}
            report['startup_pre_roll_record_indices'] = sorted(prepublished)
            report['startup_policy'] = 'First original scan plus original TF bracket, wait for first pose before 2x paced continuation; no scan duplication or synthetic sensor samples'
            if args.acquisition_aware:
                report['startup_policy'] = ('First original scan plus original TF bracket; wait for first pose OR explicit pending acquisition acknowledgement, '
                                            'then unchanged paced continuation without acquisition-dependent pauses; no synthetic poses or samples')
            start = time.monotonic()
            begin = records[0][1]
            scanned = 1
            lag_ms = []
            for index,(topic, ns, data) in enumerate(records):
                target = start + (ns-begin)/1e9/args.rate
                spin(max(0., target-time.monotonic()))
                require(process.poll() is None, 'AMCL exited during replay')
                clock_pub.publish(Clock(clock=stamp(ns)))
                if index in prepublished:
                    continue
                if topic == '/scan':
                    scan = deserialize_message(data, LaserScan)
                    sns = source_by_index[index]['reference_stamp_ns']
                    sent[sns] = time.monotonic_ns()
                    lag_ms.append(max(0., time.monotonic()-target)*1000.)
                    scanned += 1
                publish_record(index)
                if scanned and scanned % 500 == 0 and topic == '/scan':
                    report.update(published_scans=scanned, received_poses=len(received), elapsed_replay_s=time.monotonic()-start)
                    save()
                    print(json.dumps({k: report[k] for k in ['backend','published_scans','received_poses','elapsed_replay_s']}), flush=True)
            # Advance only the replay clock while draining end-of-bag TF timer
            # requests. No sensor/odometry samples are fabricated.
            for tick in range(1, 61):
                clock_pub.publish(Clock(clock=stamp(records[-1][1] + tick * 50000000)))
                spin(.01)
            spin(max(.5, parameters.get('scan_pending_timeout_sec', 2.) + .5))
            report['replay_wall_s'] = time.monotonic()-start
            report['published_scans'] = scanned
            report['publisher_lag_ms'] = {'p50': float(np.percentile(lag_ms,50)), 'p95':float(np.percentile(lag_ms,95)), 'max': max(lag_ms)}
            state(Transition.TRANSITION_DEACTIVATE)
            state(Transition.TRANSITION_CLEANUP)
            report['localization_output_available'] = bool(received)
            if not args.acquisition_aware:
                require(received, 'No poses received')
            report['status'] = 'completed'
    except BaseException as error:
        report.update(status='failed', error=repr(error))
        raise
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                report['node_returncode'] = process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                report['node_returncode'] = process.wait(timeout=5.)
                report['forced_cleanup'] = True
        report['received_poses'] = len(received)
        report['unique_pose_stamps'] = len({row['stamp_ns'] for row in received})
        report['unique_published_stamps'] = len(sent)
        report['actual_publication_order_sha256'] = actual_publication_hash.hexdigest()
        report['actual_publication_record_indices'] = actual_publications
        report['source_record_hash_scope'] = 'Frozen original record order; actual startup preload and primed static replay are separately hashed in actual_publication_order_sha256'
        report['map_artifact_sha256_after'] = {str(p): sha(p) for p in map_artifacts}
        report['map_artifacts_unchanged'] = report['map_artifact_sha256_before'] == report['map_artifact_sha256_after']
        report['source_scan_ids_with_pose'] = sorted({row['source_scan_id'] for row in received})
        report['source_scan_indices_with_pose'] = sorted({row['source_scan_index'] for row in received})
        report['support_valid_source_ids_without_native_motion_pose'] = sorted(
            set(s['source_scan_id'] for s in scan_sources if s['support_valid']) - set(report['source_scan_ids_with_pose']))
        report['source_scans_without_native_motion_pose'] = sorted(set(s['source_scan_id'] for s in scan_sources) - set(report['source_scan_ids_with_pose']))
        report['coverage_contract'] = 'Native motion-selected AMCL poses; no forced no-motion updates or interpolation; every scan published exactly once'
        report['finished_unix_s'] = time.time()
        (args.output / 'poses.json').write_text(json.dumps(received) + '\n')
        logs = (args.output / 'node.log').read_text()
        counters = [dict(re.findall(r'(\w+)=([^\s,]+)', line)) for line in logs.splitlines() if 'AMCL_OBSERVATION_STATS' in line]
        unique = {int(row['cpu_updates'])+int(row['cuda_updates']):row for row in counters}
        report['observation_samples'] = len(unique)
        report['observation_final'] = counters[-1] if counters else None
        report['scan_rejections'] = [line for line in logs.splitlines() if 'scan rejected' in line or 'observation rejected' in line]
        report['filter_diagnostics'] = [dict(re.findall(r'(\w+)=([^\s,]+)', line)) for line in logs.splitlines() if 'AMCL filter ' in line]
        report['acquisition_diagnostics'] = [dict(re.findall(r'(\w+)=([^\s,]+)', line)) for line in logs.splitlines() if 'AMCL_ACQUISITION ' in line]
        report['acquisition_accounting_scope'] = ('Pending/rejected acquisition inputs remain in the full input denominator; '
                                                  'a completed process with no poses is an unresolved localization attempt, not a success')
        if unique:
            timings = [float(row['last_us'])/1000. for row in unique.values()]
            report['observation_ms'] = {key:float(np.percentile(timings,q)) for key,q in [('p50',50),('p95',95),('p99',99),('max',100)]}
        timings = [row['publish_to_receipt_ms'] for row in received if row['publish_to_receipt_ms'] is not None]
        if timings:
            report['scan_publish_to_pose_ms'] = {key:float(np.percentile(timings,q)) for key,q in [('p50',50),('p95',95),('p99',99),('max',100)]}
        save()
        node.destroy_node()
        rclpy.shutdown()
    final_errors = []
    def final_require(value, message):
        if not value:
            final_errors.append(message)
    final_require(report['node_returncode'] == 0, 'Abnormal AMCL exit')
    final_require(report['map_artifacts_unchanged'], 'Frozen map artifact changed during localization')
    rejected_stamps = []
    for line in report['scan_rejections']:
        match = re.search(r'(?:source|raw)_stamp_ns=(\d+)', line)
        if match:
            rejected_stamps.append(int(match.group(1)))
    report['actual_rejected_raw_stamps'] = rejected_stamps
    report['unexpected_rejected_raw_stamps'] = sorted(set(rejected_stamps) - set(report['expected_geometry_unsupported_raw_stamps']))
    final_require(not report['unexpected_rejected_raw_stamps'], 'AMCL unexpectedly rejected support-valid input scans')
    final_require(set(rejected_stamps) == set(report['expected_geometry_unsupported_raw_stamps']), 'Native rejection set differs from independent full-beam TF support audit')
    final_require(scanned == len(scan_sources), 'Not all scans published')
    final_require(len({row['source_scan_id'] for row in received}) == len(received), 'Duplicate native source poses')
    if args.backend == 'cuda':
        if not (args.acquisition_aware and not received):
            final_require(report['observation_final'] and int(report['observation_final']['cuda_updates']) > 0, 'CUDA did not execute')
        if report['observation_final']:
            final_require(int(report['observation_final']['failures']) == 0, 'CUDA observation failure')
            final_require(int(report['observation_final']['cpu_updates']) == int(report['observation_final']['boundary_fallbacks']), 'CPU observation used beyond explicit deterministic numeric boundary guard')
        report['cuda_execution_verified'] = bool(report['observation_final'] and int(report['observation_final']['cuda_updates']) > 0)
        report['cuda_boundary_guard_scope'] = 'observation_backend=cuda; explicit numeric boundary guard may rerun an observation on CPU; counts are disclosed separately, no auto backend allowed'
    report['validation_errors'] = final_errors
    report['status'] = 'failed' if final_errors else 'completed'
    save()
    require(not final_errors, '; '.join(final_errors))
    print(json.dumps({k: report.get(k) for k in ['status','backend','published_scans','received_poses','observation_ms','scan_publish_to_pose_ms','observation_final']}), flush=True)


if __name__ == '__main__':
    main()
