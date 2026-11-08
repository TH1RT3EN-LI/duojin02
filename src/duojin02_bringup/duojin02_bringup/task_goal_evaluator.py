"""Frozen-registration task evaluator; no per-task alignment or target fitting.

Input protocol: registration {id,map_to_truth:[x,y,yaw],frozen_at_ns,evidence},
limits {xy_m,yaw_rad,linear_mps,angular_rps,dwell_s,max_sample_gap_s}.
Tasks contain id, goal_map [x,y,yaw], start_ns, deadline_ns, status,
completion_ns (nullable), and optional base_to_task [x,y,yaw].
JSONL samples contain task_id, stamp_ns, truth_base [x,y,yaw], truth_twist
[vx,vy,wz]. Missing truth is a failure in the full task denominator.
"""
import argparse
import json
import math
from pathlib import Path


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return [a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], wrap(a[2]+b[2])]


def finite_vector(value, size=3):
    return isinstance(value, (list, tuple)) and len(value) == size and all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value)


def evaluate(protocol, tasks, samples):
    registration = protocol['registration']
    transform = registration['map_to_truth']
    if not finite_vector(transform) or not registration.get('id'):
        raise ValueError('one finite, identified registration is required')
    limits = protocol['limits']
    for key in ('xy_m', 'yaw_rad', 'linear_mps', 'angular_rps', 'dwell_s', 'max_sample_gap_s'):
        if not math.isfinite(limits[key]) or limits[key] <= 0:
            raise ValueError('evaluation limits must be finite positive')
    if len({t['id'] for t in tasks}) != len(tasks):
        raise ValueError('duplicate task id')
    if any(registration['frozen_at_ns'] >= t['start_ns'] for t in tasks):
        raise ValueError('registration must be frozen before every task starts')
    if protocol.get('frozen_at_ns', registration['frozen_at_ns']) >= min(
            (t['start_ns'] for t in tasks), default=math.inf):
        raise ValueError('evaluation protocol must be frozen before execution')
    grouped = {t['id']: [] for t in tasks}
    unknown_task_samples = 0
    for sample in samples:
        if sample['task_id'] in grouped:
            grouped[sample['task_id']].append(sample)
        else:
            unknown_task_samples += 1
    results = []
    for task in tasks:
        if not finite_vector(task['goal_map']) or not finite_vector(task.get('base_to_task', [0,0,0])):
            raise ValueError('task coordinates must be finite SE2')
        if task['deadline_ns'] <= task['start_ns']:
            raise ValueError('task deadline must follow start')
        goal = compose(transform, task['goal_map'])
        rows = grouped[task['id']]
        stamps = [r['stamp_ns'] for r in rows]
        if stamps != sorted(set(stamps)):
            raise ValueError('task truth stamps must be strictly increasing; no post-hoc reordering')
        completion = task.get('completion_ns')
        end = min(task['deadline_ns'], completion if completion is not None else task['deadline_ns'])
        accepted = []
        invalid = 0
        gap_count = 0
        dwell_begin = None
        first_settled = None
        latest_joint_dwell = False
        previous_stamp = None
        initial_position = None
        max_overshoot = 0.
        terminal = None
        for row in rows:
            stamp = row['stamp_ns']
            if stamp < task['start_ns'] or stamp > end:
                continue
            if not finite_vector(row.get('truth_base')) or not finite_vector(row.get('truth_twist')):
                invalid += 1
                dwell_begin = None
                latest_joint_dwell = False
                continue
            point = compose(row['truth_base'], task.get('base_to_task', [0,0,0]))
            vx, vy, wz = row['truth_twist']
            # Twist supplied at the base reference; rigid task-point speed includes
            # the yaw-rate lever arm, independent of the point's orientation.
            lx, ly, _ = task.get('base_to_task', [0,0,0])
            linear = math.hypot(vx-wz*ly, vy+wz*lx)
            xy = math.hypot(point[0]-goal[0], point[1]-goal[1])
            yaw = abs(wrap(point[2]-goal[2]))
            joint = xy <= limits['xy_m'] and yaw <= limits['yaw_rad'] and (
                linear <= limits['linear_mps'] and abs(wz) <= limits['angular_rps'])
            if previous_stamp is not None and stamp-previous_stamp > limits['max_sample_gap_s']*1e9:
                dwell_begin = None
                gap_count += 1
            previous_stamp = stamp
            if not joint:
                dwell_begin = None
            elif dwell_begin is None:
                dwell_begin = stamp
            latest_joint_dwell = joint and stamp-dwell_begin >= limits['dwell_s']*1e9
            if latest_joint_dwell and first_settled is None:
                first_settled = stamp
            terminal = dict(stamp_ns=stamp, xy_m=xy, yaw_rad=yaw,
                            task_point_speed_mps=linear, angular_speed_rps=abs(wz))
            accepted.append(terminal)
            if initial_position is None:
                initial_position = point[:2]
            approach = [goal[i]-initial_position[i] for i in range(2)]
            length = math.hypot(*approach)
            if length > 0:
                max_overshoot = max(max_overshoot, sum((point[i]-goal[i])*approach[i]/length for i in range(2)))
        reason = 'joint_tolerance_or_stopped_dwell_not_met'
        passed = False
        if task['status'] != 'succeeded':
            reason = 'action_' + task['status']
        elif completion is None:
            reason = 'missing_completion_time'
        elif completion > task['deadline_ns']:
            reason = 'timeout'
        elif completion < task['start_ns']:
            reason = 'invalid_completion_time'
        elif terminal is None:
            reason = 'missing_independent_truth'
        elif completion != terminal['stamp_ns']:
            reason = 'missing_exact_truth_at_completion'
        elif latest_joint_dwell:
            passed, reason = True, 'joint_xy_yaw_stopped_dwell'
        pairs = task.get('command_application_pairs', [])
        if len({pair['command_id'] for pair in pairs}) != len(pairs):
            raise ValueError('duplicate command application pair')
        latencies = []
        for pair in pairs:
            if not pair.get('clock_scope') or pair['applied_ns'] < pair['issued_ns']:
                raise ValueError('command timing requires a common clock and causal application')
            latencies.append((pair['applied_ns']-pair['issued_ns'])/1e9)
        latencies.sort()
        latency = dict(status='unknown_without_identified_command_application_pairs')
        if latencies:
            latency = dict(status='recorded_pairs_only', count=len(latencies),
                           minimum_s=latencies[0], maximum_s=latencies[-1],
                           p50_s=latencies[math.ceil(.50*len(latencies))-1],
                           p95_s=latencies[math.ceil(.95*len(latencies))-1],
                           percentile_method='nearest_rank',
                           clock_scopes=sorted({pair['clock_scope'] for pair in pairs}))
        results.append(dict(task_id=task['id'], passed=passed, reason=reason,
                            requested_goal_map=list(task['goal_map']), fixed_goal_truth=goal,
                            registration_id=registration['id'], terminal=terminal,
                            valid_truth_samples=len(accepted), invalid_truth_samples=invalid,
                            sample_gap_count=gap_count, first_joint_settled_ns=first_settled,
                            settle_time_s=None if first_settled is None else (first_settled-task['start_ns'])/1e9,
                            elapsed_s=None if completion is None else (completion-task['start_ns'])/1e9,
                            overshoot_m=max_overshoot if accepted else None,
                            actuator_latency=latency))
    count = len(tasks)
    success = sum(r['passed'] for r in results)
    return dict(schema='frozen_task_evaluation.v1', evidence=registration.get('evidence', 'unknown'),
                physical_calibration=registration.get('physical_calibration', 'unknown'),
                alignment='single_registration_frozen_before_all_tasks_no_refitting',
                total_tasks=count, successes=success, failures=count-success,
                joint_success_fraction=None if not count else success/count,
                unknown_task_samples=unknown_task_samples, tasks=results,
                confidence_claim='none_task_sampling_and_external_calibration_not_established')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--tasks', type=Path, required=True)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    samples = [json.loads(line) for line in args.samples.read_text().splitlines() if line.strip()]
    result = evaluate(json.loads(args.protocol.read_text()), json.loads(args.tasks.read_text()), samples)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
