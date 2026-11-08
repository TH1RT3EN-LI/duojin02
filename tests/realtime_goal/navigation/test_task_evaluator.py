import copy
import importlib.util
import math
from pathlib import Path
import unittest

MODULE = Path(__file__).resolve().parents[3] / 'src/duojin02_bringup/duojin02_bringup/task_goal_evaluator.py'
spec = importlib.util.spec_from_file_location('task_goal_evaluator', MODULE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

PROTOCOL = dict(frozen_at_ns=0, registration=dict(id='frozen-fixture', map_to_truth=[1,2,math.pi/2],
                frozen_at_ns=0, evidence='synthetic_only'), limits=dict(xy_m=.02,
                yaw_rad=math.pi/180, linear_mps=.01, angular_rps=.02,
                dwell_s=.25, max_sample_gap_s=.15))
TASK = dict(id='a', goal_map=[.25,0,0], start_ns=10**9, deadline_ns=3*10**9,
            status='succeeded', completion_ns=2*10**9)

def samples(point=None, twist=None):
    return [dict(task_id='a', stamp_ns=10**9+i*50_000_000,
            truth_base=point or [1,2.25,math.pi/2], truth_twist=twist or [0,0,0]) for i in range(21)]

class TaskEvaluatorTests(unittest.TestCase):
    def test_joint_success_and_all_failure_denominator(self):
        failed = dict(TASK, id='timeout', status='timed_out', completion_ns=None)
        result = m.evaluate(PROTOCOL, [TASK, failed], samples())
        self.assertEqual((result['successes'], result['total_tasks']), (1,2))
        self.assertEqual(result['joint_success_fraction'], .5)
        self.assertEqual(result['tasks'][1]['reason'], 'action_timed_out')

    def test_yaw_speed_and_drift_each_prevent_success(self):
        for rows in (samples([1,2.25,math.pi/2+.1]), samples(twist=[.02,0,0])):
            self.assertFalse(m.evaluate(PROTOCOL,[TASK],rows)['tasks'][0]['passed'])
        rows = samples()
        rows[-1]['truth_base'][0] = 1.1
        self.assertFalse(m.evaluate(PROTOCOL,[TASK],rows)['tasks'][0]['passed'])

    def test_gap_resets_dwell_and_missing_truth_fails(self):
        rows = samples()[:3]+samples()[-3:]
        result = m.evaluate(PROTOCOL,[TASK],rows)['tasks'][0]
        self.assertFalse(result['passed'])
        self.assertEqual(result['sample_gap_count'], 1)
        self.assertFalse(m.evaluate(PROTOCOL,[TASK],[])['tasks'][0]['passed'])
        rows = samples()
        rows[-1]['truth_base'] = None
        self.assertFalse(m.evaluate(PROTOCOL,[TASK],rows)['tasks'][0]['passed'])

    def test_missing_completion_truth_cannot_impute_stopped_state(self):
        # The last known truth is settled; unknown motion in the following 50ms
        # must not inherit a pass merely because it is within max_sample_gap_s.
        result = m.evaluate(PROTOCOL,[TASK],samples()[:-1])['tasks'][0]
        self.assertFalse(result['passed'])
        self.assertEqual(result['reason'],'missing_exact_truth_at_completion')

    def test_registration_cannot_be_fit_after_start_or_per_task(self):
        protocol = copy.deepcopy(PROTOCOL)
        protocol['registration']['frozen_at_ns'] = TASK['start_ns']
        with self.assertRaises(ValueError):
            m.evaluate(protocol,[TASK],samples())
        with self.assertRaises(ValueError):
            m.evaluate(PROTOCOL,[TASK],list(reversed(samples())))

    def test_execution_latency_needs_identified_same_clock_pairs(self):
        task = dict(TASK, command_application_pairs=[dict(command_id='1',issued_ns=10,
                    applied_ns=20,clock_scope='same_host_steady')])
        latency = m.evaluate(PROTOCOL,[task],samples())['tasks'][0]['actuator_latency']
        self.assertEqual(latency['count'],1)
        self.assertEqual(latency['p95_s'],1e-8)
        task['command_application_pairs'][0]['applied_ns']=0
        with self.assertRaises(ValueError):
            m.evaluate(PROTOCOL,[task],samples())

    def test_workpoint_yaw_lever_arm_and_overshoot(self):
        task = dict(TASK, goal_map=[0,0,0], base_to_task=[.5,0,0])
        rows = samples(point=[1,1.5,math.pi/2], twist=[0,0,.02])
        result = m.evaluate(PROTOCOL,[task],rows)['tasks'][0]
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['terminal']['task_point_speed_mps'], .01)
        task = dict(TASK, goal_map=[0,0,0])
        rows = samples(point=[1,1.9,math.pi/2])
        rows[-1]['truth_base'] = [1,2.03,math.pi/2]
        self.assertAlmostEqual(m.evaluate(PROTOCOL,[task],rows)['tasks'][0]['overshoot_m'], .03)

if __name__ == '__main__':
    unittest.main()
