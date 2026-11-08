import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import math
import unittest
from unittest.mock import patch
from tempfile import TemporaryDirectory

MODULE = Path(__file__).resolve().parents[3] / 'src/duojin02_bringup/duojin02_bringup/task_goal_capture.py'
spec = importlib.util.spec_from_file_location('task_goal_capture', MODULE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def pose(x=math.nextafter(1., 2.)):
    return S(position=S(x=x, y=-.12345678912345678, z=0.),
             orientation=S(x=0., y=0., z=0., w=1.))


class ContractTests(unittest.TestCase):
    def test_default_recording_creates_unique_private_jsonl_files(self):
        import os
        with TemporaryDirectory() as directory, patch.dict(os.environ, {"ROS_LOG_DIR": directory}):
            with m.open_record() as first, m.open_record() as second:
                self.assertNotEqual(first.name, second.name)
                self.assertEqual(Path(first.name).parent, Path(directory))
                self.assertEqual(os.stat(first.name).st_mode & 0o777, 0o600)
                first.write('{"kind":"goal"}\n')
            self.assertEqual(Path(first.name).read_text(), '{"kind":"goal"}\n')

    def test_explicit_path_never_overwrites_and_recording_can_be_disabled(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            path.write_text("keep")
            with self.assertRaises(FileExistsError):
                m.open_record(path)
            self.assertEqual(path.read_text(), "keep")
            self.assertIsNone(m.open_record(disabled=True))

    def test_original_binary64_roundtrip_and_unknown(self):
        request = S(header=S(frame_id='map', stamp=S(sec=3, nanosec=8)), pose=pose())
        fields = m.original_goal_fields(request)
        self.assertEqual(float(fields['original_x']), request.pose.position.x)
        self.assertNotEqual(float(fields['original_x']), 1.)
        self.assertEqual(fields['original_stamp_ns'], '3000000008')
        self.assertEqual(fields['physical_calibration'], 'unknown')
        self.assertEqual(fields['map_identity'], 'unknown')
        self.assertEqual(fields['conversion_source'], 'original_goal_pose_no_coordinate_conversion')

    def test_map_identity_changes_geometry_but_not_stamp(self):
        grid = S(header=S(frame_id='map', stamp=1),
                 info=S(width=2, height=1, resolution=.05, origin=pose(0.)), data=[0, 100])
        initial = m.map_identity(grid)
        self.assertEqual(initial, "occupancy_fnv1a64:526bf651ce37d530")
        grid.header.stamp = 99
        self.assertEqual(initial, m.map_identity(grid))
        grid.info.origin.position.x = .001
        self.assertNotEqual(initial, m.map_identity(grid))
        grid.info.origin.position.x = 0.
        grid.data[1] = -1
        self.assertNotEqual(initial, m.map_identity(grid))

    def test_nonfinite_measurement_is_explicit_and_json_serializable(self):
        import json
        record = m.json_safe_fields(dict(vx=float('nan'), nested=[float('inf'),1.]))
        self.assertIsNone(record['vx'])
        self.assertEqual(record['nonfinite_input_fields'], ['record.vx','record.nested.0'])
        json.dumps(record, allow_nan=False)

    def test_invalid_request_is_not_published_as_valid(self):
        request = S(header=S(frame_id='map', stamp=S(sec=0, nanosec=0)), pose=pose(float('nan')))
        with self.assertRaises(ValueError):
            m.original_goal_fields(request)
        request.pose = pose()
        request.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            m.original_goal_fields(request)


if __name__ == '__main__':
    unittest.main()
