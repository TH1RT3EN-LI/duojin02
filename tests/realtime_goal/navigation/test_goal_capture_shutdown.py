"""Exercise production main cleanup using fake ROS node plumbing, no DDS or motion."""
import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch
from unittest.mock import Mock

if os.environ.get('DUOJIN_CAPTURE_UNDER_TEST'):
    spec = importlib.util.spec_from_file_location('capture_shutdown_under_test', Path(os.environ['DUOJIN_CAPTURE_UNDER_TEST']))
    capture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(capture)
else:
    from duojin02_bringup import task_goal_capture as capture


class FakeNode:
    destroyed = False

    def __init__(self, *_args, **_kwargs):
        FakeNode.destroyed = False

    def create_publisher(self, *_args, **_kwargs):
        return object()

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def get_logger(self):
        return Mock()

    def destroy_node(self):
        FakeNode.destroyed = True


class ShutdownTest(unittest.TestCase):
    def run_shutdown(self, exception, context_ok):
        with patch('rclpy.node.Node', FakeNode), patch('rclpy.init'), \
             patch('rclpy.spin', side_effect=exception), patch('rclpy.ok', return_value=context_ok), \
             patch('rclpy.shutdown') as shutdown:
            capture.main([])
            self.assertTrue(FakeNode.destroyed)
            self.assertEqual(shutdown.call_count, int(context_ok))

    def test_sigint_context_already_shutdown(self):
        self.run_shutdown(KeyboardInterrupt(), False)

    def test_external_shutdown_context_already_shutdown(self):
        from rclpy.executors import ExternalShutdownException
        self.run_shutdown(ExternalShutdownException(), False)

    def test_normal_spin_return_closes_live_context_once(self):
        self.run_shutdown(None, True)


if __name__ == '__main__':
    unittest.main()
