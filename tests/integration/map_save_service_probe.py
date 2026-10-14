"""Real ROS service and executable regression checks using temporary fake map services."""
import os
import json
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import GetParameters, ListParameters
from nav_msgs.srv import GetMap
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_srvs.srv import Trigger
from duojin02_slam_tools.snapshots import artifact_path, validate_map
from duojin02_slam_tools.offline import await_response


def run():
    os.environ['ROS_DOMAIN_ID'] = '92'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = Node('slam_toolbox', start_parameter_services=False)
    mode = ['success']
    def refresh(request, response):
        if mode[0] == 'shared_budget':
            time.sleep(0.15)
        response.map.info.width = response.map.info.height = 1
        return response
    def raster(request, response):
        if mode[0] == 'timeout':
            time.sleep(1.2)
        if mode[0] == 'shared_budget':
            time.sleep(0.15)
        if mode[0] == 'raster_failure':
            response.result = SaveMap.Response.RESULT_UNDEFINED_FAILURE
            return response
        prefix = Path(request.name.data)
        if prefix.parent.exists():
            artifact_path(prefix, '.yaml').write_text(
                f'image: {prefix.name}.pgm\nmode: trinary\nresolution: 0.03\n'
                'origin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            artifact_path(prefix, '.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
        response.result = SaveMap.Response.RESULT_SUCCESS
        return response
    def graph(request, response):
        if mode[0] == 'shared_budget':
            time.sleep(0.3)
        if mode[0] == 'graph_failure':
            response.result = SerializePoseGraph.Response.RESULT_FAILED_TO_WRITE_FILE
            return response
        prefix = Path(request.filename)
        if prefix.parent.exists():
            artifact_path(prefix, '.posegraph').write_bytes(b'graph')
            artifact_path(prefix, '.data').write_bytes(b'scans')
        response.result = SerializePoseGraph.Response.RESULT_SUCCESS
        return response
    node.create_service(SaveMap, '/slam_toolbox/save_map', raster)
    node.create_service(GetMap, '/slam_toolbox/dynamic_map', refresh)
    node.create_service(SerializePoseGraph, '/slam_toolbox/serialize_map', graph)
    def parameter_names(request, response):
        response.result.names = ['map_start_pose']
        return response
    def parameter_values(request, response):
        if mode[0] == 'metadata_timeout':
            time.sleep(0.8)
        if mode[0] != 'metadata_incomplete':
            response.values = [Parameter('map_start_pose', value=[1.0, 2.0, 0.5]).to_parameter_msg().value]
        return response
    node.create_service(ListParameters, '/slam_toolbox/list_parameters', parameter_names)
    node.create_service(GetParameters, '/slam_toolbox/get_parameters', parameter_values)
    def delayed_status(request, response):
        time.sleep(2.2)
        response.success = True
        return response
    node.create_service(Trigger, '/phase_budget_status', delayed_status)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            def save(name, timeout='2.0'):
                return subprocess.run(['ros2', 'run', 'duojin02_slam_tools', 'save_map_client_node',
                                       '--ros-args', '-p', f'output_dir:={output}', '-p', f'map_name:={name}',
                                       '-p', f'save_timeout:={timeout}'],
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=15)
            success = save('room.v2')
            assert success.returncode == 0, success.stdout
            validate_map(output / 'room.v2.yaml')
            snapshot = json.loads((output / 'room.v2.snapshot.json').read_text())
            assert snapshot['provenance']['slam_parameters']['map_start_pose'] == [1.0, 2.0, 0.5]
            assert save('room.v2').returncode != 0, 'existing map overwritten'
            mode[0] = 'graph_failure'
            failure = save('failed_graph')
            assert failure.returncode != 0 and not (output / 'failed_graph.yaml').exists(), failure.stdout
            mode[0] = 'raster_failure'
            assert save('failed_raster').returncode != 0
            mode[0] = 'timeout'
            began = time.monotonic()
            expired = save('expired', timeout='0.2')
            assert expired.returncode != 0 and not (output / 'expired.yaml').exists(), expired.stdout
            assert time.monotonic() - began < 5
            time.sleep(1.3)  # Allow the cancelled service request to finish before shutdown.
            mode[0] = 'shared_budget'
            shared = save('shared_budget', timeout='0.4')
            assert shared.returncode != 0 and not (output / 'shared_budget.yaml').exists(), shared.stdout
            time.sleep(0.4)
            mode[0] = 'metadata_incomplete'
            partial = save('metadata_incomplete')
            assert partial.returncode == 0, partial.stdout
            validate_map(output / 'metadata_incomplete.yaml')
            provenance = json.loads((output / 'metadata_incomplete.snapshot.json').read_text())['provenance']
            assert provenance['algorithm'] == 'slam_toolbox' and provenance['version']
            assert provenance['backend_build'] and 'incomplete' in provenance['warnings'][0]
            assert 'slam_parameters' not in provenance, 'partial parameter list presented as complete'
            mode[0] = 'metadata_timeout'
            warned = save('metadata_timeout', timeout='0.4')
            assert warned.returncode == 0, warned.stdout
            validate_map(output / 'metadata_timeout.yaml')
            provenance = json.loads((output / 'metadata_timeout.snapshot.json').read_text())['provenance']
            assert provenance['backend_build'] and provenance['warnings']
            time.sleep(0.9)
            control = Node('offline_phase_budget_probe')
            try:
                client = control.create_client(Trigger, '/phase_budget_status')
                assert client.wait_for_service(timeout_sec=3)
                began = time.monotonic()
                response = await_response(control, client.call_async(Trigger.Request()),
                                          began + 3.0, 'delayed status')
                assert response.success and time.monotonic() - began > 2.0
                stalled = Future()
                began = time.monotonic()
                try:
                    await_response(control, stalled, began + 0.2, 'stalled status')
                    raise AssertionError('stalled status ignored the phase budget')
                except TimeoutError:
                    assert stalled.cancelled() and time.monotonic() - began < 0.6
                dead = Future()
                try:
                    await_response(control, dead, time.monotonic() + 3, 'dead process',
                                   (type('Stopped', (), {'poll': lambda self: 1})(),))
                    raise AssertionError('dead process was ignored')
                except RuntimeError:
                    assert dead.cancelled()
            finally:
                control.destroy_node()
            print('map service success, existing-name refusal, graph/raster failures, one save budget, '
                  'metadata timeout/incomplete warning, delayed/stalled/dead-process phase budgets: passed')
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
