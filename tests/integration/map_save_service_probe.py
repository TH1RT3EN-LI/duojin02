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
from nav_msgs.srv import GetMap
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from duojin02_slam_tools.snapshots import artifact_path, validate_map


def run():
    os.environ['ROS_DOMAIN_ID'] = '92'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = Node('slam_toolbox')
    node.declare_parameter('map_start_pose', [1.0, 2.0, 0.5])
    mode = ['success']
    def refresh(request, response):
        response.map.info.width = response.map.info.height = 1
        return response
    def raster(request, response):
        if mode[0] == 'timeout':
            time.sleep(1.2)
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
        if mode[0] == 'graph_failure':
            response.result = SerializePoseGraph.Response.RESULT_FAILED_TO_WRITE_FILE
            return response
        prefix = Path(request.filename)
        artifact_path(prefix, '.posegraph').write_bytes(b'graph')
        artifact_path(prefix, '.data').write_bytes(b'scans')
        response.result = SerializePoseGraph.Response.RESULT_SUCCESS
        return response
    node.create_service(SaveMap, '/slam_toolbox/save_map', raster)
    node.create_service(GetMap, '/slam_toolbox/dynamic_map', refresh)
    node.create_service(SerializePoseGraph, '/slam_toolbox/serialize_map', graph)
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
                                       '-p', f'response_timeout:={timeout}', '-p', 'wait_timeout:=2.0'],
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
            print('map service success, existing-name refusal, graph failure, raster failure, bounded timeout: passed')
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run()
