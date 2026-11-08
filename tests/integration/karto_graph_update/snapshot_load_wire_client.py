"""Independent process for the real typed LoadSnapshot service fixture."""
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import LoadSnapshot, SaveSnapshot

from duojin02_slam_tools.snapshots import validate_bundle


def main():
    directory = Path(sys.argv[1])
    rclpy.init()
    node = Node('snapshot_typed_load_wire_client')
    try:
        saver = node.create_client(SaveSnapshot, '/slam_toolbox/save_snapshot')
        loader = node.create_client(LoadSnapshot, '/slam_toolbox/load_snapshot')
        def call(client, request):
            assert client.wait_for_service(timeout_sec=5), 'Service unavailable'
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20)
            assert future.done() and not future.exception(), 'Service call failed or timed out'
            return future.result()
        def save(name):
            request = SaveSnapshot.Request()
            request.mode = request.SNAPSHOT
            request.include_graph = True
            request.filename = str(directory / name)
            response = call(saver, request)
            assert response.result == response.RESULT_SUCCESS, response.message
            manifest, _ = validate_bundle(directory / name)
            assert response.snapshot_id == manifest['snapshot_id']
            return manifest
        def load(name):
            request = LoadSnapshot.Request()
            request.filename = str(directory / name)
            request.match_type = request.START_AT_FIRST_NODE
            return call(loader, request)
        before = save('wire-before-load')
        attempts = []
        for name in ('bad-hash', 'bad-archive'):
            response = load(name)
            assert response.result == response.RESULT_FAILED and not response.state_committed, response.message
            after = save('wire-after-' + name)
            assert before['snapshot_id'] == after['snapshot_id'], 'Failed wire Load modified live state'
            attempts.append(dict(input=name, result=response.result, state_committed=response.state_committed, message=response.message, before_snapshot_id=before['snapshot_id'], after_snapshot_id=after['snapshot_id']))
        response = load('f0')
        assert response.result == response.RESULT_SUCCESS and response.state_committed, response.message
        source, _ = validate_bundle(directory / 'f0')
        assert response.snapshot_id == source['snapshot_id']
        assert response.migration_policy == 'exact_backend_reference_checkpoint'
        loaded = save('wire-after-valid-load')
        for name in ('map.yaml', 'map.pgm', 'graph_update.json'):
            assert (directory / 'f0' / name).read_bytes() == (directory / 'wire-after-valid-load' / name).read_bytes(), name
        (directory / 'typed-load-wire-result.json').write_text(json.dumps(dict(
            passed=True, scope='independent Python process using generated DDS service types and actual common C++ service handlers',
            rejected_loads_preserve_live=attempts, valid_load=dict(source_snapshot_id=response.snapshot_id, migration_policy=response.migration_policy, state_committed=response.state_committed, loaded_snapshot_id=loaded['snapshot_id']),
        ), indent=2) + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
