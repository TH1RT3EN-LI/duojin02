import os
import hashlib
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
from slam_toolbox.srv import SaveSnapshot
from std_srvs.srv import Trigger
from duojin02_slam_tools.snapshots import validate_map
from duojin02_slam_tools.offline import await_response
from ros_isolation import configure_isolation

ROOT = Path(__file__).resolve().parents[2]


def run():
    configure_isolation()
    rclpy.init()
    node = Node("slam_toolbox", start_parameter_services=False)
    mode = ["success"]

    def snapshot(request, response):
        if mode[0] == "timeout":
            time.sleep(1.2)
        if mode[0] == "shared_budget":
            time.sleep(0.5)
        response.result = SaveSnapshot.Response.RESULT_FAILED
        if mode[0] in ("raster_failure", "graph_failure"):
            response.message = "injected " + mode[0]
            return response
        directory = Path(request.filename)
        # A timed-out client owns and removes the unpublished pending parent.
        if not directory.parent.exists():
            response.message = "client abandoned pending destination"
            return response
        directory.mkdir()
        contents = {
            "yaml": ("map.yaml", b"image: map.pgm\nmode: trinary\nresolution: 0.03\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"),
            "image": ("map.pgm", b"P5\n1 1\n255\n\xfe"),
            "graph_update": ("graph_update.json", b'{"graph_revision":1}'),
            "scan_geometry": ("scan_geometry.json", b'{"schema":"fixture-only"}'),
            "input_boundary": ("input_boundary.json", b'{"admissions_paused":true,"queues_drained":true}'),
            "runtime": ("runtime.json", b'{"source_marker_binding":"fixture-only"}'),
        }
        if request.include_graph:
            contents.update(posegraph=("graph.posegraph", b"graph"), data=("graph.data", b"scans"))
        entries = []
        for role, (name, content) in sorted(contents.items()):
            (directory / name).write_bytes(content)
            entries.append(dict(role=role, path=name, sha256=hashlib.sha256(content).hexdigest(), size_bytes=len(content)))
        finalized = request.mode == request.FINALIZE
        operation = "finalize" if finalized else "snapshot"
        identity = "".join(f'{e["role"]}\t{e["path"]}\t{e["sha256"]}\t{e["size_bytes"]}\n' for e in entries) + f"{operation}\n{request.request_id}\n"
        snapshot_id = hashlib.sha256(identity.encode()).hexdigest()
        (directory / "snapshot.json").write_text(json.dumps(dict(
            schema_version=2, producer="slam_toolbox.SaveSnapshot.v1", snapshot_id=snapshot_id,
            operation=operation, request_id=request.request_id, finalized=finalized,
            consistency="single_graph_owner_boundary", files=entries,
        )))
        response.result = SaveSnapshot.Response.RESULT_SUCCESS
        response.snapshot_id = snapshot_id if mode[0] != "wrong_identity" else "0" * 64
        response.manifest_path = str(directory / "snapshot.json")
        response.finalized = finalized
        response.state_committed = finalized
        response.commit_id = 1 if finalized else 0
        return response

    node.create_service(SaveSnapshot, "/slam_toolbox/save_snapshot", snapshot)

    def parameter_names(request, response):
        response.result.names = ["map_start_pose"]
        return response

    def parameter_values(request, response):
        if mode[0] == "metadata_timeout":
            time.sleep(0.8)
        if mode[0] != "metadata_incomplete":
            response.values = [
                Parameter("map_start_pose", value=[1.0, 2.0, 0.5]).to_parameter_msg().value
            ]
        return response

    node.create_service(ListParameters, "/slam_toolbox/list_parameters", parameter_names)
    node.create_service(GetParameters, "/slam_toolbox/get_parameters", parameter_values)

    def delayed_status(request, response):
        time.sleep(2.2)
        response.success = True
        return response

    node.create_service(Trigger, "/phase_budget_status", delayed_status)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)

            def save(name, timeout="2.0", save_mode="snapshot", request_id=""):
                return subprocess.run(
                    [
                        "ros2",
                        "run",
                        "duojin02_slam_tools",
                        "save_map_client_node",
                        "--ros-args",
                        "-p",
                        f"output_dir:={output}",
                        "-p",
                        f"map_name:={name}",
                        "-p",
                        f"save_timeout:={timeout}",
                        "-p", f"save_mode:={save_mode}",
                        *(["-p", f"finalize_request_id:={request_id}"] if request_id else []),
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=15,
                )

            def launch_save(name):
                return subprocess.run(
                    [
                        "ros2",
                        "launch",
                        str(ROOT / "src/duojin02_bringup/launch/save_map.launch.py"),
                        f"output_dir:={output}",
                        f"map_name:={name}",
                        "save_timeout:=2.0",
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=15,
                )

            success = save("room.v2")
            assert success.returncode == 0, success.stdout
            validate_map(output / "room.v2/map.yaml")
            snapshot = json.loads((output / "room.v2.receipt.json").read_text())
            assert snapshot["client_observed_provenance"]["slam_parameters"]["map_start_pose"] == [1.0, 2.0, 0.5]
            missing_id = save("missing_finalize_id", save_mode="finalize")
            assert missing_id.returncode != 0 and not (output / "missing_finalize_id").exists()
            finalized = save("finalized", save_mode="finalize", request_id="declared-finalize-1")
            assert finalized.returncode == 0, finalized.stdout
            assert json.loads((output / "finalized/snapshot.json").read_text())["finalized"] is True
            launch_success = launch_save("launch_success")
            assert launch_success.returncode == 0, launch_success.stdout
            validate_map(output / "launch_success/map.yaml")
            assert save("room.v2").returncode != 0, "existing map overwritten"
            mode[0] = "graph_failure"
            launch_failure = launch_save("launch_failure")
            assert (
                launch_failure.returncode != 0
            ), "launch reported success after its save client failed"
            assert not (output / "launch_failure").exists(), launch_failure.stdout
            failure = save("failed_graph")
            assert (
                failure.returncode != 0 and not (output / "failed_graph").exists()
            ), failure.stdout
            mode[0] = "wrong_identity"
            assert save("wrong_identity").returncode != 0 and not (output / "wrong_identity").exists()
            mode[0] = "raster_failure"
            assert save("failed_raster").returncode != 0
            mode[0] = "timeout"
            began = time.monotonic()
            expired = save("expired", timeout="0.2")
            assert (
                expired.returncode != 0 and not (output / "expired").exists()
            ), expired.stdout
            assert time.monotonic() - began < 5
            time.sleep(1.3)
            mode[0] = "shared_budget"
            shared = save("shared_budget", timeout="0.4")
            assert (
                shared.returncode != 0 and not (output / "shared_budget").exists()
            ), shared.stdout
            time.sleep(0.4)
            mode[0] = "metadata_incomplete"
            partial = save("metadata_incomplete")
            assert partial.returncode == 0, partial.stdout
            validate_map(output / "metadata_incomplete/map.yaml")
            provenance = json.loads((output / "metadata_incomplete.receipt.json").read_text())[
                "client_observed_provenance"
            ]
            assert provenance["algorithm"] == "slam_toolbox" and provenance["version"]
            assert provenance["backend_build"] and "incomplete" in provenance["warnings"][0]
            assert (
                "slam_parameters" not in provenance
            ), "partial parameter list presented as complete"
            mode[0] = "metadata_timeout"
            warned = save("metadata_timeout", timeout="0.4")
            assert warned.returncode == 0, warned.stdout
            validate_map(output / "metadata_timeout/map.yaml")
            provenance = json.loads((output / "metadata_timeout.receipt.json").read_text())[
                "client_observed_provenance"
            ]
            assert provenance["backend_build"] and provenance["warnings"]
            time.sleep(0.9)
            control = Node("offline_phase_budget_probe")
            try:
                client = control.create_client(Trigger, "/phase_budget_status")
                assert client.wait_for_service(timeout_sec=3)
                began = time.monotonic()
                response = await_response(
                    control, client.call_async(Trigger.Request()), began + 3.0, "delayed status"
                )
                assert response.success and time.monotonic() - began > 2.0
                stalled = Future()
                began = time.monotonic()
                try:
                    await_response(control, stalled, began + 0.2, "stalled status")
                    raise AssertionError("stalled status ignored the phase budget")
                except TimeoutError:
                    assert stalled.cancelled() and time.monotonic() - began < 0.6
                dead = Future()
                try:
                    await_response(
                        control,
                        dead,
                        time.monotonic() + 3,
                        "dead process",
                        (type("Stopped", (), {"poll": lambda self: 1})(),),
                    )
                    raise AssertionError("dead process was ignored")
                except RuntimeError:
                    assert dead.cancelled()
            finally:
                control.destroy_node()
            print(
                "single-RPC immutable bundle service/launch success, response identity binding, launch failure status propagation, existing-name refusal, graph/raster failures, one save budget, "
                "metadata timeout/incomplete warning, delayed/stalled/dead-process phase budgets: passed"
            )
    finally:
        stopped = executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        print(
            json.dumps(
                {
                    "map_service_fixture_cleanup": {
                        "executor_stopped": stopped,
                        "worker_stopped": not thread.is_alive(),
                    }
                }
            )
        )
        assert stopped and not thread.is_alive(), "Map service fixture did not finish normally"


if __name__ == "__main__":
    run()
