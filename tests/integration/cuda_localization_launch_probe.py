"""Verify the shipped localization launch selects the new node and backend policy."""
import argparse
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time

from ros_isolation import configure_isolation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    domain = configure_isolation()
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    rclpy.init()
    node = rclpy.create_node("cuda_localization_launch_probe")
    image = args.output / "map.pgm"
    image.write_bytes(b"P5\n20 20\n255\n" + bytes([254]) * 400)
    map_file = args.output / "map.yaml"
    map_file.write_text(f"image: {image.resolve()}\nresolution: 0.05\norigin: [0, 0, 0]\n"
                        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    report = {"status": "running", "domain": domain, "arms": [],
              "scope": "real installed localization launch and lifecycle/parameter wiring; no movement"}

    def call(service_type, name, request, deadline):
        client = node.create_client(service_type, name)
        try:
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=.1):
                    break
            else:
                raise RuntimeError(f"Service unavailable: {name}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., deadline-time.monotonic()))
            if not future.done() or future.result() is None:
                raise RuntimeError(f"Service timeout: {name}")
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        for enabled, expected in (("false", "cpu"), ("true", "auto")):
            log_path = args.output / f"{expected}.log"
            arm = {"use_gpu": enabled, "expected_backend": expected}
            report["arms"].append(arm)
            with log_path.open("w") as log:
                process = subprocess.Popen([
                    "ros2", "launch", "duojin02_bringup", "localization.launch.py",
                    f"map:={map_file.resolve()}", f"use_gpu:={enabled}"],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic()+30
                    for target in ("map_server", "amcl"):
                        while time.monotonic() < deadline:
                            if process.poll() is not None:
                                raise RuntimeError("Localization launch exited during startup")
                            state = call(GetState, f"/{target}/get_state", GetState.Request(), deadline)
                            if state.current_state.id == 3:
                                break
                            rclpy.spin_once(node, timeout_sec=.05)
                        else:
                            raise RuntimeError(f"{target} did not become active")
                    request = GetParameters.Request()
                    request.names = ["observation_backend", "laser_model_type", "robot_model_type"]
                    values = call(GetParameters, "/amcl/get_parameters", request, deadline).values
                    arm["parameters"] = dict(zip(request.names, [v.string_value for v in values]))
                    assert arm["parameters"]["observation_backend"] == expected
                    assert arm["parameters"]["laser_model_type"] == "likelihood_field_prob"
                    assert arm["parameters"]["robot_model_type"] == "nav2_amcl::OmniMotionModel"
                    arm["both_active"] = True
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        arm["exit_code"] = process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        # The still-live session leader identifies this probe's owned group.
                        if os.getpgid(process.pid) == process.pid:
                            os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                        raise RuntimeError("Localization launch required forced cleanup")
                assert arm["exit_code"] == 0
            text = log_path.read_text()
            assert "Managed nodes are active" in text
            assert "escalating" not in text
            started = dict(re.findall(r"\[INFO\] \[([^\]]+)\]: process started with pid \[(\d+)\]", text))
            clean = dict(re.findall(r"\[INFO\] \[([^\]]+)\]: process has finished cleanly \[pid (\d+)\]", text))
            assert len(started) == 3 and started == clean, (started, clean)
            assert "process has died" not in text
            arm["clean_child_exits"] = clean
            survivors = []
            for entry in Path("/proc").glob("[0-9]*/stat"):
                try:
                    fields = entry.read_text().rsplit(")", 1)[1].split()
                    if int(fields[2]) == process.pid and fields[0] != "Z":
                        survivors.append(int(entry.parent.name))
                except (FileNotFoundError, ProcessLookupError):
                    pass
            arm["surviving_group_members"] = survivors
            assert not survivors, survivors
        report["status"] = "passed"
    except Exception as error:
        report["status"], report["error"] = "failed", str(error)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
        (args.output / "launch.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
