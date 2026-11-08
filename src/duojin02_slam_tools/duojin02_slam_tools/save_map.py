import json
import math
import os
import time
import xml.etree.ElementTree as ET
from array import array
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterDescriptor
from rcl_interfaces.srv import GetParameters, ListParameters
from slam_toolbox.srv import SaveSnapshot

from .snapshots import commit_bundle_snapshot, reserve_snapshot, validate_bundle


DEFAULT_SAVE_TIMEOUT = 30.0


def output_directory(value):
    output = Path(value).expanduser()
    if output.is_absolute():
        return output
    configured = os.environ.get("DUOJIN02_WORKSPACE_ROOT")
    if configured:
        root = Path(configured).expanduser()
        if not root.is_absolute():
            raise ValueError("DUOJIN02_WORKSPACE_ROOT must be absolute")
    else:
        prefix = Path(get_package_prefix("duojin02_slam_tools"))
        root = next(
            (item.parent for item in (prefix, *prefix.parents) if item.name == "install"),
            None,
        )
        if root is None:
            raise ValueError("Set DUOJIN02_WORKSPACE_ROOT or use an absolute output_dir")
    return root / output


class SaveMapClient(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("save_map_client_node", parameter_overrides=parameter_overrides)
        descriptor = ParameterDescriptor(read_only=True)
        self.map_name = self.declare_parameter("map_name", "auto", descriptor).value
        self.output = output_directory(
            self.declare_parameter("output_dir", "maps", descriptor).value
        )
        self.save_timeout = float(
            self.declare_parameter("save_timeout", DEFAULT_SAVE_TIMEOUT, descriptor).value
        )
        self.serialize_graph = self.declare_parameter("serialize_graph", True, descriptor).value
        if not math.isfinite(self.save_timeout) or self.save_timeout <= 0:
            raise ValueError("save_timeout must be finite and positive")
        self.save_mode = self.declare_parameter("save_mode", "snapshot", descriptor).value
        self.finalize_request_id = self.declare_parameter("finalize_request_id", "", descriptor).value
        if self.save_mode not in ("snapshot", "finalize"):
            raise ValueError("save_mode must be snapshot or finalize")
        if self.save_mode == "finalize" and (not self.finalize_request_id or not self.serialize_graph):
            raise ValueError("Finalize requires finalize_request_id and serialize_graph=true")
        self.snapshot_client = self.create_client(SaveSnapshot, "/slam_toolbox/save_snapshot")

    def remaining(self):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Map save budget exhausted")
        return remaining

    def await_response(self, future, description):
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.remaining())
        except Exception:
            future.cancel()
            raise
        if not future.done():
            future.cancel()
            raise TimeoutError(f"Timed out waiting for {description}")
        if future.exception():
            raise future.exception()
        return future.result()

    def call(self, client, request, description):
        if not client.wait_for_service(timeout_sec=self.remaining()):
            raise TimeoutError(f"Service unavailable: {description}")
        return self.await_response(client.call_async(request), description)

    def provenance(self):
        share = Path(get_package_share_directory("slam_toolbox"))
        result = {
            "algorithm": "slam_toolbox",
            "version": ET.parse(share / "package.xml").findtext("version"),
        }
        build = share / "hardware_build.json"
        if build.is_file():
            result["backend_build"] = json.loads(build.read_text())
        saver_share = Path(get_package_share_directory("nav2_map_server"))
        saver_build = saver_share / "hardware_build.json"
        result["map_saver"] = {
            "version": ET.parse(saver_share / "package.xml").findtext("version"),
        }
        if saver_build.is_file():
            result["map_saver"]["build"] = json.loads(saver_build.read_text())
        names_client = self.create_client(ListParameters, "/slam_toolbox/list_parameters")
        values_client = self.create_client(GetParameters, "/slam_toolbox/get_parameters")
        try:
            listing = self.call(names_client, ListParameters.Request(), "SLAM parameter names")
            request = GetParameters.Request()
            request.names = listing.result.names
            response = self.call(values_client, request, "SLAM parameters")
            if len(response.values) != len(request.names):
                raise RuntimeError("SLAM parameter response is incomplete")
            result["slam_parameters"] = {
                name: Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
                for name, value in zip(listing.result.names, response.values)
            }

            result["slam_parameters"] = {
                name: list(value) if isinstance(value, array) else value
                for name, value in result["slam_parameters"].items()
            }
        except Exception as error:
            warning = f"Runtime SLAM parameters unavailable: {error}"
            result["warnings"] = [warning]
            self.get_logger().warning(warning)
        return result

    def run(self):
        self.deadline = time.monotonic() + self.save_timeout
        try:
            with reserve_snapshot(self.output, self.map_name, deadline=self.deadline) as (
                pending,
                final,
            ):
                request = SaveSnapshot.Request()
                request.mode = request.FINALIZE if self.save_mode == "finalize" else request.SNAPSHOT
                request.filename = str(pending)
                request.request_id = self.finalize_request_id if self.save_mode == "finalize" else ""
                request.include_graph = self.serialize_graph
                response = self.call(self.snapshot_client, request, "/slam_toolbox/save_snapshot")
                if response.result != SaveSnapshot.Response.RESULT_SUCCESS:
                    raise RuntimeError(f"Snapshot failed: {response.message}")
                if self.save_mode == "finalize" and not response.state_committed:
                    raise RuntimeError("Finalize returned no confirmed native commit")
                if response.finalized != (self.save_mode == "finalize"):
                    raise RuntimeError("Snapshot response operation mismatch")
                native, _ = validate_bundle(pending, require_graph=self.serialize_graph)
                if native["snapshot_id"] != response.snapshot_id or Path(response.manifest_path).resolve() != (pending / "snapshot.json").resolve():
                    raise RuntimeError("Snapshot response identity/path does not match its bundle")
                saved = commit_bundle_snapshot(
                    pending, final, self.provenance(), require_graph=self.serialize_graph,
                )
                self.get_logger().info(f"Saved complete map snapshot: {saved}")
                return 0
        except Exception as error:
            self.get_logger().error(f"Map save failed: {error}")
            return 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SaveMapClient()
        return node.run()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
