"""Bounded ROS service calls for paired raster and pose-graph snapshots."""

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
from nav_msgs.srv import GetMap
from slam_toolbox.srv import SaveMap, SerializePoseGraph

from .snapshots import commit_snapshot, reserve_snapshot


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
        root = next((item.parent for item in (prefix, *prefix.parents) if item.name == "install"), None)
        if root is None:
            raise ValueError("Set DUOJIN02_WORKSPACE_ROOT or use an absolute output_dir")
    return root / output


class SaveMapClient(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("save_map_client_node", parameter_overrides=parameter_overrides)
        descriptor = ParameterDescriptor(read_only=True)
        self.map_name = self.declare_parameter("map_name", "auto", descriptor).value
        self.output = output_directory(self.declare_parameter("output_dir", "maps", descriptor).value)
        self.save_timeout = float(self.declare_parameter("save_timeout", DEFAULT_SAVE_TIMEOUT, descriptor).value)
        self.serialize_graph = self.declare_parameter("serialize_graph", True, descriptor).value
        if not math.isfinite(self.save_timeout) or self.save_timeout <= 0:
            raise ValueError("save_timeout must be finite and positive")
        self.map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.refresh_client = self.create_client(GetMap, "/slam_toolbox/dynamic_map")
        self.graph_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

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
        result = {"algorithm": "slam_toolbox", "version": ET.parse(share / "package.xml").findtext("version")}
        build = share / "hardware_build.json"
        if build.is_file():
            result["backend_build"] = json.loads(build.read_text())
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
            # Humble exposes numeric ROS parameter arrays as array.array objects.
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
            with reserve_snapshot(self.output, self.map_name, deadline=self.deadline) as (pending, final):
                current = self.call(self.refresh_client, GetMap.Request(), "/slam_toolbox/dynamic_map")
                if not current.map.info.width or not current.map.info.height:
                    raise RuntimeError("SLAM has not produced a map yet")
                request = SaveMap.Request()
                request.name.data = str(pending)
                response = self.call(self.map_client, request, "/slam_toolbox/save_map")
                if response.result != SaveMap.Response.RESULT_SUCCESS:
                    raise RuntimeError(f"Raster map save failed with result {response.result}")
                if self.serialize_graph:
                    graph_request = SerializePoseGraph.Request()
                    graph_request.filename = str(pending)
                    response = self.call(self.graph_client, graph_request, "/slam_toolbox/serialize_map")
                    if response.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                        raise RuntimeError(f"Pose graph save failed with result {response.result}")
                # dynamic_map refreshes the published raster after queue drain;
                # live mapping may still update before SaveMap. Validate the
                # selected artifact rather than comparing these two responses.
                saved = commit_snapshot(pending, final, self.provenance(),
                                        self.serialize_graph, validate_raster=True)
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
