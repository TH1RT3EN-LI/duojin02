"""Bounded ROS service calls for paired raster and pose-graph snapshots."""

import json
import math
import os
import xml.etree.ElementTree as ET
from array import array
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters, ListParameters
from nav_msgs.srv import GetMap
from slam_toolbox.srv import SaveMap, SerializePoseGraph

from .snapshots import commit_snapshot, reserve_snapshot


def output_directory(value):
    output = Path(value).expanduser()
    if output.is_absolute():
        return output
    configured = os.environ.get("DUOJIN01_WORKSPACE_ROOT")
    if configured:
        root = Path(configured).expanduser()
        if not root.is_absolute():
            raise ValueError("DUOJIN01_WORKSPACE_ROOT must be absolute")
    else:
        prefix = Path(get_package_prefix("duojin01_slam_tools"))
        root = next((item.parent for item in (prefix, *prefix.parents) if item.name == "install"), None)
        if root is None:
            raise ValueError("Set DUOJIN01_WORKSPACE_ROOT or use an absolute output_dir")
    return root / output


class SaveMapClient(Node):
    def __init__(self):
        super().__init__("save_map_client_node")
        self.map_name = self.declare_parameter("map_name", "auto").value
        self.output = output_directory(self.declare_parameter("output_dir", "maps").value)
        self.wait_timeout = float(self.declare_parameter("wait_timeout", 30.0).value)
        self.response_timeout = float(self.declare_parameter("response_timeout", 30.0).value)
        self.serialize_graph = self.declare_parameter("serialize_graph", True).value
        if not all(math.isfinite(value) and value > 0 for value in (self.wait_timeout, self.response_timeout)):
            raise ValueError("Service timeouts must be positive")
        self.map_client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        self.refresh_client = self.create_client(GetMap, "/slam_toolbox/dynamic_map")
        self.graph_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

    def await_response(self, future, description):
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.response_timeout)
        if not future.done():
            future.cancel()
            raise TimeoutError(f"Timed out waiting for {description}")
        if future.exception():
            raise future.exception()
        return future.result()

    def call(self, client, request, description):
        if not client.wait_for_service(timeout_sec=self.wait_timeout):
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
        listing = self.call(names_client, ListParameters.Request(), "SLAM parameter names")
        request = GetParameters.Request()
        request.names = listing.result.names
        response = self.call(values_client, request, "SLAM parameters")
        result["slam_parameters"] = {
            name: Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
            for name, value in zip(listing.result.names, response.values)
        }
        # Humble exposes numeric ROS parameter arrays as array.array objects.
        result["slam_parameters"] = {
            name: list(value) if isinstance(value, array) else value
            for name, value in result["slam_parameters"].items()
        }
        return result

    def run(self):
        try:
            with reserve_snapshot(self.output, self.map_name, self.wait_timeout) as (pending, final):
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
                saved = commit_snapshot(pending, final, self.provenance(), self.serialize_graph)
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
