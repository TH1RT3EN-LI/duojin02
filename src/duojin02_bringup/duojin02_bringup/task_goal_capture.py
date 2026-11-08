"""Passive original-map-goal and command/measurement recorder (never sends motion).

Run with python3 -m duojin02_bringup.task_goal_capture. /goal_pose is the RViz
request path; direct NavigateToPose Action clients must explicitly publish the
same original request or provide their own immutable task ledger.
"""
import argparse
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import time


def map_identity(message):
    """Content identity includes frame, grid geometry and cells, not receipt time."""
    i, p = message.info, message.info.origin
    canonical = message.header.frame_id.encode("utf-8") + b"\0"
    canonical += struct.pack("<IIdddddddd", i.width, i.height, i.resolution,
                              p.position.x, p.position.y, p.position.z,
                              p.orientation.x, p.orientation.y,
                              p.orientation.z, p.orientation.w)
    canonical += bytes((int(cell) & 255) for cell in message.data)
    digest = 14695981039346656037
    for byte in canonical:
        digest = ((digest ^ byte) * 1099511628211) & ((1 << 64)-1)
    return "occupancy_fnv1a64:" + format(digest, "016x")


def original_goal_fields(message, map_version="unknown", generation="unknown", sequence=1):
    p, q = message.pose.position, message.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite original request")
    if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 1e-6:
        raise ValueError("original request quaternion must be normalized")
    return dict(schema="task_goal.v1", goal_id=str(sequence),
                original_frame=message.header.frame_id,
                original_stamp_ns=str(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec),
                original_x=repr(p.x), original_y=repr(p.y), original_z=repr(p.z),
                original_qx=repr(q.x), original_qy=repr(q.y), original_qz=repr(q.z),
                original_qw=repr(q.w), task_frame="base_footprint", map_identity=map_version,
                map_generation=str(generation), local_reference="frozen_map",
                conversion_source="original_goal_pose_no_coordinate_conversion",
                registration="map_click", physical_registration="unknown",
                task_uncertainty="unknown", physical_calibration="unknown")


def json_safe_fields(fields):
    invalid = []
    def clean(value, path):
        if isinstance(value, float) and not math.isfinite(value):
            invalid.append(path)
            return None
        if isinstance(value, dict):
            return {key: clean(item, path + '.' + key) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [clean(item, path + '.' + str(index)) for index, item in enumerate(value)]
        return value
    result = clean(fields, 'record')
    if invalid:
        result['nonfinite_input_fields'] = invalid
    return result


def open_record(output_path=None, disabled=False):
    """Create a unique session log, or honor an explicit exclusive path."""
    if disabled:
        return None
    if output_path is not None:
        return output_path.open("x", encoding="utf-8", buffering=1)
    directory = Path(os.environ.get("ROS_LOG_DIR") or Path.home() / ".ros" / "log")
    directory.mkdir(parents=True, exist_ok=True)
    return tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", buffering=1,
                                       prefix="task-goal-", suffix=".jsonl",
                                       dir=directory, delete=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    recording = parser.add_mutually_exclusive_group()
    recording.add_argument("--output", type=Path, help="new JSONL file; existing paths are refused")
    recording.add_argument("--no-record", action="store_true", help="publish metadata without a session file")
    args, ros_args = parser.parse_known_args(argv)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    output = open_record(args.output, args.no_record)
    try:
        rclpy.init(args=ros_args)
    except BaseException:
        if output:
            output.close()
        raise

    class Capture(Node):
        def __init__(self):
            super().__init__("task_goal_capture")
            if output:
                self.get_logger().info(f"Task goal recording: {output.name}")
            else:
                self.get_logger().warning("Task goal recording disabled")
            self.identity, self.generation, self.sequence = "unknown", 0, 0
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.publisher = self.create_publisher(DiagnosticArray, "task_goal_contract", qos)
            self.create_subscription(OccupancyGrid, "map", self.on_map, qos)
            self.create_subscription(PoseStamped, "goal_pose", self.on_goal, 10)
            for topic in ("cmd_vel_nav", "cmd_vel", "cmd_vel_nav_guarded", "cmd_vel_base"):
                self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_twist(t, msg), 100)
            self.create_subscription(Odometry, "odom", self.on_odom, 100)
            for topic in ("task_state_at_control", "amcl_estimate_contract"):
                self.create_subscription(DiagnosticArray, topic,
                                         lambda msg, t=topic: self.on_contract(t, msg), 100)

        def write(self, kind, fields):
            if output:
                output.write(json.dumps(json_safe_fields(dict(kind=kind, receipt_steady_ns=time.monotonic_ns(),
                                             receipt_ros_ns=self.get_clock().now().nanoseconds,
                                             steady_clock_scope="host_monotonic", **fields)),
                                        allow_nan=False) + "\n")

        def on_map(self, msg):
            identity = map_identity(msg)
            if identity != self.identity:
                self.identity, self.generation = identity, self.generation + 1
                self.write("map", dict(map_identity=identity, map_generation=self.generation))

        def on_goal(self, msg):
            self.sequence += 1
            try:
                fields = original_goal_fields(msg, self.identity, self.generation, self.sequence)
            except ValueError as error:
                self.write("invalid_goal", dict(reason=str(error)))
                return
            self.write("goal", fields)
            status = DiagnosticStatus(name="navigation/task_goal", message="metadata_only",
                                      values=[KeyValue(key=k, value=v) for k, v in fields.items()])
            result = DiagnosticArray(status=[status])
            result.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(result)

        def on_twist(self, topic, msg):
            self.write("command", dict(topic=topic, vx=msg.linear.x, vy=msg.linear.y,
                                       wz=msg.angular.z, application_time="unknown"))

        def on_odom(self, msg):
            self.write("measured_twist", dict(topic="odom", frame=msg.child_frame_id,
                       source_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                       vx=msg.twist.twist.linear.x, vy=msg.twist.twist.linear.y,
                       wz=msg.twist.twist.angular.z, actuator_calibration="unknown"))

        def on_contract(self, topic, msg):
            self.write("contract", dict(topic=topic, status=[dict(name=s.name,
                       values={kv.key: kv.value for kv in s.values}) for s in msg.status]))

    node = None
    try:
        node = Capture()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if output:
            output.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
