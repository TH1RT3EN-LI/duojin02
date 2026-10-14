"""Report fixed-map scan consistency without controlling robot motion."""
from dataclasses import fields
import hashlib
import json
import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from duojin02_bringup.localization_quality import ScorePolicy, MapEvidence, PARAMETER_DEFAULTS


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        descriptor = ParameterDescriptor(read_only=True)
        defaults = ScorePolicy()
        policy = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name), descriptor).value
                  for f in fields(defaults)}
        self.policy = ScorePolicy(**policy)
        if not all(math.isfinite(float(value)) for value in policy.values()):
            raise ValueError('Quality policy parameters must be finite')
        if any(not 0 <= policy[key] <= 1 for key in
               ('hit_fraction', 'known_fraction', 'max_contradiction_fraction')):
            raise ValueError('Quality fractions must be in [0,1]')
        if (min(policy['min_hits'], policy['max_beams'], policy['max_ray_cells'],
                policy['endpoint_tolerance'], policy['max_range']) <= 0 or
                policy['ray_margin'] < policy['endpoint_tolerance']):
            raise ValueError('Quality work budgets and ranges must be positive; ray margin must cover endpoint tolerance')
        self.map_frame = self.declare_parameter('map_frame', PARAMETER_DEFAULTS['map_frame'], descriptor).value
        self.convention = self.declare_parameter('map_cell_convention', PARAMETER_DEFAULTS['map_cell_convention'], descriptor).value
        self.grid, self.pending, self.map_signature = None, None, None
        self.received = self.processed = 0
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/localization/status', durable)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish('awaiting_map_scan_transform')

    def on_map(self, msg):
        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError('Map frame does not match configured map_frame')
            q = msg.info.origin.orientation
            if (abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or
                    not math.isfinite(q.w) or not math.isfinite(q.z) or
                    abs(q.w*q.w + q.z*q.z - 1.0) > 1e-3):
                raise ValueError('Map must have a finite planar orientation')
            signature = (msg.info.width, msg.info.height, msg.info.resolution,
                         msg.info.origin.position.x, msg.info.origin.position.y, yaw(q),
                         hashlib.sha256(msg.data.tobytes()).hexdigest())
            if signature == self.map_signature:
                return
            self.grid = MapEvidence(msg.info.width, msg.info.height, msg.info.resolution,
                                    (msg.info.origin.position.x, msg.info.origin.position.y, yaw(q)),
                                    msg.data, self.convention)
            self.map_signature = signature
            self.publish('map_received_or_changed')
        except (ValueError, TypeError, OverflowError) as error:
            self.grid = self.map_signature = None
            self.publish('invalid_map:' + str(error))

    def on_scan(self, msg):
        self.received += 1
        self.pending = msg  # Quality scoring never builds a queue of old scans.

    def tick(self):
        if self.pending is None:
            return
        msg, self.pending = self.pending, None
        if self.grid is None:
            self.publish('map_unavailable')
            return
        try:
            raw = msg.header.stamp
            if (not msg.header.frame_id or raw.sec < 0 or
                    not 0 <= raw.nanosec < 1_000_000_000 or
                    raw.sec == 0 and raw.nanosec == 0):
                raise ValueError('Invalid scan frame or timestamp')
            expected_max = msg.angle_min + (len(msg.ranges)-1)*msg.angle_increment
            if (not math.isfinite(msg.angle_max) or
                    abs(msg.angle_max-expected_max) > max(1e-4, abs(msg.angle_increment)*0.05) or
                    not all(math.isfinite(v) and v >= 0 for v in (msg.scan_time, msg.time_increment))):
                raise ValueError('Inconsistent LaserScan metadata')
            stamp = Time.from_msg(raw)
            tf = self.buffer.lookup_transform(self.map_frame, msg.header.frame_id, stamp)
            p, q = tf.transform.translation, tf.transform.rotation
            if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                    abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1.0) > 1e-3):
                raise ValueError('Invalid transform values')
            evidence = self.grid.score(msg.ranges, msg.angle_min, msg.angle_increment,
                                       msg.range_min, msg.range_max, (p.x, p.y, yaw(q)), self.policy)
            self.processed += 1
            self.publish(evidence['reason'], evidence, stamp.nanoseconds/1e9)
        except TransformException:
            self.publish('exact_transform_unavailable')
        except (ValueError, TypeError, OverflowError) as error:
            self.publish('invalid_scan_or_transform:' + str(error))

    def publish(self, reason, evidence=None, scan_stamp=None):
        consistent = bool(evidence['valid']) if evidence is not None else None
        quality = 'unavailable' if consistent is None else 'consistent' if consistent else 'inconsistent'
        self.status_pub.publish(String(data=json.dumps(dict(
            advisory_only=True, quality=quality, consistent=consistent, reason=reason,
            emitted_ros_ns=self.get_clock().now().nanoseconds, scan_stamp=scan_stamp,
            received_scans=self.received, processed_scans=self.processed,
            map_cell_convention=self.convention, evidence=evidence), sort_keys=True)))
        if consistent is False or reason.startswith(('invalid_', 'exact_transform_')):
            self.get_logger().warning('Localization quality: ' + reason, throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown(context=node.context)


if __name__ == '__main__':
    main()
