import math
import time

from duojin02_bringup.safety_profile import safety_defaults


class ScanReadiness:
    def __init__(self, scan_timeout_s=None):
        self.timeout_s = (
            safety_defaults()["scan_timeout_s"] if scan_timeout_s is None else scan_timeout_s
        )
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("scan_timeout_s must be finite and positive")
        self.timeout_ns = self.timeout_s * 1_000_000_000
        self.pending = None
        self.last_seen_ns = -1
        self.last_clock_ns = None

    def check_time(self, now_ns, wall):
        valid = isinstance(now_ns, int) and now_ns > 0 and math.isfinite(wall)
        if not valid:
            self.pending = None
            return False
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.pending = None
            self.last_seen_ns = -1
        self.last_clock_ns = now_ns
        return True

    @staticmethod
    def stamp_ns(scan):
        try:
            s = scan.header.stamp
            if (
                not isinstance(s.sec, int)
                or not isinstance(s.nanosec, int)
                or s.sec < 0
                or not 0 <= s.nanosec < 1_000_000_000
            ):
                return None
            value = s.sec * 1_000_000_000 + s.nanosec

            return value if value > 0 else None
        except (AttributeError, TypeError):
            return None

    @staticmethod
    def usable(scan):

        try:
            if (
                not scan.header.frame_id
                or not scan.ranges
                or not all(
                    math.isfinite(v)
                    for v in (
                        scan.angle_min,
                        scan.angle_max,
                        scan.angle_increment,
                        scan.range_min,
                        scan.range_max,
                        scan.time_increment,
                        scan.scan_time,
                    )
                )
                or scan.angle_increment == 0
                or scan.range_min < 0
                or scan.range_max <= scan.range_min
                or scan.time_increment < 0
                or scan.scan_time < 0
            ):
                return False
            last_angle = scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1)
            tolerance = max(1e-5, abs(scan.angle_increment) * 1e-3)
            if not math.isfinite(last_angle) or abs(last_angle - scan.angle_max) > tolerance:
                return False
            usable = False
            for ray in scan.ranges:
                if math.isinf(ray):
                    if ray < 0:
                        return False
                    usable = True
                elif math.isfinite(ray) and scan.range_min <= ray <= scan.range_max:
                    usable = True
            return usable
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    def receive(self, scan, now_ns, wall):
        clock_valid = self.check_time(now_ns, wall)
        if not clock_valid:
            return
        stamp = self.stamp_ns(scan)
        if stamp is None or not self.usable(scan) or abs(now_ns - stamp) >= self.timeout_ns:

            if stamp is not None and abs(now_ns - stamp) < self.timeout_ns:
                self.last_seen_ns = max(self.last_seen_ns, stamp)
            self.pending = None
            return
        if stamp <= self.last_seen_ns:
            return
        self.last_seen_ns = stamp
        if self.pending is None or not self.fresh(self.pending[1], self.pending[2], now_ns, wall):
            self.pending = (scan, stamp, wall)

    def fresh(self, stamp, receipt, now_ns, wall):
        return abs(now_ns - stamp) < self.timeout_ns and 0 <= wall - receipt < self.timeout_s

    def poll(self, now_ns, wall, exact_tf_ready):
        if not self.check_time(now_ns, wall) or self.pending is None:
            return False
        scan, stamp, receipt = self.pending
        if not self.fresh(stamp, receipt, now_ns, wall):
            self.pending = None
            return False
        try:
            return bool(exact_tf_ready(scan))
        except Exception:
            return False


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rcl_interfaces.msg import ParameterDescriptor
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node("navigation_localization_readiness")
    timeout = node.declare_parameter("timeout_s", 0.0, ParameterDescriptor(read_only=True)).value
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    limits = safety_defaults()
    scans = ScanReadiness(limits["scan_timeout_s"])
    subscription = node.create_subscription(
        LaserScan,
        "/scan",
        lambda m: scans.receive(m, node.get_clock().now().nanoseconds, time.monotonic()),
        qos_profile_sensor_data,
    )

    def exact_tf_ready(scan):
        stamp = Time.from_msg(scan.header.stamp)
        if not buffer.can_transform("odom", scan.header.frame_id, stamp):
            return False
        if not buffer.can_transform("map", "base_footprint", Time()):
            return False
        tf = buffer.lookup_transform("map", "base_footprint", Time())
        return (
            abs((node.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds)
            < limits["localization_tf_timeout_s"] * 1_000_000_000
        )

    start = time.monotonic()
    ready = False
    node.get_logger().info(
        "Waiting for live scan and map localization; fixed maps require an initial pose."
    )
    while rclpy.ok() and (timeout <= 0 or time.monotonic() - start < timeout):
        rclpy.spin_once(node, timeout_sec=0.05)
        if scans.poll(node.get_clock().now().nanoseconds, time.monotonic(), exact_tf_ready):
            ready = True
            break
    node.destroy_node()
    rclpy.try_shutdown()
    if not ready:
        raise RuntimeError("Navigation localization readiness interrupted or timed out")


if __name__ == "__main__":
    main()
