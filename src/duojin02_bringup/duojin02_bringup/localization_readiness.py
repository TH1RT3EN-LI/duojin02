"""Wait for distinct live scans and localization before starting navigation."""
from collections import deque
import json
import math
import time


class ScanReadiness:
    """Give a new scan bounded time to receive its exact transform.

    A transient TF miss is pending evidence, not a failed observation. Source
    age and a steady-clock watchdog bound that wait. Stamp high-water marks
    survive faults so duplicate / replayed scans cannot rebuild readiness.
    """
    def __init__(self, max_pending=8):
        if max_pending < 1:
            raise ValueError('max_pending must be positive')
        self.max_pending = max_pending
        self.pending = deque()
        self.counted = 0
        self.last_seen_ns = -1
        self.last_clock_ns = None
        self.last_wall = None
        self.last_input_wall = None
        self.last_count_clock_ns = -1

    def reset(self, now_ns):
        self.pending.clear()
        self.counted = 0
        self.last_input_wall = None
        self.last_count_clock_ns = now_ns

    def check_time(self, now_ns, wall):
        valid = (isinstance(now_ns, int) and now_ns > 0 and
                 math.isfinite(wall))
        if (not valid or
            self.last_clock_ns is not None and now_ns < self.last_clock_ns or
            self.last_wall is not None and wall < self.last_wall):
            self.reset(now_ns)
        self.last_clock_ns, self.last_wall = now_ns, wall
        if self.last_input_wall is not None and wall-self.last_input_wall >= .3:
            self.reset(now_ns)
        return valid

    @staticmethod
    def stamp_ns(scan):
        try:
            s = scan.header.stamp
            if (not isinstance(s.sec, int) or not isinstance(s.nanosec, int) or
                    s.sec < 0 or not 0 <= s.nanosec < 1_000_000_000):
                return None
            value = s.sec*1_000_000_000+s.nanosec
            # Time(0) requests the latest transform rather than this sensor time.
            return value if value > 0 else None
        except (AttributeError, TypeError):
            return None

    @staticmethod
    def usable(scan):
        # Same scan-liveness contract as the downstream velocity guard:
        # +Inf is a legal no return; -Inf and wholly unusable rays fail closed.
        try:
            if (not scan.header.frame_id or not scan.ranges or
                not all(math.isfinite(v) for v in
                        (scan.angle_min, scan.angle_max, scan.angle_increment,
                         scan.range_min, scan.range_max, scan.time_increment,
                         scan.scan_time)) or scan.angle_increment == 0 or
                scan.range_min < 0 or scan.range_max <= scan.range_min or
                scan.time_increment < 0 or scan.scan_time < 0):
                return False
            last_angle = scan.angle_min+scan.angle_increment*(len(scan.ranges)-1)
            tolerance = max(1e-5, abs(scan.angle_increment)*1e-3)
            if not math.isfinite(last_angle) or abs(last_angle-scan.angle_max) > tolerance:
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
        stamp = self.stamp_ns(scan)
        if (not clock_valid or stamp is None or not self.usable(scan) or
                abs(now_ns-stamp) >= 300_000_000):
            # A malformed scan cannot bridge two qualifying runs. A valid
            # timestamp near current time also forms a replay barrier.
            if stamp is not None and abs(now_ns-stamp) < 300_000_000:
                self.last_seen_ns = max(self.last_seen_ns, stamp)
            self.reset(now_ns)
            return
        if stamp <= self.last_seen_ns:
            return
        self.last_seen_ns = stamp
        if len(self.pending) >= self.max_pending:
            self.reset(now_ns)
        self.pending.append((scan, stamp, wall))
        self.last_input_wall = wall

    def poll(self, now_ns, wall, exact_tf_ready):
        if not self.check_time(now_ns, wall):
            return False
        for _, stamp, receipt in self.pending:
            if abs(now_ns-stamp) >= 300_000_000 or wall-receipt >= .3:
                self.reset(now_ns)
                return False
        while self.pending:
            scan, _, _ = self.pending[0]
            # Paused ROS time cannot qualify a stream of future-stamped data.
            if now_ns <= self.last_count_clock_ns:
                break
            try:
                valid = exact_tf_ready(scan)
            except Exception:
                valid = False
            if not valid:
                break
            self.pending.popleft()
            self.counted += 1
            self.last_count_clock_ns = now_ns
            if self.counted >= 3:
                return True
        return False


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from duojin02_bringup.localization_health import HealthSignal

    rclpy.init()
    node=Node('navigation_localization_readiness')
    timeout=node.declare_parameter('timeout_s',0.0).value
    require_health=node.declare_parameter('require_health',False).value
    buffer=Buffer();listener=TransformListener(buffer,node)
    scans=ScanReadiness()
    subscription=node.create_subscription(LaserScan,'/scan',
        lambda m:scans.receive(m,node.get_clock().now().nanoseconds,time.monotonic()),
        qos_profile_sensor_data)
    health=[];signal=HealthSignal()
    def health_callback(msg):
        try:valid=signal.accept(json.loads(msg.data),node.get_clock().now().nanoseconds/1e9,time.monotonic())
        except (json.JSONDecodeError,TypeError):valid=False
        health.__setitem__(slice(None),[(valid,time.monotonic())])
    health_subscription=node.create_subscription(String,'/localization/status',health_callback,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def exact_tf_ready(scan):
        stamp=Time.from_msg(scan.header.stamp)
        if not buffer.can_transform('odom',scan.header.frame_id,stamp):
            return False
        if not buffer.can_transform('map','base_footprint',Time()):
            return False
        tf=buffer.lookup_transform('map','base_footprint',Time())
        return abs((node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds)<350_000_000

    start=time.monotonic();ready=False
    node.get_logger().info('Waiting for live scan and map localization; fixed maps require an initial pose.')
    while rclpy.ok() and (timeout<=0 or time.monotonic()-start<timeout):
        rclpy.spin_once(node,timeout_sec=.05)
        if require_health:
            # The persistent monitor already requires distinct sensor stamps
            # and geometric evidence; repeated heartbeats cannot advance it.
            if health and health[0][0] and time.monotonic()-health[0][1]<.35 and signal.fresh(node.get_clock().now().nanoseconds/1e9):
                ready=True;break
            continue
        if scans.poll(node.get_clock().now().nanoseconds,time.monotonic(),exact_tf_ready):
            ready=True;break
    node.destroy_node();rclpy.try_shutdown()
    if not ready:raise RuntimeError('Navigation localization readiness interrupted or timed out')


if __name__=='__main__':main()
