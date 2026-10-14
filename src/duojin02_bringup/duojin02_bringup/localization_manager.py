"""Own fixed-map localization permission, recovery and navigation command lease.

No truth, odometry reset, map-origin adaptation, or robot motion is used. Global
localization can fail in ambiguous scenes: failure keeps navigation closed.
"""
from dataclasses import fields
import hashlib
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.impl.implementation_singleton import rclpy_implementation
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from duojin02_bringup.localization_health import (
    HealthPolicy, HealthState, MapEvidence, PARAMETER_DEFAULTS)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CommandPollingGroup(CallbackGroup):
    """Take the command with DDS source time on the manager's steady tick."""

    def can_execute(self, entity):
        return False

    def beginning_execution(self, entity):
        return False

    def ending_execution(self, entity):
        pass


class LocalizationManager(Node):
    def __init__(self):
        super().__init__('localization_manager')
        defaults = HealthPolicy()
        policy = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value for f in fields(defaults)}
        self.policy = HealthPolicy(**policy)
        for key in ('hit_fraction', 'known_fraction', 'max_contradiction_fraction'):
            if not 0 <= policy[key] <= 1:
                raise ValueError(f'{key} must be in [0,1]')
        if min(policy['min_hits'], policy['max_beams'], policy['max_ray_cells'], policy['good_scans'], policy['bad_scans'], policy['recovery_scans']) < 1 or policy['scan_timeout_s'] <= 0:
            raise ValueError('Evidence windows, work budget and timeout must be positive')
        if (not all(math.isfinite(float(value)) for value in policy.values()) or
                min(policy['good_span_s'],policy['bad_span_s'],policy['recovery_span_s'],policy['endpoint_tolerance'],policy['max_range'])<=0 or
                policy['ray_margin'] < policy['endpoint_tolerance']):
            raise ValueError('Evidence time spans, tolerances and range must be finite and positive')
        self.state = HealthState(self.policy)
        self.map_frame = self.declare_parameter('map_frame', PARAMETER_DEFAULTS['map_frame']).value
        self.convention = self.declare_parameter('map_cell_convention', PARAMETER_DEFAULTS['map_cell_convention']).value
        self.recovery_timeout = self.declare_parameter('recovery_attempt_timeout_s', PARAMETER_DEFAULTS['recovery_attempt_timeout_s']).value
        self.max_attempts = self.declare_parameter('recovery_max_attempts', PARAMETER_DEFAULTS['recovery_max_attempts']).value
        if not 1 <= self.max_attempts <= 3 or not 0 < self.recovery_timeout <= 60:
            raise ValueError('Recovery must have 1..3 finite attempts of at most 60 seconds')
        self.grid, self.pending, self.evidence = None, None, {}
        self.pending_tf_wait = False
        self.map_signature = None
        self.processed = self.received = 0
        self.pose_stamp = None
        self.recovery_pose_baseline = None
        self.pose_rx_wall = None
        self.attempts, self.recovery_phase, self.deadline = 0, None, None
        self.recovery_ack_wall = None
        self.global_future = self.nomotion_future = None
        self.last_nomotion = 0.0
        self.recovery_mode = None
        self.session, self.sequence = str(uuid.uuid4()), 0
        self.recovery_sensor_cutoff = None
        self.command_timeout = self.declare_parameter('command_timeout_s', PARAMETER_DEFAULTS['command_timeout_s']).value
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError('command_timeout_s must be finite and positive')
        self.nav_allowed = self.nav_active = False
        self.nav_command = self.nav_requested_wall = None
        self.nav_epoch_ns = time.time_ns()
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.request_subscription = self.create_subscription(
            Twist, '/nav/localization_requested', lambda _: None, 1,
            callback_group=CommandPollingGroup())
        self.score_due = True
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(Bool, '/localization/healthy', durable)
        self.status_pub = self.create_publisher(String, '/localization/status', durable)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        global_name = self.declare_parameter('global_localization_service', PARAMETER_DEFAULTS['global_localization_service']).value
        nomotion_name = self.declare_parameter('nomotion_update_service', PARAMETER_DEFAULTS['nomotion_update_service']).value
        self.global_client = self.create_client(Empty, global_name)
        self.nomotion_client = self.create_client(Empty, nomotion_name)
        self.create_service(Trigger, '/localization/recover', self.recover)
        # Interleave the two 20 Hz duties. Scan scoring does not sit directly
        # before every second command publication and add its cost as jitter.
        self.create_timer(0.025, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def on_map(self, msg):
        try:
            if msg.header.frame_id != self.map_frame:
                raise ValueError('Map frame does not match configured map_frame')
            q = msg.info.origin.orientation
            if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or not math.isfinite(q.w) or not math.isfinite(q.z) or abs(q.w*q.w + q.z*q.z - 1.0) > 1e-3:
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
            self.state.invalidate('map_received_or_changed', latch=self.state.last_stamp is not None)
        except ValueError as error:
            self.grid = None
            self.map_signature = None
            self.state.invalidate(str(error), latch=True)
            self.get_logger().error(str(error))
        self.publish()

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, *msg.pose.covariance)
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (msg.header.frame_id != self.map_frame or not all(math.isfinite(v) for v in values) or
                abs(sum(v*v for v in (q.x,q.y,q.z,q.w)) - 1.0) > 1e-3 or
                any(msg.pose.covariance[i] < 0 for i in (0,7,14,21,28,35)) or
                not -0.05 <= age <= self.policy.scan_timeout_s or stamp <= 0 or
                self.pose_stamp is not None and stamp <= self.pose_stamp):
            return
        self.pose_stamp, self.pose_rx_wall = stamp, time.monotonic()

    def on_scan(self, msg):
        self.received += 1
        same_pending = (self.pending is not None and
                        msg.header.frame_id == self.pending.header.frame_id and
                        msg.header.stamp.sec == self.pending.header.stamp.sec and
                        msg.header.stamp.nanosec == self.pending.header.stamp.nanosec)
        self.pending_tf_wait = self.pending_tf_wait and same_pending
        self.pending = msg  # Do not build up stale scans while a transform is missing.

    def on_initialpose(self, msg):
        # A new operator-supplied pose is an explicit recovery attempt, not proof.
        # Initial startup initialization still uses the normal evidence window.
        if self.grid is None or self.state.state in ('WAITING', 'SUSPECT'):
            return
        if msg.header.frame_id != self.map_frame:
            return
        self.state.start_recovery()
        self.recovery_mode = 'manual_pose'
        self.recovery_pose_baseline = self.pose_stamp
        self.recovery_ack_wall = time.monotonic()
        self.recovery_sensor_cutoff = self.get_clock().now().nanoseconds / 1e9
        self.attempts = 1
        self.recovery_phase, self.deadline = 'VERIFY_SCANS', self.recovery_ack_wall + self.recovery_timeout
        self.publish()

    def recover(self, _request, response):
        if self.recovery_phase is not None:
            response.success, response.message = False, 'Recovery already running; navigation remains stopped'
            return response
        if self.grid is None:
            response.success, response.message = False, 'No supported saved map available'
            return response
        self.attempts = 0
        self.recovery_mode = 'global_service'
        self.begin_attempt()
        response.success, response.message = True, 'Recovery accepted; observe /localization/status; acceptance is not success'
        return response

    def begin_attempt(self):
        self.attempts += 1
        self.state.start_recovery()
        self.recovery_pose_baseline = self.pose_stamp
        self.recovery_ack_wall = None
        self.recovery_phase, self.deadline = 'WAIT_SERVICE', time.monotonic() + 2.0
        self.publish()

    def fail_attempt(self, reason):
        self.global_future = None
        if self.recovery_mode == 'global_service' and self.attempts < self.max_attempts:
            self.begin_attempt()
        else:
            self.recovery_phase = None
            self.state.recovery_failed(reason)
            self.publish()

    def recovery_tick(self, wall):
        if self.recovery_phase is None:
            return
        if wall > self.deadline:
            self.fail_attempt('bounded_global_recovery_timeout:' + self.recovery_phase)
            return
        if self.state.healthy:
            self.recovery_phase = None
            self.get_logger().info('Recovery confirmed by distinct scans and new localizer observations')
            return
        if self.recovery_phase == 'WAIT_SERVICE' and self.global_client.service_is_ready():
            try:
                self.global_future = self.global_client.call_async(Empty.Request())
                self.recovery_phase, self.deadline = 'WAIT_RESPONSE', wall + 3.0
            except Exception as error:
                self.fail_attempt('global_localization_service_failed:' + str(error))
        elif self.recovery_phase == 'WAIT_RESPONSE' and self.global_future.done():
            try:
                if self.global_future.result() is None:
                    raise RuntimeError('Empty global-localization response')
            except Exception as error:
                self.fail_attempt('global_localization_service_failed:' + str(error))
                return
            self.recovery_ack_wall = wall
            # Only source-time measurements after the response epoch count;
            # queued pre-reset pose messages cannot satisfy confirmation.
            self.recovery_sensor_cutoff = self.get_clock().now().nanoseconds / 1e9
            self.state.reset_windows()  # Pre-response scans cannot confirm the new hypothesis.
            self.recovery_phase, self.deadline = 'VERIFY_SCANS', wall + self.recovery_timeout
        elif self.recovery_phase == 'VERIFY_SCANS':
            # AMCL ordinarily skips stationary measurement updates. Request
            # bounded fresh measurements without commanding vehicle motion.
            if wall - self.last_nomotion >= 0.35 and self.nomotion_client.service_is_ready() and (self.nomotion_future is None or self.nomotion_future.done()):
                try:
                    self.nomotion_future = self.nomotion_client.call_async(Empty.Request())
                except Exception as error:
                    self.state.reason = 'nomotion_update_unavailable:' + str(error)
                self.last_nomotion = wall

    def tick(self):
        wall = time.monotonic()
        if not self.score_due:
            self.score_due = True
            self.state.tick(wall)
            self.update_permission()
            self.forward_command()
            return
        self.score_due = False
        if self.pending is not None and self.grid is not None:
            msg = self.pending
            raw_stamp = msg.header.stamp
            if raw_stamp.sec < 0 or not 0 <= raw_stamp.nanosec < 1_000_000_000:
                self.pending = None
                self.state.invalidate('stale_or_invalid_sensor_stamp', latch=True)
            else:
                stamp = Time.from_msg(raw_stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                self.process_pending(msg, stamp, age, wall)
        wall = time.monotonic()  # Scoring must not hide a crossed watchdog or recovery deadline.
        self.state.tick(wall)
        self.recovery_tick(wall)
        self.publish()

    def clear_command(self):
        if self.nav_active:
            self.command_pub.publish(Twist())
        self.nav_active = False
        self.nav_command = self.nav_requested_wall = None

    def update_permission(self):
        ros_now = self.get_clock().now().nanoseconds / 1e9
        allowed = (self.state.healthy and
                   -0.05 <= ros_now - self.state.last_stamp <= self.policy.scan_timeout_s)
        if allowed != self.nav_allowed:
            self.clear_command()
            self.nav_allowed = allowed
            self.nav_epoch_ns = time.time_ns()

    def forward_command(self):
        with self.request_subscription.handle:
            request = self.request_subscription.handle.take_message(Twist, False)
        wall = time.monotonic()
        if request is not None:
            command, info = request
            source_ns = info['source_timestamp']
            source_age = (time.time_ns() - source_ns) / 1e9
            if (not self.nav_allowed or source_ns <= self.nav_epoch_ns or
                    not -0.05 <= source_age <= self.command_timeout or
                    not any((command.linear.x, command.linear.y, command.linear.z,
                             command.angular.x, command.angular.y, command.angular.z))):
                self.clear_command()
                return
            self.nav_command, self.nav_requested_wall = command, wall
            self.nav_active = True
        if not self.nav_active:
            return
        if wall - self.nav_requested_wall > self.command_timeout:
            self.clear_command()
            return
        self.command_pub.publish(self.nav_command)

    def process_pending(self, msg, stamp, age, wall):
        # Check every received payload before classifying a source-time gap.
        # A malformed duplicate must not inherit a prior scan's TF wait status.
        try:
            expected_max = msg.angle_min + (len(msg.ranges)-1) * msg.angle_increment
            if (not msg.header.frame_id or not msg.ranges or
                    not all(math.isfinite(v) for v in (msg.angle_min,msg.angle_max,msg.angle_increment,msg.range_min,msg.range_max)) or
                    msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max or
                    not math.pi <= abs(msg.angle_increment)*(len(msg.ranges)-1) <= 2*math.pi + 0.05 or
                    abs(msg.angle_max-expected_max) > max(1e-4,abs(msg.angle_increment)*0.05) or
                    not all(math.isfinite(v) and v >= 0 for v in (msg.scan_time,msg.time_increment))):
                raise ValueError('Inconsistent LaserScan metadata')
            if any(math.isnan(r) or r == -math.inf or r < msg.range_min for r in msg.ranges):
                raise ValueError('Invalid LaserScan ray')
        except (ValueError, TypeError, OverflowError) as error:
            self.pending = None
            self.state.invalidate('invalid_scan_or_transform:' + str(error), latch=True)
            return
        if age > self.policy.scan_timeout_s and self.pending_tf_wait:
            # This scan was fresh on its first attempt; only its exact TF
            # failed to arrive. Drop it and require a new evidence window.
            self.pending = None
            self.state.perception_stale('exact_transform_timeout')
        elif abs(age) > self.policy.scan_timeout_s or stamp.nanoseconds <= 0:
            self.pending = None
            self.state.invalidate('stale_or_invalid_sensor_stamp', latch=True)
        elif self.state.last_stamp is not None and stamp.nanoseconds / 1e9 < self.state.last_stamp:
            self.pending = None
            self.state.invalidate('sensor_stamp_rewind', latch=True)
        elif (self.state.state == 'RECOVERING' and self.recovery_phase == 'VERIFY_SCANS' and
                stamp.nanoseconds / 1e9 <= self.recovery_sensor_cutoff):
            # A pre-response scan may receive its exact TF after the response,
            # or arrive late. Neither may start the post-response dwell window.
            self.pending = None
        elif self.state.last_stamp is None or stamp.nanoseconds / 1e9 > self.state.last_stamp:
            try:
                tf = self.buffer.lookup_transform(self.map_frame, msg.header.frame_id, stamp)
                p, q = tf.transform.translation, tf.transform.rotation
                if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1.0)>1e-3:
                    raise ValueError('Invalid transform values')
                evidence = self.grid.score(msg.ranges, msg.angle_min, msg.angle_increment,
                                           msg.range_min, msg.range_max, (p.x, p.y, yaw(q)), self.policy)
                recovery_pose = None
                if (self.recovery_phase == 'VERIFY_SCANS' and self.pose_rx_wall is not None and
                        self.pose_rx_wall >= self.recovery_ack_wall and
                        self.recovery_sensor_cutoff < self.pose_stamp <= stamp.nanoseconds / 1e9 and
                        stamp.nanoseconds / 1e9 - self.pose_stamp <= self.policy.scan_timeout_s and
                        (self.recovery_pose_baseline is None or self.pose_stamp > self.recovery_pose_baseline)):
                    recovery_pose = self.pose_stamp
                # Nothing before global-service acknowledgement can unlock.
                if self.state.state != 'RECOVERING' or self.recovery_phase == 'VERIFY_SCANS':
                    self.state.observe(stamp.nanoseconds / 1e9, wall, evidence, recovery_pose)
                self.evidence, self.pending = evidence, None
                self.processed += 1
            except TransformException:
                self.pending_tf_wait = True  # Exact-stamp TF gets a bounded opportunity to arrive.
            except (ValueError, TypeError, OverflowError) as error:
                self.pending = None
                self.state.invalidate('invalid_scan_or_transform:' + str(error), latch=True)
        else:
            self.pending = None

    def publish(self):
        self.update_permission()
        self.sequence += 1
        self.health_pub.publish(Bool(data=self.state.healthy))
        self.status_pub.publish(String(data=json.dumps(dict(state=self.state.state,
            session=self.session, sequence=self.sequence, emitted_ros_ns=self.get_clock().now().nanoseconds,
            healthy=self.state.healthy, reason=self.state.reason, scan_stamp=self.state.last_stamp,
            received_scans=self.received, processed_scans=self.processed,
            distinct_good_scans=len(self.state.good), recovery_pose_observations=len(self.state.recovery_observations),
            recovery_phase=self.recovery_phase, recovery_attempt=self.attempts,
            recovery_mode=self.recovery_mode,
            map_cell_convention=self.convention, evidence=self.evidence), sort_keys=True)))


def main():
    rclpy.init()
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.state.invalidate('manager_stopping', latch=True)
        try:
            if node.context.ok():
                try:
                    node.publish()
                except rclpy_implementation.RCLError:
                    # SIGINT can close the context between the check and a
                    # publication. A live-context failure must still surface.
                    if node.context.ok():
                        raise
        finally:
            node.destroy_node()
            rclpy.try_shutdown(context=node.context)


if __name__ == '__main__':
    main()
