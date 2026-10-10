"""Start Nav2 after simulated sensor timestamps can actually be transformed."""
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node=Node('navigation_sensor_readiness',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    latest=[]
    node.create_subscription(LaserScan,'/scan',lambda m:latest.append(m) if not latest else latest.__setitem__(0,m),qos_profile_sensor_data)
    ready=False;deadline=time.monotonic()+75
    while rclpy.ok() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if latest and node.get_clock().now().nanoseconds>0 and buffer.can_transform('odom',latest[0].header.frame_id,Time.from_msg(latest[0].header.stamp)):
            ready=True;break
    node.destroy_node();rclpy.try_shutdown()
    if not ready:raise RuntimeError('Sensor / odom TF readiness timeout')
