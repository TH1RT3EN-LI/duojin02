"""Wait for live perception and localization before starting navigation servers."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node=Node('navigation_localization_readiness')
    timeout=node.declare_parameter('timeout_s',0.0).value
    buffer=Buffer();listener=TransformListener(buffer,node)
    latest=[]
    subscription=node.create_subscription(LaserScan,'/scan',lambda m:latest.__setitem__(slice(None),[m]),qos_profile_sensor_data)
    start=time.monotonic();ready=False;stable=0
    node.get_logger().info('Waiting for live scan and map localization; fixed maps require an initial pose.')
    while rclpy.ok() and (timeout<=0 or time.monotonic()-start<timeout):
        rclpy.spin_once(node,timeout_sec=.05)
        valid=False
        if latest:
            stamp=Time.from_msg(latest[0].header.stamp)
            if 0<node.get_clock().now().nanoseconds and abs((node.get_clock().now()-stamp).nanoseconds)<300_000_000:
                if buffer.can_transform('odom',latest[0].header.frame_id,stamp) and buffer.can_transform('map','base_footprint',Time()):
                    tf=buffer.lookup_transform('map','base_footprint',Time())
                    valid=abs((node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds)<350_000_000
        stable=stable+1 if valid else 0
        if stable>=3:ready=True;break
    node.destroy_node();rclpy.try_shutdown()
    if not ready:raise RuntimeError('Navigation localization readiness interrupted or timed out')


if __name__=='__main__':main()
