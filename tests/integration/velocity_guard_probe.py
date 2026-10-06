"""Exercise the compiled safety node with sensor faults; no physical devices."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def run():
    os.environ['ROS_DOMAIN_ID']='92';os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=Node('velocity_guard_probe')
    status=[];output=[]
    node.create_subscription(String,'/navigation/guard_status',lambda m:status.append(json.loads(m.data)),10)
    node.create_subscription(Twist,'/cmd_vel_guarded',output.append,10)
    cmd=node.create_publisher(Twist,'/cmd_vel_requested',10)
    odom=node.create_publisher(Odometry,'/odometry/filtered',10)
    scan=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    grid=node.create_publisher(OccupancyGrid,'/local_costmap/costmap',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    covariance=node.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    parameters=Path(__file__).resolve().parents[2]/'src/duojin01_bringup/config/velocity_guard.yaml'
    with open('/tmp/velocity-guard-probe.log','w') as log:
        process=subprocess.Popen(['ros2','run','duojin01_navigation','velocity_guard','--ros-args','--params-file',str(parameters)],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        def pump(duration,point=(1.5,0),requested=.3,actual=0.0,send_scan=True,send_command=True,scan_min=.02):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                stamp=node.get_clock().now().to_msg()
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint'
                o.pose.pose.orientation.w=1.0;o.twist.twist.linear.x=actual;odom.publish(o)
                if send_scan:
                    s=LaserScan();s.header.stamp=stamp;s.header.frame_id='odom';s.range_min=scan_min;s.range_max=8.0
                    s.angle_min=float(math.atan2(point[1],point[0])) if point else 0.0;s.angle_increment=.01
                    s.ranges=[float(math.hypot(*point)) if point else math.inf];scan.publish(s)
                if send_command:
                    v=Twist();v.linear.x=requested;cmd.publish(v)
                rclpy.spin_once(node,timeout_sec=.02)
        try:
            deadline=time.monotonic()+10
            while node.count_subscribers('/scan')==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            assert node.count_subscribers('/scan')
            pump(.5);assert status[-1]['reason']=='clear' and output[-1].linear.x==.3,status[-1]
            pump(.5,point=(.35,0));assert status[-1]['reason']=='braking_slowdown' and 0<output[-1].linear.x<.3,status[-1]
            pump(.5,point=(.241,0))
            assert status[-1]['reason']=='braking_slowdown' and 0<output[-1].linear.x<=.04,status[-1]
            pump(.5,point=(.10,0));assert status[-1]['reason']=='measured_braking_collision' and output[-1].linear.x==0,status[-1]
            # Fresh scan heartbeats with blind / invalid rays must not erase a
            # previously seen obstacle after a short retention timeout.
            pump(.8,point=None,scan_min=.30)
            assert status[-1]['reason']=='measured_braking_collision' and output[-1].linear.x==0,status[-1]
            pump(.5,point=(.36,0),actual=.35);assert status[-1]['reason']=='measured_braking_collision' and output[-1].linear.x==0,status[-1]
            pump(.5);pump(.4,send_command=False);assert status[-1]['reason']=='command_stale' and output[-1].linear.x==0,status[-1]
            pump(.5);pump(.4,send_scan=False);assert status[-1]['reason']=='scan_or_tf_stale' and output[-1].linear.x==0,status[-1]
            pump(.5)
            m=OccupancyGrid();m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg()
            m.info.width=m.info.height=1;m.info.resolution=.01;m.info.origin.position.x=.50;m.info.origin.orientation.w=1.0;m.data=[100]
            c=PoseWithCovarianceStamped();c.pose.covariance[0]=c.pose.covariance[7]=.25;covariance.publish(c)
            grid.publish(m);pump(.3)
            assert status[-1]['reason']=='clear' and output[-1].linear.x==.3,status[-1]
            m.header.stamp=node.get_clock().now().to_msg();m.info.origin.position.x=.10
            grid.publish(m);pump(.3)
            assert status[-1]['reason']=='measured_braking_collision' and output[-1].linear.x==0,status[-1]
            pump(.5);assert status[-1]['reason']=='costmap_stale' and output[-1].linear.x==0,status[-1]
            print(json.dumps({'checks':11,'filled_scan_interior':True,'filled_map_interior':True,'braking_slowdown':True,
                              'core_validated_boundary_crawl':True,
                              'live_observation_not_amcl_inflated':True,
                              'blind_obstacle_memory':True,'positive_free_ray_clearing':True,
                              'measured_speed_braking':True,'command_dropout':True,'scan_dropout':True,'map_dropout':True}))
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':run()
