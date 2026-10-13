"""Verify compiled map-prior indexing with rotated localization and negative bins."""
import json,math,os,signal,subprocess,time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='94';os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import TransformStamped,Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
rclpy.init();n=rclpy.create_node('prior_index_probe');status=[]
sub=n.create_subscription(String,'/navigation/guard_status',lambda m:status.append(json.loads(m.data)),10)
cmd=n.create_publisher(Twist,'/cmd_vel_requested',10);odom=n.create_publisher(Odometry,'/odometry/filtered',10)
scan=n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
prior=n.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
broadcaster=TransformBroadcaster(n)
angle=.73;tx=-1.2;ty=.8;c=math.cos(angle);s=math.sin(angle)
log=open('/tmp/prior-index-probe.log','w');p=subprocess.Popen(['ros2','run','duojin01_navigation','velocity_guard','--ros-args','--params-file',str(Path(__file__).resolve().parents[2]/'src/duojin01_bringup/config/velocity_guard.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg();tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='odom';tf.child_frame_id='map'
  tf.transform.translation.x=tx;tf.transform.translation.y=ty;tf.transform.rotation.z=math.sin(angle/2);tf.transform.rotation.w=math.cos(angle/2);broadcaster.sendTransform(tf)
  o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint';o.pose.pose.orientation.w=1.;odom.publish(o)
  laser=LaserScan();laser.header.stamp=stamp;laser.header.frame_id='odom';laser.range_min=.02;laser.range_max=8.;laser.angle_increment=.01;laser.ranges=[1.5];laser.angle_max=laser.angle_min;scan.publish(laser)
  v=Twist();v.linear.x=.3;cmd.publish(v);rclpy.spin_once(n,timeout_sec=.02)
def map_point(x,y,res):
 qx=c*(x-tx)+s*(y-ty);qy=-s*(x-tx)+c*(y-ty)
 m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.info.width=m.info.height=1;m.info.resolution=res
 m.info.origin.position.x=qx-res/2;m.info.origin.position.y=qy-res/2;m.info.origin.orientation.w=1.;m.data=[100];prior.publish(m)
try:
 pump(2.)
 records=[]
 for x,y,res,expected in [(1.,0.,.01,'clear'),(.10,.05,.01,'measured_braking_collision'),(-.1,-.05,.03,'measured_braking_collision'),(.35,0.,.01,'braking_slowdown'),(2.51,0.,.01,'clear')]:
  map_point(x,y,res);pump(.6);assert status[-1]['reason']==expected,status[-1];records.append(dict(point=[x,y],resolution=res,reason=status[-1]['reason']))
 print(json.dumps(dict(checks=5,rotation_rad=angle,translation_m=[tx,ty],records=records)))
finally:
 os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5);log.close();n.destroy_node();rclpy.shutdown()
