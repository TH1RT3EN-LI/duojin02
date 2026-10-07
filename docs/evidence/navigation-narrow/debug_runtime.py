import json,time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
rclpy.init();n=Node('navigation_debug_runtime');values={};b=Buffer();listener=TransformListener(b,n)
def grid(m):
 values['grid']=dict(frame=m.header.frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,origin=[m.info.origin.position.x,m.info.origin.position.y],width=m.info.width*m.info.resolution,height=m.info.height*m.info.resolution,cells=len(m.data),lethal=sum(x==100 for x in m.data),unknown=sum(x==-1 for x in m.data))
def odom(m):values['odom']=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,x=m.pose.pose.position.x,y=m.pose.pose.position.y,vx=m.twist.twist.linear.x,vy=m.twist.twist.linear.y)
n.create_subscription(OccupancyGrid,'/local_costmap/costmap',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Odometry,'/odometry/filtered',odom,10)
for topic in ['/cmd_vel_nav','/cmd_vel_requested','/cmd_vel_base']:
 n.create_subscription(Twist,topic,lambda m,t=topic:values.update({t:[m.linear.x,m.linear.y,m.angular.z]}),10)
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
for to,frm in [('odom','base_footprint'),('odom','map')]:
 try:
  t=b.lookup_transform(to,frm,Time());values[to+'<-'+frm]=dict(stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9,x=t.transform.translation.x,y=t.transform.translation.y)
 except Exception as e:values[to+'<-'+frm]=str(e)
print(json.dumps(values,indent=2));n.destroy_node();rclpy.shutdown()
