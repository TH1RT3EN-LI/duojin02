import json,time,sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import String
rclpy.init();n=Node('command_chain_observer',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
records=[];subs=[]
for topic in ['/cmd_vel_nav','/cmd_vel','/cmd_vel_requested','/cmd_vel_guarded','/cmd_vel_base']:
 def cb(m,t=topic):records.append(dict(t=n.get_clock().now().nanoseconds/1e9,topic=t,x=m.linear.x,y=m.linear.y,w=m.angular.z))
 subs.append(n.create_subscription(Twist,topic,cb,100))
subs.append(n.create_subscription(String,'/navigation/guard_status',lambda m:records.append(dict(t=n.get_clock().now().nanoseconds/1e9,topic='guard',data=json.loads(m.data))),100))
end=time.monotonic()+70
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
open(sys.argv[1],'w').write(json.dumps(records))
n.destroy_node();rclpy.shutdown()
