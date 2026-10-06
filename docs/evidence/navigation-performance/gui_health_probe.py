import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
rclpy.init();n=Node('navigation_delivery_health')
result=dict(rmw_implementation=get_rmw_implementation_identifier());latest={}
def scan(m):latest['scan']=dict(frame=m.header.frame_id,count=len(m.ranges),range_min=m.range_min,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
def grid(m):latest['map']=dict(resolution=m.info.resolution,width=m.info.width,height=m.info.height)
def guard(m):latest['guard']=json.loads(m.data)
n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(String,'/navigation/guard_status',guard,10)
for name in ['controller_server','planner_server','bt_navigator','collision_monitor','velocity_smoother']:
 c=n.create_client(GetState,'/'+name+'/get_state');assert c.wait_for_service(timeout_sec=10),name
 f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=10)
 assert f.done() and f.result() and f.result().current_state.id==3,name
 result[name]=f.result().current_state.label
for name,keys in [('controller_server',['use_sim_time','FollowPath.vx_max','FollowPath.vy_max','general_goal_checker.plugin','general_goal_checker.settle_time']),('velocity_smoother',['feedback','max_velocity']),('slam_toolbox',['resolution','minimum_travel_distance','use_scan_barycenter'])]:
 c=n.create_client(GetParameters,'/'+name+'/get_parameters');assert c.wait_for_service(timeout_sec=10),name
 req=GetParameters.Request();req.names=keys;f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10);assert f.done() and f.result()
 values={}
 for key,value in zip(keys,f.result().values):
  values[key]=value.bool_value if value.type==1 else value.integer_value if value.type==2 else value.double_value if value.type==3 else value.string_value if value.type==4 else list(value.double_array_value) if value.type==8 else str(value)
 result[name+'_parameters']=values
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
result.update(latest);result['nodes']=[name for name,namespace in n.get_node_names_and_namespaces()]
assert result['rmw_implementation']=='rmw_cyclonedds_cpp'
assert result['controller_server_parameters']['general_goal_checker.plugin']=='duojin01_navigation::StableStoppedGoalChecker'
assert result['controller_server_parameters']['general_goal_checker.settle_time']==.25
assert result['controller_server_parameters']['FollowPath.vx_max']==.5
assert result['velocity_smoother_parameters']['feedback']=='OPEN_LOOP'
assert result['slam_toolbox_parameters']['minimum_travel_distance']==.15
assert result['slam_toolbox_parameters']['use_scan_barycenter'] is False
assert 'amcl' not in result['nodes'] and 'map_server' not in result['nodes']
assert result['scan']['count']==540 and abs(result['map']['resolution']-.01)<1e-6
print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
