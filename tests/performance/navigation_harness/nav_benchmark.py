#!/usr/bin/env python3
"""Paired real Gazebo/Nav2 trials. Ground truth is used only by this evaluator."""
import argparse
from collections import Counter, deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import statistics
import subprocess
import time
import numpy as np
from shapely.geometry import Polygon, box as rectangle
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.utilities import get_rmw_implementation_identifier
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowPath
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from duojin01_bringup.navigation_stack import NAVIGATION_NODES


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def scenarios():
    base=dict(start=[-1.7,0,0],goal=[1.7,0,0],timeout=55)
    result=[dict(base,name='open',goal=[1.7,.6,.35],obstacles=[])]
    for width in [.70,.56,.52,.50,.48,.46,.44,.42,.40]:
        result.append(dict(base,name=f'corridor_{width:.2f}',width=width,
            # Doorway spans the room: a route around the test aperture is impossible.
            obstacles=[[0,(2.5+width/2)/2,.60,2.5-width/2],
                       [0,-(2.5+width/2)/2,.60,2.5-width/2]]))
    result += [
      dict(base,name='precision_doorway_0.46',width=.46,map_resolution=.01,
           obstacles=[[0,(2.5+.23)/2,.60,2.5-.23],[0,-(2.5+.23)/2,.60,2.5-.23]]),
      dict(base,name='legacy_map_3cm',goal=[1.7,.6,.35],map_resolution=.03,obstacles=[]),
      dict(base,name='thin_obstacles',obstacles=[[-.6,.07,.015,.025],[.1,-.10,.018,.04],[.7,.14,.020,.04]]),
      dict(base,name='s_bend',obstacles=[[-.55,.65,.12,1.3],[.55,-.65,.12,1.3]]),
      dict(base,name='dynamic_crossing',obstacles=[],dynamic=True),
      dict(base,name='sensor_dropout',obstacles=[],dropout=True),
      dict(base,name='late_initial_pose',obstacles=[],initial_delay_s=8),
      dict(base,name='initial_pose_error',obstacles=[],initial_error=.20),
      dict(base,name='online_thin',obstacles=[[-.6,.07,.015,.025],[.1,-.10,.018,.04],[.7,.14,.020,.04]],online=True),
      dict(base,name='online_open',goal=[1.7,.6,.35],obstacles=[],online=True),
      dict(base,name='office_chicane',obstacles=[[-.8,.6,.25,1.0],[.1,-.6,.25,1.0],[1.0,.6,.25,1.0]]),
      dict(base,name='online_chicane',obstacles=[[-.8,.6,.25,1.0],[.1,-.6,.25,1.0],[1.0,.6,.25,1.0]],online=True),
      dict(base,name='online_corridor_0.50',width=.50,online=True,
           obstacles=[[0,1.375,.60,2.25],[0,-1.375,.60,2.25]]),
      dict(base,name='corridor_waypoints_0.50',width=.50,waypoints=[[-.7,0,0],[.7,0,0],[1.7,0,0]],
           obstacles=[[0,1.375,.60,2.25],[0,-1.375,.60,2.25]]),
      dict(base,name='reverse_goal',start=[1.7,0,0],goal=[-1.7,.5,0],obstacles=[]),
      dict(base,name='unsafe_fast_command',obstacles=[[-.60,0,.015,.025]],unsafe=True,unsafe_speed=.50,timeout=6),
      dict(base,name='unsafe_command',obstacles=[[-.60,0,.015,.025]],unsafe=True,timeout=6),
    ]
    return result


class Probe(Node):
    def __init__(self,case,footprint,seed):
        super().__init__('navigation_benchmark_evaluator', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.case,self.footprint,self.seed=case,footprint,seed
        self.tf=Buffer(cache_time=Duration(seconds=20));self.listener=TransformListener(self.tf,self)
        self.clock,self.truth,self.amcl=None,None,None
        self.samples,self.commands,self.scans,self.guard=[] ,[],[],[]
        self.command_chain=[]
        self.pending_tf=deque();self.tf_failures=Counter()
        self.guard_reasons=Counter();self.collect=False;self.collision_events=0;self.in_collision=False
        self.reference=[rectangle(o['x']-o['sx']/2,o['y']-o['sy']/2,o['x']+o['sx']/2,o['y']+o['sy']/2) for o in case['obstacle_reference']]
        self.reference += [rectangle(x-sx/2,y-sy/2,x+sx/2,y+sy/2) for x,y,sx,sy in case['oracle_walls']]
        self.dynamic_reference=None
        self.plans=[];self.stimuli=[]
        self.goal_submissions=0
        self.stimulus_time=None;self.stop_after_stimulus=None;self.resources=[];self.last_resource=0
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'clock',m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/sim/ground_truth/odom',self.receive_truth,10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:setattr(self,'amcl',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.commands.append(time.monotonic()) if self.collect else None,10)
        for topic in ('/cmd_vel_nav','/nav/localization_requested','/cmd_vel','/cmd_vel_requested','/cmd_vel_base'):
            self.create_subscription(Twist,topic,lambda m,t=topic:self.command_chain.append(dict(
                t=self.clock,wall_t=time.monotonic(),topic=t,x=m.linear.x,y=m.linear.y,yaw=m.angular.z)) if self.collect else None,20)
        self.create_subscription(LaserScan,'/scan',lambda m:self.scans.append(time.monotonic()) if self.collect else None,qos_profile_sensor_data)
        self.create_subscription(String,'/navigation/guard_status',self.receive_guard,10)
        self.create_subscription(NavPath,'/plan',lambda m:self.plans.append(dict(t=self.clock,poses=[[p.pose.position.x,p.pose.position.y,angle(p.pose.orientation)] for p in m.poses])) if self.collect else None,10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.motor=self.create_publisher(Int8,'/motor_control',10)
        self.forced_command=self.create_publisher(Twist,'/cmd_vel',10)
        self.action_type=FollowPath if case.get('controller_component') else (NavigateThroughPoses if case.get('waypoints') else NavigateToPose)
        self.action=ActionClient(self,self.action_type,'/follow_path' if case.get('controller_component') else ('/navigate_through_poses' if case.get('waypoints') else '/navigate_to_pose'))

    def receive_guard(self,m):
        if not self.collect:return
        data=json.loads(m.data);self.guard.append(data['compute_ms']);self.guard_reasons[data['reason']]+=1

    def receive_truth(self,m):
        self.truth=m
        if not self.collect:return
        if self.stimulus_time is not None and self.stop_after_stimulus is None and math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)<.005:
            self.stop_after_stimulus=self.clock-self.stimulus_time
        x,y=m.pose.pose.position.x,m.pose.pose.position.y;q=angle(m.pose.pose.orientation)
        obstacles=self.reference+([self.dynamic_reference] if self.dynamic_reference is not None else [])
        previous=self.samples[-1] if self.samples else dict(x=x,y=y,yaw=q)
        dx,dy=x-previous['x'],y-previous['y'];dq=math.remainder(q-previous['yaw'],2*math.pi)
        radius=max(math.hypot(*p) for p in self.footprint)
        steps=max(1,math.ceil((math.hypot(dx,dy)+radius*abs(dq))/.004))
        clearance=math.inf;collision=False
        for i in range(steps+1):
            f=i/steps;heading=previous['yaw']+f*dq;c,s=math.cos(heading),math.sin(heading)
            shape=Polygon([(previous['x']+f*dx+c*px-s*py,previous['y']+f*dy+s*px+c*py) for px,py in self.footprint])
            clearance=min(clearance,min(shape.distance(o) for o in obstacles))
            collision=collision or any(shape.intersects(o) for o in obstacles)
        if collision and not self.in_collision:self.collision_events+=1
        self.in_collision=collision
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.samples.append(dict(t=stamp,x=x,y=y,yaw=q,estimated_x=math.nan,estimated_y=math.nan,estimated_yaw=math.nan,estimated_odom_yaw=math.nan,
            roll=math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.x+m.pose.pose.orientation.y*m.pose.pose.orientation.z),1-2*(m.pose.pose.orientation.x**2+m.pose.pose.orientation.y**2)),
            pitch=math.asin(max(-1.,min(1.,2*(m.pose.pose.orientation.w*m.pose.pose.orientation.y-m.pose.pose.orientation.z*m.pose.pose.orientation.x)))),
            clearance=clearance,collision=collision,speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)))
        self.pending_tf.append((self.samples[-1],Time.from_msg(m.header.stamp)))
        self.resolve_estimates()

    def resolve_estimates(self,flush=False):
        # TF publication trails truth. Evaluate an earlier pose at exactly its own
        # timestamp, after both transforms arrive, never against latest TF.
        while self.pending_tf and (flush or self.clock-self.pending_tf[0][0]['t']>.25):
            row,stamp=self.pending_tf.popleft()
            try:
                tf=self.tf.lookup_transform('map','base_footprint',stamp)
                row.update(estimated_x=tf.transform.translation.x,estimated_y=tf.transform.translation.y,
                           estimated_yaw=angle(tf.transform.rotation))
                odom=self.tf.lookup_transform('odom','base_footprint',stamp)
                row['estimated_odom_yaw']=angle(odom.transform.rotation)
                if self.case.get('online'):
                    row['estimated_x']+=self.case['start'][0]
                    row['estimated_y']+=self.case['start'][1]
            except Exception as error:self.tf_failures[type(error).__name__]+=1

    def wait(self,predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if getattr(self, 'launch_process', None) is not None and self.launch_process.poll() is not None:
                raise RuntimeError('Launch exited while waiting for ROS nodes')
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate():return True
        return False

    def sample_resources(self):
        wall=time.monotonic()
        if wall-self.last_resource<.5:return
        self.last_resource=wall
        for directory in Path('/proc').iterdir():
            if not directory.name.isdigit():continue
            try:
                pid=int(directory.name)
                if os.getpgid(pid)!=self.launch_process.pid:continue
                arguments=(directory/'cmdline').read_bytes().split(b'\0')
                tracked=set(NAVIGATION_NODES)|{'amcl','async_slam_toolbox_node','velocity_guard','localization_manager','ekf_node'}
                name=next((Path(value.decode()).name for value in arguments[:2]
                           if Path(value.decode()).name in tracked),None)
                if name is None:continue
                fields=(directory/'stat').read_text().split(') ',1)[1].split()
                cpu=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
                rss=int(fields[21])*os.sysconf('SC_PAGE_SIZE')/2**20
                self.resources.append((name,wall,cpu,rss))
            except (FileNotFoundError,ProcessLookupError,PermissionError):pass

    def wait_navigation_active(self):
        # A discovered Action endpoint can belong to an INACTIVE server.
        # Require managed navigation nodes to be ACTIVE before sending a goal.
        deadline=time.monotonic()+60
        for name in NAVIGATION_NODES:
            client=self.create_client(GetState,'/'+name+'/get_state');active=False
            while time.monotonic()<deadline:
                if client.service_is_ready():
                    future=client.call_async(GetState.Request())
                    if self.wait(future.done,2) and future.result().current_state.id==3:active=True;break
                self.wait(lambda:False,.1)
            if not active:raise RuntimeError('Navigation lifecycle activation timeout: '+name)
        if not self.wait(self.action.server_is_ready,5):raise RuntimeError('Activated Nav2 action endpoint unavailable')

    def initialize(self):
        if not self.wait(lambda:self.clock is not None and self.truth is not None,75):
            raise RuntimeError('Sensor startup timeout')
        if self.case.get('online'):
            if not self.wait(lambda:self.tf.can_transform('map','base_footprint',Time()),20):
                raise RuntimeError('Online SLAM map transform startup timeout')
            if not self.wait(self.action.server_is_ready,30):raise RuntimeError('Online Nav2 action startup timeout')
            self.wait(lambda:False,2)
            return
        rng=random.Random(self.seed)
        lifecycle=self.create_client(GetState,'/amcl/get_state')
        active=False;deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            if lifecycle.service_is_ready():
                state=lifecycle.call_async(GetState.Request())
                if self.wait(state.done,2) and state.result().current_state.id==3:
                    active=True;break
            self.wait(lambda:False,.2)
        if not active or not self.wait(lambda:self.initial.get_subscription_count()>0,5):
            raise RuntimeError('AMCL lifecycle / initial-pose subscription not ready')
        if self.case.get('initial_delay_s'):self.wait(lambda:False,self.case['initial_delay_s'])
        initial=PoseWithCovarianceStamped();initial.header.frame_id='map'
        error=self.case.get('initial_error',.07)
        initial.pose.pose.position.x=self.case['start'][0]+rng.uniform(-error,error)
        initial.pose.pose.position.y=self.case['start'][1]+rng.uniform(-error,error)
        yaw=self.case['start'][2]+rng.uniform(-error,error)
        initial.pose.pose.orientation.z=math.sin(yaw/2);initial.pose.pose.orientation.w=math.cos(yaw/2)
        initial.pose.covariance[0]=initial.pose.covariance[7]=max(.01,error**2)
        initial.pose.covariance[35]=max(.01,error**2)
        for _ in range(5):
            initial.header.stamp=(self.get_clock().now()-Duration(seconds=.1)).to_msg();self.initial.publish(initial)
            self.wait(lambda:False,.2)
        deadline=time.monotonic()+15
        while self.amcl is None and time.monotonic()<deadline:
            initial.header.stamp=(self.get_clock().now()-Duration(seconds=.1)).to_msg();self.initial.publish(initial)
            self.wait(lambda:self.amcl is not None,.5)
        if self.amcl is None:raise RuntimeError('AMCL did not initialize')
        # Localization provides the map transform required to activate the
        # global costmap. Waiting for Nav2 first can deadlock startup.
        if not self.wait(self.action.server_is_ready,30):raise RuntimeError('Nav2 action startup timeout after localization')
        self.wait(lambda:False,1.0)

    def run(self):
        self.initialize();self.wait_navigation_active()
        x,y,yaw=map(float,self.case['goal'])
        def target(values):
            px,py,heading=map(float,values);pose=PoseStamped();pose.header.frame_id='map'
            pose.header.stamp=self.get_clock().now().to_msg()
            pose.pose.position.x=px-(self.case['start'][0] if self.case.get('online') else 0)
            pose.pose.position.y=py-(self.case['start'][1] if self.case.get('online') else 0)
            pose.pose.orientation.z=math.sin(heading/2);pose.pose.orientation.w=math.cos(heading/2)
            return pose
        goal=self.action_type.Goal()
        if self.case.get('controller_component'):
            # Declared free-scene path is a controller fixture; it bypasses the
            # global planner and is reported separately from NavigateToPose.
            goal.path=NavPath();goal.path.header.frame_id='map';goal.path.header.stamp=self.get_clock().now().to_msg()
            sx,sy,sa=self.case['start'];gx,gy,ga=self.case['goal'];sx+=.35
            points=self.case.get('controller_path')
            if points is not None:
                if len(points)<2 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in points):
                    raise ValueError('Invalid declared controller fixture path')
                goal.path.poses=[target(p) for p in points]
            else:
                goal.path.poses=[target([sx+(gx-sx)*i/100,sy+(gy-sy)*i/100,sa+(ga-sa)*i/100]) for i in range(101)]
            goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
        elif self.case.get('waypoints'):goal.poses=[target(values) for values in self.case['waypoints']]
        else:goal.pose=target(self.case['goal'])
        handle=None;result=None
        if not self.case.get('unsafe'):
            future=self.action.send_goal_async(goal)
            self.goal_submissions+=1
            if not self.wait(future.done,8):raise RuntimeError('Goal acceptance timeout')
            handle=future.result()
            if not handle.accepted:raise RuntimeError('Goal rejected')
            result=handle.get_result_async()
        self.collect=True;start=time.monotonic();sim_start=self.clock
        injected=False;restored=False
        while time.monotonic()-start<self.case['timeout']:
            rclpy.spin_once(self,timeout_sec=.01)
            self.sample_resources()
            elapsed=self.clock-sim_start
            if self.case.get('unsafe'):
                forced=Twist();forced.linear.x=self.case.get('unsafe_speed',.35);self.forced_command.publish(forced)
            if self.case.get('dynamic') and elapsed>3 and not injected:
                # Fixture moves in Gazebo; it is never injected into the navigation map.
                px=self.truth.pose.pose.position.x+.85
                acknowledgement=subprocess.run(['gz','service','-s','/world/duojin02_lab/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','1000','--req',f'name: "dynamic_obstacle" position {{x:{px} y:0 z:0.15}} orientation {{w:1}}'],capture_output=True,text=True,check=True)
                if 'true' not in acknowledgement.stdout:raise RuntimeError('Dynamic obstacle pose service rejected stimulus')
                self.stimuli.append(dict(t=self.clock,x=px,y=0,sx=.22,sy=.22,acknowledgement=acknowledgement.stdout.strip()))
                self.dynamic_reference=rectangle(px-.11,-.11,px+.11,.11);injected=True
            if self.case.get('dropout') and elapsed>3 and not injected:
                self.motor.publish(Int8(data=0));injected=True;self.stimulus_time=self.clock
            if self.case.get('dropout') and elapsed>5 and not restored:
                self.motor.publish(Int8(data=1));restored=True
            if (result is not None and result.done()) or self.collision_events:break
        status=result.result().status if result is not None and result.done() else 0
        if handle is not None and not result.done():
            handle.cancel_goal_async();self.wait(lambda:False,.25)
        if self.case.get('unsafe'):self.forced_command.publish(Twist())
        self.wait(lambda:False,.3);self.collect=False;self.resolve_estimates(flush=True)
        rows=self.samples
        errors=[math.hypot(r['estimated_x']-r['x'],r['estimated_y']-r['y']) for r in rows if math.isfinite(r['estimated_x'])]
        angular=[abs(math.atan2(math.sin(r['estimated_yaw']-r['yaw']),math.cos(r['estimated_yaw']-r['yaw']))) for r in rows if math.isfinite(r['estimated_yaw'])]
        lengths=[math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(rows,rows[1:])]
        gaps=np.diff(self.commands)
        final=rows[-1] if rows else dict(x=math.nan,y=math.nan,yaw=math.nan)
        resources={}
        for name in sorted({r[0] for r in self.resources}):
            values=[r for r in self.resources if r[0]==name]
            resources[name]=dict(cpu_percent_one_core=100*(values[-1][2]-values[0][2])/max(.001,values[-1][1]-values[0][1]),
                                 peak_rss_mib=max(r[3] for r in values))
        position_error=math.hypot(final['x']-x,final['y']-y)
        yaw_error=abs(math.remainder(final['yaw']-yaw,2*math.pi))
        dwell=0.0
        for row in reversed(rows):
            if row['speed']>.020 or math.hypot(row['x']-x,row['y']-y)>.020 or abs(math.remainder(row['yaw']-yaw,2*math.pi))>.035:break
            dwell=rows[-1]['t']-row['t']
        arrival_pass=status==4 and self.collision_events==0 and position_error<=.020 and yaw_error<=.035 and final.get('speed',math.inf)<=.020 and dwell>=.25
        speeds=[r['speed'] for r in rows]
        moving=[v for v in speeds if v>.02]
        return dict(status=status,success=status==4 and self.collision_events==0,collision_events=self.collision_events,
            navigation_goal_count=self.goal_submissions,
            arrival_acceptance_pass=arrival_pass,stopped_dwell_s=dwell,arrival_limits=dict(position_m=.020,yaw_rad=.035,speed_mps=.020,dwell_s=.25),
            final_speed_mps=final.get('speed'),speed_p95_mps=float(np.percentile(speeds,95)) if speeds else None,
            moving_speed_median_mps=float(np.median(moving)) if moving else None,
            average_progress_mps=sum(lengths)/max(.001,self.clock-sim_start),
            max_abs_roll_rad=max((abs(r['roll']) for r in rows),default=None),max_abs_pitch_rad=max((abs(r['pitch']) for r in rows),default=None),
            navigation_action='FollowPath' if self.case.get('controller_component') else ('NavigateThroughPoses' if self.case.get('waypoints') else 'NavigateToPose'),
            expected_outcome='safe_stop' if self.case.get('unsafe') else 'navigate',
            safety_stop_pass=self.collision_events==0 and rows[-1]['speed']<.005 if self.case.get('unsafe') and rows else None,
            min_clearance_m=min((r['clearance'] for r in rows),default=None),path_length_m=sum(lengths),
            goal_position_error_m=math.hypot(final['x']-x,final['y']-y),goal_yaw_error_rad=abs(math.atan2(math.sin(final['yaw']-yaw),math.cos(final['yaw']-yaw))),
            localization_rmse_m=math.sqrt(statistics.mean(e*e for e in errors)) if errors else None,
            localization_p95_m=float(np.percentile(errors,95)) if errors else None,
            localization_yaw_rmse_rad=math.sqrt(statistics.mean(e*e for e in angular)) if angular else None,
            localization_samples=len(errors),truth_samples=len(rows),sim_elapsed_s=self.clock-sim_start,wall_elapsed_s=time.monotonic()-start,
            localization_tf_failures=dict(self.tf_failures),
            node_resources=resources,scan_dropout_stop_latency_s=self.stop_after_stimulus,
            aperture_crossed=any(abs(r['x'])<.1 and abs(r['y'])<self.case.get('width',100)/2 for r in rows),
            command_period_p95_ms=float(np.percentile(gaps,95)*1000) if len(gaps) else None,
            guard_compute_p95_ms=float(np.percentile(self.guard,95)) if self.guard else None,
            guard_compute_max_ms=max(self.guard) if self.guard else None,
            guard_reasons=dict(self.guard_reasons)),rows


def trial(args):
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    case=json.loads(Path(args.scenario_file).read_text()) if args.scenario_file else next(c for c in scenarios() if c['name']==args.case)
    (output/'scenario.json').write_text(json.dumps(case))
    profile=yaml.safe_load(Path('/ws/simulation/src/duojin02_sim/config/profile.yaml').read_text())
    profile['base'].update(odom_velocity_scale=1.03,odom_velocity_noise_mps=.003,gyro_bias_radps=.001,random_seed=args.seed)
    (output/'profile.yaml').write_text(yaml.safe_dump(profile))
    config='/hardware/docs/evidence/navigation-upgrade/baseline-nav2.yaml' if args.variant=='baseline' else '/hardware/src/duojin01_bringup/config/nav2.yaml'
    config=args.nav_config or config
    (output/'nav2-source.yaml').write_bytes(Path(config).read_bytes())
    slam_config=Path('/hardware/src/duojin01_bringup/config/slam_toolbox.yaml')
    (output/'slam-source.yaml').write_bytes(slam_config.read_bytes())
    command=['ros2','launch','duojin02_sim','lab.launch.py','headless:=true','rviz:=false',f'mapping:={str(bool(case.get("online"))).lower()}','navigation:=true',
        f'guard:={str(args.variant=="upgraded").lower()}',f'nav_params:={config}',f'scenario:={output}/scenario.json',
        f'profile:={output}/profile.yaml','description_path:=/ws/src/duojin01_description']
    if args.slam_graph:
        if not case.get('online'):raise ValueError('A saved SLAM graph requires online mode')
        command.append(f'slam_graph:={args.slam_graph}')
    log=(output/'launch.log').open('w');process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report=dict(case=args.case,variant=args.variant,seed=args.seed,odom_scale=1.03,odom_noise_mps=.003,gyro_bias_radps=.001,range_noise_stddev_m=.005,evaluation_translation_step_m=.004)
    report['slam_configuration_sha256']=hashlib.sha256(slam_config.read_bytes()).hexdigest()
    report['configuration_sha256']=hashlib.sha256(Path(config).read_bytes()).hexdigest()
    report['evaluator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['slam_graph_sha256']={suffix:hashlib.sha256(Path(args.slam_graph+suffix).read_bytes()).hexdigest()
                               for suffix in ('.data','.posegraph')} if args.slam_graph else None
    report['algorithm_sha256']={str(p.relative_to('/hardware')):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in Path('/hardware/src/duojin01_navigation').rglob('*') if p.is_file()}
    node=None
    try:
        manifest=Path('/tmp/navbuild/navigation-build.json')
        if manifest.exists():
            built=json.loads(manifest.read_text())
            for group in ('sources','binaries'):
                for file,expected in built[group].items():
                    if hashlib.sha256(Path(file).read_bytes()).hexdigest()!=expected:
                        raise RuntimeError(f'Build / source mismatch: {file}; rebuild before running')
            report['verified_build']=True
            (output/'navigation-build.json').write_bytes(manifest.read_bytes())
        end=time.monotonic()+40;reference=None
        while time.monotonic()<end:
            text=(output/'launch.log').read_text()
            if 'Benchmark reference: ' in text:
                reference=Path(text.split('Benchmark reference: ',1)[1].splitlines()[0]);break
            if process.poll() is not None:raise RuntimeError('Launch exited')
            time.sleep(.2)
        if reference is None:raise RuntimeError('Scene generation timeout')
        case=json.loads(reference.read_text());(output/'case.json').write_text(json.dumps(case,indent=2))
        config_data=yaml.safe_load(Path(config).read_text())
        footprint=json.loads(config_data['local_costmap']['local_costmap']['ros__parameters']['footprint'])
        rclpy.init();node=Probe(case,footprint,args.seed);node.launch_process=process
        report['rmw_implementation']=get_rmw_implementation_identifier()
        metrics,rows=node.run();report.update(metrics)
        (output/'plans.json').write_text(json.dumps(node.plans))
        (output/'stimuli.json').write_text(json.dumps(node.stimuli))
        with (output/'command-chain.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=['t','wall_t','topic','x','y','yaw']);writer.writeheader();writer.writerows(node.command_chain)
        if rows:
            with (output/'trajectory.csv').open('w') as f:
                writer=csv.DictWriter(f,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    except Exception as error:
        report.update(success=False,error=str(error))
    finally:
        try:
            if node is not None:node.destroy_node()
            rclpy.try_shutdown()
        except (Exception,KeyboardInterrupt) as cleanup_error:
            report['cleanup_note']=str(cleanup_error)
        finally:
            forced_cleanup=False
            # Launch owns its child shutdown. Signalling the whole group also
            # delivers SIGINT to each child before Launch sends it again.
            try:process.send_signal(signal.SIGINT);process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                forced_cleanup=True;os.killpg(process.pid,signal.SIGKILL);process.wait()
            except ProcessLookupError:pass
        log.close()
        logs=(output/'launch.log').read_text()
        forced_cleanup = forced_cleanup or any(marker in logs for marker in
            ('escalating to', "sending signal 'SIGTERM'", "sending signal 'SIGKILL'"))
        report['launch_returncode']=process.returncode
        report['forced_cleanup']=forced_cleanup
        report['complete_flow']=process.returncode==0 and not forced_cleanup and 'cleanup_note' not in report
        report['controller_deadline_warnings']=logs.count('Control loop missed its desired rate')
        import re
        report['gpu_control_summaries']=[dict(zip(('native_samples','cuda_samples','best_sequence','gpu_failures','deadline_misses'),map(int,m))) for m in re.findall(r'GPU_CONTROL_SUMMARY native_samples=(\d+) cuda_samples=(\d+) best_sequence=(\d+) gpu_failures=(\d+) deadline_misses=(\d+)',logs)]
        report['gpu_geometry_summaries']=re.findall(r'GPU_GEOMETRY_SUMMARY[^\n]+',logs)
        report={k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in report.items()}
        (output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        print(json.dumps(report),flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',action='store_true');p.add_argument('--case',default='open')
    p.add_argument('--variant',choices=['baseline','upgraded'],default='upgraded');p.add_argument('--seed',type=int,default=1)
    p.add_argument('--nav-config',default='')
    p.add_argument('--scenario-file',default='')
    p.add_argument('--slam-graph',default='')
    p.add_argument('--output',default='/evidence/open-upgraded-1');args=p.parse_args()
    trial(args)


if __name__=='__main__':main()
