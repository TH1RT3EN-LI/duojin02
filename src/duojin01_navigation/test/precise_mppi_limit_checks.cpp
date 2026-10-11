#include <cassert>
#include <iostream>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "../src/precise_mppi.cpp"

using namespace duojin01_navigation;
class NativeOracle : public mppi::Optimizer {
 public:
  NativeOracle() {settings_.base_constraints={.5f,-.2f,.3f,.65f};}
  OmniVelocityBounds bounds() const {const auto &b=settings_.constraints;return {b.vx_max,b.vx_min,b.vy,b.wz};}
};
class TestController : public PreciseMPPI {
 public:
  void forcePeriodicReset() {last_time_called_=clock_->now()-rclcpp::Duration::from_seconds(2.0);}
};
geometry_msgs::msg::PoseStamped pose(double x=0,double y=0,double a=0) {
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="odom";p.pose.position.x=x;p.pose.position.y=y;
  p.pose.orientation.z=std::sin(a/2);p.pose.orientation.w=std::cos(a/2);return p;
}
void bounded(const geometry_msgs::msg::Twist &v,const OmniVelocityBounds &b,bool stop=false) {
  assert(std::isfinite(v.linear.x+v.linear.y+v.angular.z));
  assert(v.linear.x>=b.vx_min-1e-9 && v.linear.x<=b.vx_max+1e-9);
  assert(std::abs(v.linear.y)<=b.vy_max+1e-9 && std::abs(v.angular.z)<=b.wz_max+1e-9);
  if(stop)assert(v.linear.x==0 && v.linear.y==0 && v.angular.z==0);
}
int main(int argc,char **argv) {
  bool gpu=false,structured=false;for(int i=1;i<argc;++i){if(std::string(argv[i])=="--gpu")gpu=true;if(std::string(argv[i])=="--structured")structured=true;}
  if(gpu){GeometryCompute probe({ComputeBackend::Auto,0,2000});if(!probe.available())return 77;}
  NativeOracle oracle;OmniVelocityLimits reference;reference.configure({.5,-.2,.3,.65});
  for(const auto request:std::vector<std::pair<double,bool>>{{20,true},{60,true},{.1,false},{0,false},{0,true}}) {
    oracle.setSpeedLimit(request.first,request.second);reference.update(request.first,request.second);
    const auto a=oracle.bounds(),b=reference.effective();
    assert(std::abs(a.vx_max-b.vx_max)<1e-6 && std::abs(a.vx_min-b.vx_min)<1e-6 &&
      std::abs(a.vy_max-b.vy_max)<1e-6 && std::abs(a.wz_max-b.wz_max)<1e-6);
  }
  reference.update(-1,true);oracle.setSpeedLimit(reference.nativePercentage(),true);
  const auto stopped_native=oracle.bounds();
  assert(stopped_native.vx_max==0 && stopped_native.vx_min==0 && stopped_native.vy_max==0 && stopped_native.wz_max==0);
  std::cout<<"INSTALLED_HUMBLE_ORACLE percentage/absolute/zero-reset and fail-closed encoding passed\n";
  rclcpp::init(argc,argv);
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("controller_repair_costmap");
  map->set_parameter(rclcpp::Parameter("global_frame",std::string("odom")));
  map->set_parameter(rclcpp::Parameter("robot_base_frame",std::string("base_footprint")));
  map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{"inflation_layer"}));
  map->declare_parameter("inflation_layer.plugin",std::string("nav2_costmap_2d::InflationLayer"));
  map->declare_parameter("inflation_layer.inflation_radius",.4);map->declare_parameter("inflation_layer.cost_scaling_factor",6.);
  map->set_parameter(rclcpp::Parameter("footprint",std::string("[[0.213,0.191465],[0.201465,0.203],[-0.202879,0.203],[-0.214,0.191879],[-0.214,-0.190465],[-0.201465,-0.203],[0.202879,-0.203],[0.213,-0.192879]]")));
  map->set_parameter(rclcpp::Parameter("footprint_padding",.008));map->on_configure(rclcpp_lifecycle::State());map->getCostmap()->resizeMap(600,600,.01,-3.,-3.);
  // Explicit main body-axis/model settings keep this API regression independent
  // of map files, launch lifecycle, and future production YAML additions.
  // Critic selection here only makes the free-map native branch executable;
  // this is a bounds test, not a trajectory quality or collision benchmark.
  auto make_options=[&]() {
    rclcpp::NodeOptions o;
    o.parameter_overrides({
      rclcpp::Parameter("controller_frequency",20.),
      rclcpp::Parameter("FollowPath.vx_max",.5),
      rclcpp::Parameter("FollowPath.vx_min",-.2),
      rclcpp::Parameter("FollowPath.vy_max",.3),
      rclcpp::Parameter("FollowPath.wz_max",.65),
      rclcpp::Parameter("FollowPath.narrow_speed_max",.5),
      rclcpp::Parameter("FollowPath.motion_model",std::string("Omni")),
      rclcpp::Parameter("FollowPath.time_steps",56),
      rclcpp::Parameter("FollowPath.model_dt",.05),
      rclcpp::Parameter("FollowPath.batch_size",1600),
      rclcpp::Parameter("FollowPath.vx_std",.15),
      rclcpp::Parameter("FollowPath.vy_std",.12),
      rclcpp::Parameter("FollowPath.wz_std",.18),
      rclcpp::Parameter("FollowPath.reset_period",1.),
      rclcpp::Parameter("FollowPath.critics",std::vector<std::string>{
        "ConstraintCritic","GoalCritic","GoalAngleCritic","PathFollowCritic"})});
    if(gpu) {
      o.append_parameter_override("FollowPath.compute_backend",std::string("auto"));
      o.append_parameter_override("FollowPath.sampling_backend",std::string("cuda"));
      o.append_parameter_override("FollowPath.gpu_batch_size",3200);
      o.append_parameter_override("FollowPath.gpu_support_points",4);
      o.append_parameter_override("FollowPath.gpu_vx_std",.30);
      o.append_parameter_override("FollowPath.gpu_vy_std",.30);
      o.append_parameter_override("FollowPath.gpu_wz_std",.12);
      o.append_parameter_override("FollowPath.gpu_path_weight",3.);
      o.append_parameter_override("FollowPath.gpu_goal_weight",16.);
      if(structured)o.append_parameter_override("FollowPath.gpu_candidate_mode",std::string("path_primitives"));
    }
    return o;
  };
  auto options=make_options();options.append_parameter_override("FollowPath.wz_max",.1);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",options);
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());tf2_ros::TransformListener listener(*tf);
  TestController controller;controller.configure(node,"FollowPath",tf,map);controller.activate();
  nav2_controller::SimpleGoalChecker checker;checker.initialize(node,"probe_goal",map);
  OmniVelocityLimits expected;expected.configure({.5,-.2,.3,.1});
  if(structured) {
    // A clear incoming chord must still activate the sampler before a bend.
    // With zero measured speed its first command obeys the 0.4 m/s^2 rollout
    // limit, whereas the ordinary deterministic tracker requests 0.36 m/s.
    nav_msgs::msg::Path bend;bend.header.frame_id="odom";bend.poses={pose(-.8,0),pose(0,0),pose(0,.8)};
    controller.setPlan(bend);auto incoming=pose(-.8,0);incoming.header.stamp=node->now();geometry_msgs::msg::Twist resting;
    const auto guided=controller.computeVelocityCommands(incoming,resting,&checker).twist;
    assert(std::hypot(guided.linear.x,guided.linear.y)>.005);
    assert(std::hypot(guided.linear.x,guided.linear.y)<=.4*.05+1e-8);
    std::cout<<"STRUCTURED_EARLY_BEND acceleration-bounded takeover passed\n";
  }
  const std::vector<geometry_msgs::msg::PoseStamped> goals={pose(.4,0),pose(-.4,0),pose(0,.4),pose(.3,.3,1),pose(0,0,1),pose(.8,0),pose(-.8,0),pose(0,.8),pose(.8,.2,1)};
  int count=0;
  for(const auto request:std::vector<std::pair<double,bool>>{{20,true},{60,true},{20,true},{.1,false},{0,false},{200,true},{2,false},{-1,true},{std::numeric_limits<double>::quiet_NaN(),false},{std::numeric_limits<double>::infinity(),true},{0,true}}) {
    controller.setSpeedLimit(request.first,request.second);expected.update(request.first,request.second);
    for(int narrow:{0,1}) {
      map->getCostmap()->setCost(300,300,narrow ? 120 : 0);
      for(auto goal:goals) {
        auto start=pose();start.header.stamp=node->now();goal.header.stamp=start.header.stamp;
        nav_msgs::msg::Path path;path.header=start.header;const double a=tf2::getYaw(goal.pose.orientation);
        for(int i=0;i<=100;++i)path.poses.push_back(pose(goal.pose.position.x*i/100,goal.pose.position.y*i/100,a*i/100));
        controller.setPlan(path);geometry_msgs::msg::Twist speed;
        const auto out=controller.computeVelocityCommands(start,speed,&checker).twist;bounded(out,expected.effective(),expected.stopped());++count;
      }
    }
  }
  map->getCostmap()->setCost(300,300,0);controller.setSpeedLimit(20,true);
  const auto changed=node->set_parameters_atomically({rclcpp::Parameter("FollowPath.vx_max",.4),rclcpp::Parameter("FollowPath.vx_min",-.1),rclcpp::Parameter("FollowPath.vy_max",.2),rclcpp::Parameter("FollowPath.wz_max",.04)});assert(changed.successful);
  expected.configure({.4,-.1,.2,.04});expected.update(20,true);
  auto start=pose();start.header.stamp=node->now();auto goal=pose(.3,.2,1);goal.header=start.header;
  nav_msgs::msg::Path path;path.header=start.header;path.poses={start,goal};controller.setPlan(path);geometry_msgs::msg::Twist speed;
  auto out=controller.computeVelocityCommands(start,speed,&checker).twist;bounded(out,expected.effective());assert(out.angular.z>0);
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.wz_max",std::numeric_limits<double>::quiet_NaN())).successful);
  out=controller.computeVelocityCommands(start,speed,&checker).twist;bounded(out,{},true);
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.wz_max",.04)).successful);
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.vy_max",0.)).successful);controller.setSpeedLimit(0,false);
  goal=pose(.3,0);goal.header=start.header;path.poses={start,goal};controller.setPlan(path);
  out=controller.computeVelocityCommands(start,speed,&checker).twist;assert(out.linear.x>0 && out.linear.y==0);bounded(out,{.4,-.1,0,.04});
  assert(node->set_parameter(rclcpp::Parameter("FollowPath.vy_max",.2)).successful);controller.setSpeedLimit(20,true);
  // More than .2 m off the plan disables deterministic chord tracking; this
  // invokes the real native MPPI with a valid goal checker and same free map.
  path.poses={pose(.35,0),pose(1,0)};controller.setPlan(path);
  for(int i=0;i<3;++i) {
    controller.forcePeriodicReset();start.header.stamp=node->now();out=controller.computeVelocityCommands(start,speed,&checker).twist;bounded(out,expected.effective());
    std::cout<<"NATIVE_PUBLIC_LIMIT vx="<<out.linear.x<<" vy="<<out.linear.y<<" wz="<<out.angular.z<<"\n";
  }
  controller.deactivate();controller.cleanup();
  if(gpu) {
    auto fault_options=make_options();
    fault_options.append_parameter_override("FollowPath.gpu_deadline_ms",.000001);
    auto fault_node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",fault_options);
    TestController fault;fault.configure(fault_node,"FollowPath",tf,map);fault.activate();
    fault.setPlan(path);start.header.stamp=fault_node->now();
    const auto stopped_command=fault.computeVelocityCommands(start,speed,&checker).twist;
    assert(stopped_command.linear.x==0 && stopped_command.linear.y==0 && stopped_command.angular.z==0);
    fault.deactivate();fault.cleanup();
    std::cout<<"GPU_DEADLINE_FAULT zero command passed\n";
  }
  for(const auto &bad:std::vector<std::pair<std::string,double>>{{"vx_max",0},{"vx_min",.2},{"vy_max",-.1},{"wz_max",-.1},{"wz_max",std::numeric_limits<double>::infinity()}}) {
    auto o=make_options();o.append_parameter_override("FollowPath."+bad.first,bad.second);auto invalid_node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",o);
    TestController invalid;bool rejected=false;
    try {invalid.configure(invalid_node,"FollowPath",tf,map);}catch(const std::invalid_argument &e) {rejected=std::string(e.what()).find(bad.first)!=std::string::npos;std::cout<<"INVALID_CONFIG_REJECTED "<<e.what()<<"\n";}
    assert(rejected);invalid.cleanup();
  }
  map->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();
  std::cout<<"PUBLIC_API_CHECKS "<<count<<" deterministic/docking/narrow percentage/absolute/reset/invalid commands; dynamic config; native periodic reset; invalid setup passed\n";
}
