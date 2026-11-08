// Actual current controller + actual StableStoppedGoalChecker; declared synthetic
// plant/truth only. No ROS Action, hardware, external smoother/mux/guard or Gazebo.
#include "duojin02_navigation/stable_goal_checker.hpp"
#include "../../../src/duojin02_navigation/src/precise_mppi.cpp"
#include <rcl/time.h>
#include <cassert>
#include <deque>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using nlohmann::json;
using namespace duojin02_navigation;
static geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "odom";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw/2); p.pose.orientation.w = std::cos(yaw/2);
  return p;
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string output = argv[1];
  rclcpp::init(argc, argv);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("synthetic_task_costmap");
  map->set_parameter(rclcpp::Parameter("global_frame", std::string("odom")));
  map->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_footprint")));
  map->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}));
  map->declare_parameter("inflation_layer.plugin", std::string("nav2_costmap_2d::InflationLayer"));
  map->declare_parameter("inflation_layer.inflation_radius", .4);
  map->declare_parameter("inflation_layer.cost_scaling_factor", 6.);
  map->set_parameter(rclcpp::Parameter("footprint", std::string(
    "[[0.213,0.191465],[0.201465,0.203],[-0.202879,0.203],[-0.214,0.191879],[-0.214,-0.190465],[-0.201465,-0.203],[0.202879,-0.203],[0.213,-0.192879]]")));
  map->set_parameter(rclcpp::Parameter("footprint_padding", .008));
  map->on_configure(rclcpp_lifecycle::State());
  map->getCostmap()->resizeMap(600,600,.01,-3.,-3.);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("controller_frequency",20.),
    rclcpp::Parameter("FollowPath.use_gpu",false),
    rclcpp::Parameter("FollowPath.optimizer_backend",std::string("cpu")),
    rclcpp::Parameter("FollowPath.optimizer_seed",42),
    rclcpp::Parameter("FollowPath.motion_model",std::string("Omni")),
    rclcpp::Parameter("FollowPath.vx_max",.5), rclcpp::Parameter("FollowPath.vx_min",-.2),
    rclcpp::Parameter("FollowPath.vy_max",.3), rclcpp::Parameter("FollowPath.wz_max",.65),
    rclcpp::Parameter("FollowPath.critics",std::vector<std::string>{"ConstraintCritic","GoalCritic","GoalAngleCritic","PathFollowCritic"})});
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("synthetic_task_controller",options);
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener listener(*tf);
  PreciseMPPI controller;
  controller.configure(node,"FollowPath",tf,map);
  controller.activate();
  node->declare_parameter("goal.xy_goal_tolerance",.004);
  node->declare_parameter("goal.yaw_goal_tolerance",.02);
  node->declare_parameter("goal.trans_stopped_velocity",.01);
  node->declare_parameter("goal.rot_stopped_velocity",.02);
  StableStoppedGoalChecker checker;
  checker.initialize(node,"goal",map);
  auto clock=node->get_clock()->get_clock_handle();
  assert(rcl_enable_ros_time_override(clock)==RCL_RET_OK);
  json protocol={{"frozen_at_ns",0},{"registration",{{"id","synthetic_identity"},
    {"map_to_truth",{0.,0.,0.}},{"frozen_at_ns",0},{"evidence","synthetic_native_components_only"},
    {"physical_calibration","unknown"}}},{"limits",{{"xy_m",.02},{"yaw_rad",M_PI/180},
    {"linear_mps",.01},{"angular_rps",.02},{"dwell_s",.25},{"max_sample_gap_s",.15}}}};
  std::ofstream(output+"/protocol.json") << protocol.dump(2)<<'\n';
  std::ofstream contract_log(output+"/control-contracts.jsonl");
  auto observer=std::make_shared<rclcpp::Node>("task_contract_observer");
  auto goal_contract_pub=observer->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "task_goal_contract",rclcpp::QoS(1).transient_local());
  auto estimate_contract_pub=observer->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "amcl_estimate_contract",10);
  int contract_count=0;
  auto contract_subscription=observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "task_state_at_control",100,[&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        for (const auto &status:message->status) {
          ContractFields fields;
          for (const auto &kv:status.values) fields[kv.key]=kv.value;
          assert(fields.at("physical_arrival_guarantee")=="unknown");
          assert(fields.at("goal_identity_association")=="unverified_no_action_goal_id");
          assert(fields.at("original_request_association")=="exact_endpoint_pose_match_action_identity_unknown");
          assert(fields.at("map_identity_consistency")=="equal");
          assert(std::stoll(fields.at("control_boundary_steady_ns"))<=std::stoll(fields.at("control_consumed_steady_ns")));
          assert(std::stoll(fields.at("control_consumed_steady_ns"))<=std::stoll(fields.at("command_available_steady_ns")));
          contract_log<<json(fields).dump()<<'\n';
          ++contract_count;
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(observer);
  auto publish_contract=[&](const std::string &name,const ContractFields &fields,auto publisher) {
    diagnostic_msgs::msg::DiagnosticArray message;
    diagnostic_msgs::msg::DiagnosticStatus status;status.name=name;
    for (const auto &field:fields) {
      diagnostic_msgs::msg::KeyValue kv;kv.key=field.first;kv.value=field.second;status.values.push_back(kv);
    }
    message.status.push_back(status);publisher->publish(message);
  };
  std::ofstream samples(output+"/samples.jsonl");
  json tasks=json::array();
  const double dt=.05;
  int trial=0;
  for (const auto bias : {0.,.04}) for (int delay : {0,4}) {
    const std::string id="bias_"+std::to_string(bias)+"_delay_"+std::to_string(delay);
    const int64_t start=1000000000LL + trial++*20000000000LL;
    const auto goal=pose(.3,.1,.2);
    nav_msgs::msg::Path path;
    path.header=goal.header;
    path.poses={pose(bias,0,0),goal};
    controller.setPlan(path);
    const auto &q=goal.pose.orientation;
    publish_contract("navigation/task_goal",{{"goal_id",id},{"original_frame","odom"},
      {"original_x",contractNumber(.3)},{"original_y",contractNumber(.1)},
      {"original_qx",contractNumber(q.x)},{"original_qy",contractNumber(q.y)},
      {"original_qz",contractNumber(q.z)},{"original_qw",contractNumber(q.w)},
      {"map_identity","synthetic-fixture-version"}},goal_contract_pub);
    publish_contract("amcl/estimate_contract",{{"map_identity","synthetic-fixture-version"},
      {"source_id","synthetic-diag-only"},{"physical_calibration","unknown"}},estimate_contract_pub);
    for (int wait=0;wait<20;++wait) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    checker.reset();
    std::deque<geometry_msgs::msg::Twist> queue(delay);
    geometry_msgs::msg::Twist measured;
    double x=0,y=0,yaw=0;
    bool succeeded=false;
    int64_t completion=0;
    for (int step=0;step<=240;++step) {
      const int64_t stamp=start+step*50000000LL;
      assert(rcl_set_ros_time_override(clock,stamp)==RCL_RET_OK);
      auto estimated=pose(x+bias,y,yaw);
      estimated.header.stamp=rclcpp::Time(stamp);
      const auto command=controller.computeVelocityCommands(estimated,measured,&checker);
      executor.spin_some();
      samples << json{{"task_id",id},{"stamp_ns",stamp},{"truth_base",{x,y,yaw}},
        {"truth_twist",{measured.linear.x,measured.linear.y,measured.angular.z}},
        {"estimate_base",{x+bias,y,yaw}},
        {"requested_twist",{command.twist.linear.x,command.twist.linear.y,command.twist.angular.z}},
        {"truth_evidence","declared_synthetic_plant_not_external_measurement"}}.dump()<<'\n';
      if (checker.isGoalReached(estimated.pose,goal.pose,measured)) {
        succeeded=true;completion=stamp;break;
      }
      queue.push_back(command.twist);
      const auto applied=queue.front();queue.pop_front();
      const double alpha=1-std::exp(-dt/.15); // declared synthetic lag, not calibrated hardware
      measured.linear.x+=alpha*(applied.linear.x-measured.linear.x);
      measured.linear.y+=alpha*(applied.linear.y-measured.linear.y);
      measured.angular.z+=alpha*(applied.angular.z-measured.angular.z);
      x+=dt*(std::cos(yaw)*measured.linear.x-std::sin(yaw)*measured.linear.y);
      y+=dt*(std::sin(yaw)*measured.linear.x+std::cos(yaw)*measured.linear.y);
      yaw=std::remainder(yaw+dt*measured.angular.z,2*M_PI);
    }
    tasks.push_back({{"id",id},{"goal_map",{.3,.1,.2}},{"start_ns",start},
      {"deadline_ns",start+12000000000LL},{"status",succeeded?"succeeded":"timed_out"},
      {"completion_ns",succeeded?json(completion):json(nullptr)},
      {"synthetic_bias_m",bias},{"synthetic_delay_steps",delay},{"synthetic_lag_s",.15}});
  }
  std::ofstream(output+"/tasks.json") << tasks.dump(2)<<'\n';
  for (int wait=0;wait<20;++wait) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  assert(contract_count>0);
  std::ofstream(output+"/contract-validation.json") << json{{"passed",true},
    {"received_control_contracts",contract_count},{"request_identity","unverified_no_action_goal_id"},
    {"tf_association","latest_received_diagnostic_not_atomic_with_tf"}}.dump(2)<<'\n';
  executor.remove_node(node->get_node_base_interface());executor.remove_node(observer);
  controller.deactivate();controller.cleanup();
  rclcpp::shutdown();
  std::cout<<"Native controller + goal checker synthetic task loops completed; no physical claim\n";
}
