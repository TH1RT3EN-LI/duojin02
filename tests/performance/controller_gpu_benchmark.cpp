// Identical observer compiled against preserved current CPU source or candidate.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <time.h>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include DUOJIN_CONTROLLER_SOURCE
using namespace duojin01_navigation;
using Msg=geometry_msgs::msg::TwistStamped;
static size_t native_calls=0;
extern "C" Msg realNative(nav2_mppi_controller::MPPIController*,const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*) asm("__real__ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" Msg wrapNative(nav2_mppi_controller::MPPIController*,const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::Twist&,nav2_core::GoalChecker*) asm("__wrap__ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
Msg wrapNative(nav2_mppi_controller::MPPIController *self,const geometry_msgs::msg::PoseStamped &p,const geometry_msgs::msg::Twist &s,nav2_core::GoalChecker *c){++native_calls;return realNative(self,p,s,c);}
static double cpuMs(){timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec*1000.+t.tv_nsec/1e6;}
static auto pose(double x=0,double y=0,double a=0){geometry_msgs::msg::PoseStamped p;p.header.frame_id="odom";p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=std::sin(a/2);p.pose.orientation.w=std::cos(a/2);return p;}
int main(int argc,char **argv){
 if(argc!=5){std::cerr<<"controller_gpu_benchmark original|cuda iterations nav2.yaml result.json\n";return 2;}
 const std::string backend=argv[1];if(backend!="original" && backend!="cuda" && backend!="auto")return 2;const int iterations=std::stoi(argv[2]);if(iterations<1)return 2;
 std::vector<std::string> args{argv[0],"--ros-args","--params-file",argv[3]};std::vector<const char*> pointers;for(auto &s:args)pointers.push_back(s.c_str());rclcpp::init(pointers.size(),pointers.data());
 auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("benchmark_costmap");
 map->set_parameter(rclcpp::Parameter("global_frame",std::string("odom")));map->set_parameter(rclcpp::Parameter("robot_base_frame",std::string("base_footprint")));
 map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{"inflation_layer"}));
 map->declare_parameter("inflation_layer.plugin",std::string("nav2_costmap_2d::InflationLayer"));map->declare_parameter("inflation_layer.inflation_radius",.4);map->declare_parameter("inflation_layer.cost_scaling_factor",6.);
 map->set_parameter(rclcpp::Parameter("footprint",std::string("[[0.213,0.191465],[0.201465,0.203],[-0.202879,0.203],[-0.214,0.191879],[-0.214,-0.190465],[-0.201465,-0.203],[0.202879,-0.203],[0.213,-0.192879]]")));
 map->set_parameter(rclcpp::Parameter("footprint_padding",.008));map->on_configure(rclcpp_lifecycle::State());map->getCostmap()->resizeMap(400,400,.01,-2.,-2.);
 rclcpp::NodeOptions options;
 options.append_parameter_override("FollowPath.sampling_backend",backend=="original" ? std::string("original"):backend);
 options.append_parameter_override("FollowPath.compute_backend",backend=="original" ? std::string("cpu"):std::string("auto"));
 auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server",options);
 auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());tf2_ros::TransformListener listener(*tf);
 PreciseMPPI controller;controller.configure(node,"FollowPath",tf,map);controller.activate();
 nav2_controller::SimpleGoalChecker checker;checker.initialize(node,"benchmark_goal",map);
 nav_msgs::msg::Path path;path.header.frame_id="odom";for(int i=0;i<=85;++i)path.poses.push_back(pose(.35+.01*i,.017));
 controller.setPlan(path);geometry_msgs::msg::Twist measured;std::vector<double> wall,cpu;std::vector<Velocity> commands;double cold=0;size_t warm_native=0,zero=0;
 for(int i=-10;i<iterations;++i){auto p=pose(0,.017);p.header.stamp=node->now();const auto begin=std::chrono::steady_clock::now();const double c=cpuMs();
  auto result=controller.computeVelocityCommands(p,measured,&checker).twist;const double cm=cpuMs()-c,w=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  if(i==-10)cold=w;if(i==-1)warm_native=native_calls;
  if(i>=0){wall.push_back(w);cpu.push_back(cm);commands.push_back({result.linear.x,result.linear.y,result.angular.z});zero+=result.linear.x==0 && result.linear.y==0 && result.angular.z==0;
   if(!std::isfinite(result.linear.x+result.linear.y+result.angular.z) || result.linear.x<-.2-1e-6 || result.linear.x>.5+1e-6 || std::abs(result.linear.y)>.3+1e-6 || std::abs(result.angular.z)>.65+1e-6)throw std::runtime_error("public command bounds failed");}
 }
 controller.deactivate();controller.cleanup();map->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();
 auto quantile=[](std::vector<double> v,double q){std::sort(v.begin(),v.end());return v[static_cast<size_t>(q*(v.size()-1))];};auto mean=[](auto &v){return std::accumulate(v.begin(),v.end(),0.)/v.size();};
 std::ofstream out(argv[4]);out<<std::setprecision(12)<<"{\"backend\":\""<<backend<<"\",\"iterations\":"<<iterations<<",\"native_calls\":"<<native_calls-warm_native<<",\"cold_ms\":"<<cold<<",\"zero_calls\":"<<zero<<",\"wall_mean_ms\":"<<mean(wall)<<",\"wall_p95_ms\":"<<quantile(wall,.95)<<",\"wall_max_ms\":"<<*std::max_element(wall.begin(),wall.end())<<",\"cpu_mean_ms\":"<<mean(cpu)<<",\"wall_samples_ms\":[";
 for(size_t i=0;i<wall.size();++i)out<<(i ? ",":"")<<wall[i];out<<"],\"cpu_samples_ms\":[";for(size_t i=0;i<cpu.size();++i)out<<(i ? ",":"")<<cpu[i];out<<"],\"commands\":[";
 for(size_t i=0;i<commands.size();++i){auto c=commands[i];out<<(i ? ",":"")<<"["<<c.x<<","<<c.y<<","<<c.yaw<<"]";}out<<"]}\n";
}
