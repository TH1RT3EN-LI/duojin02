#include "duojin01_navigation/sweep.hpp"
#include <nav2_mppi_controller/controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace duojin01_navigation {
class PreciseMPPI : public nav2_mppi_controller::MPPIController {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    nav2_mppi_controller::MPPIController::configure(parent,name,tf,costmap);
    auto node=parent.lock();
    max_speed_=node->get_parameter(name+".vx_max").as_double();
    reverse_speed_=-node->get_parameter(name+".vx_min").as_double();
    lateral_speed_=node->get_parameter(name+".vy_max").as_double();
    speed_limit_=max_speed_;
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    nav2_mppi_controller::MPPIController::setPlan(path);
    path_=path;
    have_goal_=!path.poses.empty();if(have_goal_) goal_=path.poses.back();
  }
  void setSpeedLimit(const double &limit,const bool &percentage) override {
    nav2_mppi_controller::MPPIController::setSpeedLimit(limit,percentage);
    speed_limit_=limit==0 ? max_speed_ : percentage ? max_speed_*std::clamp(limit/100.0,0.0,1.0) : std::clamp(limit,0.0,max_speed_);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    geometry_msgs::msg::PoseStamped target;
    try {
      if(!have_goal_) return nav2_mppi_controller::MPPIController::computeVelocityCommands(pose,speed,checker);
      auto goal=goal_;goal.header.stamp=pose.header.stamp;
      tf_buffer_->transform(goal,target,pose.header.frame_id,tf2::durationFromSec(.1));
    } catch(const tf2::TransformException &) {
      return nav2_mppi_controller::MPPIController::computeVelocityCommands(pose,speed,checker);
    }
    const double distance=std::hypot(target.pose.position.x-pose.pose.position.x,target.pose.position.y-pose.pose.position.y);
    if(distance<=.45 && directApproachClear(pose,target)) return converge(pose,target,true);
    // Follow a collision-checked short SE2 lattice chord directly. Sampling is
    // reserved for blocked chords / dynamic detours, reducing CPU and jitter.
    if(trackingTarget(pose,target,speed)) return converge(pose,target,false);
    try {return nav2_mppi_controller::MPPIController::computeVelocityCommands(pose,speed,checker);}
    catch(const std::runtime_error &error) {
      if(std::string(error.what())!="Optimizer fail to compute path") throw;
      if(trackingTarget(pose,target,speed)) return converge(pose,target,false);
      throw;
    }
  }

 private:
  geometry_msgs::msg::TwistStamped converge(const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::PoseStamped &target,bool docking) {
    const double dx=target.pose.position.x-pose.pose.position.x,dy=target.pose.position.y-pose.pose.position.y;
    const double yaw=tf2::getYaw(pose.pose.orientation),dyaw=std::remainder(tf2::getYaw(target.pose.orientation)-yaw,2*M_PI);
    // Speed depends on remaining distance, yaw tracking and obstacle proximity.
    // The independent measured-velocity braking guard remains authoritative.
    const double vx=2.0*(std::cos(yaw)*dx+std::sin(yaw)*dy);
    const double vy=2.0*(-std::sin(yaw)*dx+std::cos(yaw)*dy);
    double cap=std::min(speed_limit_,(nearObstacle(pose) || nearObstacle(target)) ? .10 : max_speed_);
    if(docking) cap=std::min(cap,std::sqrt(.6*std::max(0.0,std::hypot(dx,dy)-.008)));
    cap*=std::clamp(1.0-std::abs(dyaw),.2,1.0);
    const double scale=std::min({1.0,cap/std::max(.001,std::hypot(vx,vy)),std::min(lateral_speed_,speed_limit_)/std::max(.001,std::abs(vy)),
      (vx<0 ? reverse_speed_ : max_speed_)/std::max(.001,std::abs(vx))});
    geometry_msgs::msg::TwistStamped result;result.header=pose.header;
    result.twist.linear.x=vx*scale;result.twist.linear.y=vy*scale;
    result.twist.angular.z=std::clamp(1.8*dyaw,-.45,.45);
    return result;
  }
  bool nearObstacle(const geometry_msgs::msg::PoseStamped &pose) {
    auto map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    unsigned int x=0,y=0;
    return map->worldToMap(pose.pose.position.x,pose.pose.position.y,x,y) && map->getCost(x,y)>=120;
  }
  bool trackingTarget(const geometry_msgs::msg::PoseStamped &pose,geometry_msgs::msg::PoseStamped &target,const geometry_msgs::msg::Twist &speed) {
    if(!pathTarget(pose,target,speed)) return false;
    // Align with the lattice before entering an inflated narrow region. Holding
    // a small initial yaw error there can consume the braking clearance.
    if(nearObstacle(target)) return directApproachClear(pose,target);
    // Omni translation does not require turning toward the travel direction.
    // Keep heading stable when that filled-footprint chord is feasible; use
    // the lattice heading when rotating is necessary to fit the passage.
    auto fixed_heading=target;fixed_heading.pose.orientation=pose.pose.orientation;
    if(directApproachClear(pose,fixed_heading)) {target=fixed_heading;return true;}
    return directApproachClear(pose,target);
  }
  bool pathTarget(const geometry_msgs::msg::PoseStamped &pose,geometry_msgs::msg::PoseStamped &target,const geometry_msgs::msg::Twist &speed) {
    if(path_.poses.empty()) return false;
    try {
      nav_msgs::msg::Path local;
      const auto tf=tf_buffer_->lookupTransform(pose.header.frame_id,path_.header.frame_id,pose.header.stamp,tf2::durationFromSec(.02));
      for(auto p:path_.poses) {p.header.stamp=pose.header.stamp;geometry_msgs::msg::PoseStamped q;
        tf2::doTransform(p,q,tf);local.poses.push_back(q);}
      size_t nearest=0;double minimum=1e100;
      for(size_t i=0;i<local.poses.size();++i) {
        const auto &p=local.poses[i].pose;
        const double d=std::hypot(p.position.x-pose.pose.position.x,p.position.y-pose.pose.position.y);
        const double dyaw=std::remainder(tf2::getYaw(p.orientation)-tf2::getYaw(pose.pose.orientation),2*M_PI);
        const double score=d*d+.02*dyaw*dyaw;
        if(score<minimum) {minimum=score;nearest=i;}
      }
      if(std::hypot(local.poses[nearest].pose.position.x-pose.pose.position.x,local.poses[nearest].pose.position.y-pose.pose.position.y)>.20) return false;
      size_t index=nearest;double arc=0;
      while(index+1<local.poses.size() && arc<std::clamp(.18+.4*std::hypot(speed.linear.x,speed.linear.y),.18,.35)) {
        const auto &a=local.poses[index].pose.position,&b=local.poses[index+1].pose.position;
        arc+=std::hypot(b.x-a.x,b.y-a.y);++index;
      }
      target=local.poses[index];return true;
    } catch(const tf2::TransformException &) {return false;}
  }
  bool directApproachClear(const geometry_msgs::msg::PoseStamped &a,const geometry_msgs::msg::PoseStamped &b) {
    auto map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    Polygon polygon;for(const auto &p:costmap_ros_->getRobotFootprint()) polygon.push_back({p.x,p.y});
    std::vector<Point> obstacles;
    int lx=0,ly=0,ux=0,uy=0;
    map->worldToMapEnforceBounds(std::min(a.pose.position.x,b.pose.position.x)-.5,std::min(a.pose.position.y,b.pose.position.y)-.5,lx,ly);
    map->worldToMapEnforceBounds(std::max(a.pose.position.x,b.pose.position.x)+.5,std::max(a.pose.position.y,b.pose.position.y)+.5,ux,uy);
    for(int y=ly;y<=uy;++y) for(int x=lx;x<=ux;++x) {
      if(map->getCost(x,y)<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      double wx=0,wy=0;map->mapToWorld(x,y,wx,wy);obstacles.push_back({wx,wy,map->getResolution()*std::sqrt(2.0)/2});
    }
    const double yaw=tf2::getYaw(a.pose.orientation),delta=std::remainder(tf2::getYaw(b.pose.orientation)-yaw,2*M_PI);
    const double dx=b.pose.position.x-a.pose.position.x,dy=b.pose.position.y-a.pose.position.y;
    double radius=0;for(const auto &p:polygon) radius=std::max(radius,std::hypot(p.x,p.y));
    const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(delta))/.004)));
    for(int i=0;i<=steps;++i) {
      const double f=static_cast<double>(i)/steps;
      if(collision({a.pose.position.x+f*dx,a.pose.position.y+f*dy,yaw+f*delta},polygon,obstacles,.010)) return false;
    }
    return true;
  }
  geometry_msgs::msg::PoseStamped goal_;nav_msgs::msg::Path path_;bool have_goal_{false};
  double max_speed_{.5},reverse_speed_{.2},lateral_speed_{.3},speed_limit_{.5};
};
}
PLUGINLIB_EXPORT_CLASS(duojin01_navigation::PreciseMPPI,nav2_core::Controller)
