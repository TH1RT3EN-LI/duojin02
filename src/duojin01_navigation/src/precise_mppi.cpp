#include "duojin01_navigation/tracking.hpp"
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
    if(!node->has_parameter(name+".narrow_speed_max"))
      node->declare_parameter(name+".narrow_speed_max",max_speed_);
    narrow_speed_=std::clamp(node->get_parameter(name+".narrow_speed_max").as_double(),0.0,max_speed_);
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
    if(!snapshot(pose,speed)) return stopped(pose);
    geometry_msgs::msg::PoseStamped target;
    try {
      if(!have_goal_) return sampledCommand(pose,speed,checker);
      auto goal=goal_;goal.header.stamp=pose.header.stamp;
      tf2::doTransform(goal,target,freshTransform(pose.header.frame_id,goal.header.frame_id,pose.header.stamp));
    } catch(const tf2::TransformException &) {
      return sampledCommand(pose,speed,checker);
    }
    const double distance=std::hypot(target.pose.position.x-pose.pose.position.x,target.pose.position.y-pose.pose.position.y);
    if(distance<=.45 && directApproachClear(pose,target)) return checked(converge(pose,target,true),pose,speed);
    // Follow a collision-checked short SE2 lattice chord directly. Sampling is
    // reserved for blocked chords / dynamic detours, reducing CPU and jitter.
    if(trackingTarget(pose,target,speed)) return checked(converge(pose,target,false),pose,speed);
    try {return sampledCommand(pose,speed,checker);}
    catch(const std::runtime_error &error) {
      if(std::string(error.what())!="Optimizer fail to compute path") throw;
      if(trackingTarget(pose,target,speed)) return checked(converge(pose,target,false),pose,speed);
      throw;
    }
  }

 private:
  geometry_msgs::msg::TwistStamped converge(const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::PoseStamped &target,bool docking) {
    const double dx=target.pose.position.x-pose.pose.position.x,dy=target.pose.position.y-pose.pose.position.y;
    const double yaw=tf2::getYaw(pose.pose.orientation),dyaw=std::remainder(tf2::getYaw(target.pose.orientation)-yaw,2*M_PI);
    // Speed depends on remaining distance, yaw tracking and obstacle proximity.
    // The independent measured-velocity braking guard remains authoritative.
    const bool narrow=nearObstacle(pose) || nearObstacle(target);
    auto rotation=target;rotation.pose.position=pose.pose.position;
    const bool align_first=!docking && narrow && std::abs(dyaw)>.03 && directApproachClear(pose,rotation);
    const double vx=align_first ? 0 : 2.0*(std::cos(yaw)*dx+std::sin(yaw)*dy);
    const double vy=align_first ? 0 : 2.0*(-std::sin(yaw)*dx+std::cos(yaw)*dy);
    double cap=std::min(speed_limit_,narrow ? narrow_speed_ : max_speed_);
    if(docking) cap=std::min(cap,std::sqrt(.6*std::max(0.0,std::hypot(dx,dy)-.001)));
    cap*=std::clamp(1.0-std::abs(dyaw),.2,1.0);
    // Preserve small lateral corrections instead of dividing them by a long
    // forward lookahead; they otherwise fall into the smoother's deadband.
    const double cx=narrow ? std::clamp(vx,-std::min(reverse_speed_,cap),cap) : vx;
    const double cy=narrow ? std::clamp(vy,-std::min(lateral_speed_,cap),std::min(lateral_speed_,cap)) : vy;
    const double scale=std::min({1.0,cap/std::max(.001,std::hypot(cx,cy)),std::min(lateral_speed_,speed_limit_)/std::max(.001,std::abs(cy)),
      (cx<0 ? reverse_speed_ : max_speed_)/std::max(.001,std::abs(cx))});
    geometry_msgs::msg::TwistStamped result;result.header=pose.header;
    result.twist.linear.x=cx*scale;result.twist.linear.y=cy*scale;
    const double temporal_scale=narrow && !align_first && !docking ?
      std::min(1.0,std::hypot(result.twist.linear.x,result.twist.linear.y)/std::max(.001,2*std::hypot(dx,dy))) : 1.0;
    result.twist.angular.z=std::clamp(1.8*dyaw*temporal_scale,narrow ? -.15 : -.45,narrow ? .15 : .45);
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
    if(nearObstacle(pose) || nearObstacle(target)) {
      // A holonomic bend need not rotate the chassis inside the preceding
      // straight. Retain small lattice alignment corrections, but try a
      // collision-checked fixed heading before a large upcoming turn.
      auto fixed_heading=target;fixed_heading.pose.orientation=pose.pose.orientation;
      const double delta=std::remainder(tf2::getYaw(target.pose.orientation)-tf2::getYaw(pose.pose.orientation),2*M_PI);
      if(std::abs(delta)>.08 && centeredTarget(pose,fixed_heading)) {target=fixed_heading;return true;}
      return centeredTarget(pose,target);
    }
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
      const auto tf=freshTransform(pose.header.frame_id,path_.header.frame_id,pose.header.stamp);
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
    const double yaw=tf2::getYaw(a.pose.orientation),delta=std::remainder(tf2::getYaw(b.pose.orientation)-yaw,2*M_PI);
    const double dx=b.pose.position.x-a.pose.position.x,dy=b.pose.position.y-a.pose.position.y;
    double radius=0;for(const auto &p:polygon_) radius=std::max(radius,std::hypot(p.x,p.y));
    const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(delta))/.004)));
    for(int i=0;i<=steps;++i) {
      const double f=static_cast<double>(i)/steps;
      if(collision({a.pose.position.x+f*dx,a.pose.position.y+f*dy,yaw+f*delta},polygon_,obstacles_,.010)) return false;
    }
    return true;
  }
  geometry_msgs::msg::TransformStamped freshTransform(const std::string &destination,const std::string &source,const builtin_interfaces::msg::Time &stamp) {
    try {return tf_buffer_->lookupTransform(destination,source,stamp,tf2::durationFromSec(.005));}
    catch(const tf2::TransformException &) {
      const auto tf=tf_buffer_->lookupTransform(destination,source,tf2::TimePointZero);
      // AMCL broadcasts slightly into the future. A recent latest transform
      // can bridge publication lag; older localization never bypasses a stop.
      if(std::abs((rclcpp::Time(stamp)-rclcpp::Time(tf.header.stamp)).seconds())>.20) throw;
      return tf;
    }
  }
  geometry_msgs::msg::TwistStamped stopped(const geometry_msgs::msg::PoseStamped &pose) {
    geometry_msgs::msg::TwistStamped result;result.header=pose.header;return result;
  }
  bool snapshot(const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed) {
    if(!std::isfinite(speed.linear.x+speed.linear.y+speed.angular.z+pose.pose.position.x+pose.pose.position.y+tf2::getYaw(pose.pose.orientation))) {
      RCLCPP_WARN_THROTTLE(costmap_ros_->get_logger(),*costmap_ros_->get_clock(),2000,"Local motion stopped: non-finite pose or measured velocity");return false;
    }
    auto map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    polygon_.clear();for(const auto &p:costmap_ros_->getRobotFootprint()) polygon_.push_back({p.x,p.y});
    const double extent=localSnapshotExtent(polygon_,max_speed_,reverse_speed_,lateral_speed_,
      {speed.linear.x,speed.linear.y,speed.angular.z},map->getResolution());
    const double x=pose.pose.position.x,y=pose.pose.position.y;
    if(pose.header.frame_id!=costmap_ros_->getGlobalFrameID() || !std::isfinite(extent) ||
       x-extent<map->getOriginX() || y-extent<map->getOriginY() ||
       x+extent>map->getOriginX()+map->getSizeInMetersX() || y+extent>map->getOriginY()+map->getSizeInMetersY()) {
      RCLCPP_WARN_THROTTLE(costmap_ros_->get_logger(),*costmap_ros_->get_clock(),2000,
        "Local snapshot rejected: frame %s expected %s, pose %.3f %.3f, extent %.3f, origin %.3f %.3f, size %.3f %.3f",
        pose.header.frame_id.c_str(),costmap_ros_->getGlobalFrameID().c_str(),x,y,extent,map->getOriginX(),map->getOriginY(),map->getSizeInMetersX(),map->getSizeInMetersY());return false;
    }
    int lx=0,ly=0,ux=0,uy=0;
    map->worldToMapEnforceBounds(x-extent,y-extent,lx,ly);
    map->worldToMapEnforceBounds(x+extent,y+extent,ux,uy);
    obstacles_.clear();
    for(int cy=ly;cy<=uy;++cy) for(int cx=lx;cx<=ux;++cx) {
      if(map->getCost(cx,cy)<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      double wx=0,wy=0;map->mapToWorld(cx,cy,wx,wy);
      obstacles_.push_back({wx,wy,map->getResolution()*std::sqrt(2.0)/2});
    }
    return !polygon_.empty();
  }
  bool centeredTarget(const geometry_msgs::msg::PoseStamped &pose,geometry_msgs::msg::PoseStamped &target) {
    const double dx=target.pose.position.x-pose.pose.position.x,dy=target.pose.position.y-pose.pose.position.y;
    const double length=std::hypot(dx,dy);
    if(length<.01) return directApproachClear(pose,target);
    const double nx=-dy/length,ny=dx/length;
    for(double lookahead:{1.0,.65,.4}) {
      std::vector<std::pair<double,geometry_msgs::msg::PoseStamped>> candidates;
      for(double offset:{0.,-.01,.01,-.02,.02,-.03,.03}) {
        auto candidate=target;
        candidate.pose.position.x=pose.pose.position.x+lookahead*dx+nx*offset;
        candidate.pose.position.y=pose.pose.position.y+lookahead*dy+ny*offset;
        const double clearance=footprintClearance({candidate.pose.position.x,candidate.pose.position.y,tf2::getYaw(candidate.pose.orientation)},polygon_,obstacles_);
        candidates.emplace_back(clearance-.05*std::abs(offset),candidate);
      }
      std::stable_sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.first>b.first;});
      for(const auto &candidate:candidates) if(directApproachClear(pose,candidate.second)) {target=candidate.second;return true;}
    }
    return false;
  }
  geometry_msgs::msg::TwistStamped checked(geometry_msgs::msg::TwistStamped result,const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed) {
    if(!std::isfinite(result.twist.linear.x+result.twist.linear.y+result.twist.angular.z)) return stopped(pose);
    const Pose start{pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)};
    const Velocity measured{speed.linear.x,speed.linear.y,speed.angular.z};
    if(brakingSweep(start,measured,polygon_,obstacles_,.010,.2,.4,.8)) {
      RCLCPP_WARN_THROTTLE(costmap_ros_->get_logger(),*costmap_ros_->get_clock(),2000,
        "Local measured braking rejected: %zu cells, clearance %.3f, velocity %.3f %.3f %.3f",
        obstacles_.size(),footprintClearance(start,polygon_,obstacles_),measured.x,measured.y,measured.yaw);return stopped(pose);
    }
    for(double scale:{1.,.8,.6,.4,.25,.15,.08,.04}) {
      const Velocity requested{result.twist.linear.x*scale,result.twist.linear.y*scale,result.twist.angular.z*scale};
      bool clear=!brakingSweep(start,requested,polygon_,obstacles_,.010,.2,.4,.8);
      if(clear) for(double f:{.25,.5,.75}) {
        const Velocity transition{measured.x+f*(requested.x-measured.x),measured.y+f*(requested.y-measured.y),measured.yaw+f*(requested.yaw-measured.yaw)};
        if(brakingSweep(start,transition,polygon_,obstacles_,.010,.2,.4,.8)) {clear=false;break;}
      }
      if(clear) {result.twist.linear.x*=scale;result.twist.linear.y*=scale;result.twist.angular.z*=scale;return result;}
    }
    return stopped(pose);
  }
  geometry_msgs::msg::TwistStamped sampledCommand(const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,nav2_core::GoalChecker *checker) {
    return checked(nav2_mppi_controller::MPPIController::computeVelocityCommands(pose,speed,checker),pose,speed);
  }
  Polygon polygon_;std::vector<Point> obstacles_;
  geometry_msgs::msg::PoseStamped goal_;nav_msgs::msg::Path path_;bool have_goal_{false};
  double max_speed_{.5},reverse_speed_{.2},lateral_speed_{.3},speed_limit_{.5},narrow_speed_{.5};
};
}
PLUGINLIB_EXPORT_CLASS(duojin01_navigation::PreciseMPPI,nav2_core::Controller)
