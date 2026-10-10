#include "duojin01_navigation/tracking.hpp"
#include "duojin01_navigation/velocity_limits.hpp"
#include "duojin01_navigation/local_grid_snapshot.hpp"
#include "duojin01_navigation/path_target.hpp"
#include "duojin01_navigation/gpu_backend.hpp"
#include "duojin01_navigation/gpu_sampling.hpp"
#include <chrono>
#include <cstring>
#include <nav2_mppi_controller/controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <mutex>

namespace duojin01_navigation {
class PreciseMPPI : public nav2_mppi_controller::MPPIController {
 public:
  ~PreciseMPPI() override {if(!statistics_reported_)reportStatistics();}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    nav2_mppi_controller::MPPIController::configure(parent,name,tf,costmap);
    statistics_reported_=false;
    native_samples_=cuda_samples_=best_sequences_=gpu_failures_=gpu_deadline_misses_=0;
    auto node=parent.lock();
    limits_.configure({node->get_parameter(name+".vx_max").as_double(),
      node->get_parameter(name+".vx_min").as_double(),
      node->get_parameter(name+".vy_max").as_double(),
      node->get_parameter(name+".wz_max").as_double()});
    const double max_speed=limits_.configured().vx_max;
    if(!node->has_parameter(name+".narrow_speed_max"))
      node->declare_parameter(name+".narrow_speed_max",max_speed);
    const double narrow_speed=node->get_parameter(name+".narrow_speed_max").as_double();
    if(!std::isfinite(narrow_speed) || narrow_speed<0)
      throw std::invalid_argument("PreciseMPPI narrow_speed_max must be finite and nonnegative");
    narrow_speed_=std::min(narrow_speed,max_speed);
    if(!node->has_parameter(name+".compute_backend"))node->declare_parameter(name+".compute_backend",std::string("cpu"));
    if(!node->has_parameter(name+".gpu_min_work"))node->declare_parameter(name+".gpu_min_work",int64_t(262144));
    if(!node->has_parameter(name+".gpu_deadline_ms"))node->declare_parameter(name+".gpu_deadline_ms",40.);
    gpu_deadline_ms_=node->get_parameter(name+".gpu_deadline_ms").as_double();
    if(!std::isfinite(gpu_deadline_ms_) || gpu_deadline_ms_<=0 || gpu_deadline_ms_>50)
      throw std::invalid_argument("gpu_deadline_ms must be in (0, 50]");
    const auto backend=parseComputeBackend(node->get_parameter(name+".compute_backend").as_string());
    const auto work=node->get_parameter(name+".gpu_min_work").as_int();
    if(work<0)throw std::invalid_argument("gpu_min_work must be nonnegative");
    if(backend!=ComputeBackend::Cpu) {
      geometry_=std::make_unique<GeometryCompute>(ComputeOptions{backend,static_cast<uint64_t>(work),node->get_parameter(name+".gpu_deadline_ms").as_double()});
      RCLCPP_INFO(logger_,"Navigation compute backend: %s (%s)",node->get_parameter(name+".compute_backend").as_string().c_str(),geometry_->deviceName().c_str());
    }
    if(!node->has_parameter(name+".sampling_backend"))node->declare_parameter(name+".sampling_backend",std::string("original"));
    if(!node->has_parameter(name+".gpu_support_points"))node->declare_parameter(name+".gpu_support_points",int64_t(8));
    const auto sampling=node->get_parameter(name+".sampling_backend").as_string();
    if(sampling!="original" && sampling!="cuda" && sampling!="auto")
      throw std::invalid_argument("sampling_backend must be original, cuda or auto");
    if(sampling!="original") {
      if(node->get_parameter(name+".motion_model").as_string()!="Omni")
        throw std::invalid_argument("Optional CUDA sampling supports the Omni model");
      SamplingSettings settings;
      const auto batch=node->get_parameter(name+".batch_size").as_int(),steps=node->get_parameter(name+".time_steps").as_int();
      const auto support=node->get_parameter(name+".gpu_support_points").as_int();
      if(batch<2 || steps<1 || support<1)throw std::invalid_argument("Invalid GPU sampling dimensions");
      settings.batch_size=batch;settings.time_steps=steps;settings.support_points=std::min(support,steps);
      settings.dt=node->get_parameter(name+".model_dt").as_double();
      settings.temperature=node->get_parameter(name+".temperature").as_double();
      settings.gamma=node->get_parameter(name+".gamma").as_double();
      settings.noise={node->get_parameter(name+".vx_std").as_double(),node->get_parameter(name+".vy_std").as_double(),node->get_parameter(name+".wz_std").as_double()};
      sampling_=std::make_unique<SamplingCompute>(settings,ComputeOptions{parseComputeBackend(sampling),0,gpu_deadline_ms_});
      RCLCPP_INFO(logger_,"Navigation sampling backend: %s (%s); time-interpolated sampling, filled sweeps and validated sequence selection",sampling.c_str(),sampling_->deviceName().c_str());
    }
  }
  void cleanup() override {
    reportStatistics();
    sampling_.reset();sample_path_=nav_msgs::msg::Path();geometry_.reset();
    nav2_mppi_controller::MPPIController::cleanup();
  }
  void reportStatistics() {
    if(statistics_reported_)return;
    statistics_reported_=true;
    RCLCPP_INFO(logger_,"GPU_CONTROL_SUMMARY native_samples=%lu cuda_samples=%lu best_sequence=%lu gpu_failures=%lu deadline_misses=%lu",
      native_samples_,cuda_samples_,best_sequences_,gpu_failures_,gpu_deadline_misses_);
    if(geometry_) {
      const auto &s=geometry_->statistics();
      RCLCPP_INFO(logger_,"GPU_GEOMETRY_SUMMARY cuda_batches=%lu cpu_batches=%lu boundary_rechecks=%lu",s.cuda_batches,s.cpu_batches,s.boundary_rechecks);
    }
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    nav2_mppi_controller::MPPIController::setPlan(path);
    target_path_.setPath(path);
    if(sampling_) {sample_path_=path;sampling_->reset();}
    have_goal_=!path.poses.empty();if(have_goal_) goal_=path.poses.back();
  }
  void setSpeedLimit(const double &limit,const bool &percentage) override {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    requested_limit_=limit;requested_percentage_=percentage;
    if(!limits_.update(limit,percentage))
      RCLCPP_WARN(logger_,"Invalid speed limit: all motion is stopped until a valid limit or reset is received");
    applyNativeLimit();
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    const auto begin=std::chrono::steady_clock::now();
    try {
      auto result=computeCommand(pose,speed,checker);
      if((geometry_ && geometry_->available()) || (sampling_ && sampling_->available())) {
        const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        if(elapsed>gpu_deadline_ms_) {
          ++gpu_deadline_misses_;
          RCLCPP_WARN_THROTTLE(logger_,*clock_,2000,"GPU control whole-call deadline exceeded: %.3f ms",elapsed);
          return stopped(pose);
        }
      }
      return result;
    } catch(const ComputeFailure &error) {
      ++gpu_failures_;
      RCLCPP_ERROR_THROTTLE(logger_,*clock_,2000,"GPU computation stopped motion: %s",error.what());
      return stopped(pose);
    }
  }

 private:
  geometry_msgs::msg::TwistStamped computeCommand(const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &speed,nav2_core::GoalChecker *checker) {
    if(!refreshLimits()) return stopped(pose);
    if(limits_.stopped()) return stopped(pose);
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
    const auto &bounds=limits_.effective();
    const double reverse_speed=-bounds.vx_min;
    double cap=std::min(bounds.vx_max,narrow ? narrow_speed_ : limits_.configured().vx_max);
    if(docking) cap=std::min(cap,std::sqrt(.6*std::max(0.0,std::hypot(dx,dy)-.001)));
    cap*=std::clamp(1.0-std::abs(dyaw),.2,1.0);
    // Preserve small lateral corrections instead of dividing them by a long
    // forward lookahead; they otherwise fall into the smoother's deadband.
    const double cx=narrow ? std::clamp(vx,-std::min(reverse_speed,cap),cap) : vx;
    const double cy=narrow ? std::clamp(vy,-std::min(bounds.vy_max,cap),std::min(bounds.vy_max,cap)) : vy;
    const double lateral_scale=std::abs(cy)<1e-12 ? 1.0 : std::min(bounds.vy_max,bounds.vx_max)/std::max(.001,std::abs(cy));
    const double scale=std::min({1.0,cap/std::max(.001,std::hypot(cx,cy)),lateral_scale,
      (cx<0 ? reverse_speed : bounds.vx_max)/std::max(.001,std::abs(cx))});
    geometry_msgs::msg::TwistStamped result;result.header=pose.header;
    result.twist.linear.x=cx*scale;result.twist.linear.y=cy*scale;
    const double temporal_scale=narrow && !align_first && !docking ?
      std::min(1.0,std::hypot(result.twist.linear.x,result.twist.linear.y)/std::max(.001,2*std::hypot(dx,dy))) : 1.0;
    const double yaw_cap=std::min(bounds.wz_max,narrow ? .15 : .45);
    result.twist.angular.z=std::clamp(1.8*dyaw*temporal_scale,-yaw_cap,yaw_cap);
    return result;
  }
  bool nearObstacle(const geometry_msgs::msg::PoseStamped &pose) {
    return grid_.nearObstacle(pose.pose.position.x,pose.pose.position.y);
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
    if(target_path_.empty()) return false;
    try {
      const auto tf=freshTransform(pose.header.frame_id,target_path_.frameId(),pose.header.stamp);
      return target_path_.select(pose,tf,speed,target);
    } catch(const tf2::TransformException &) {return false;}
  }
  bool directApproachClear(const geometry_msgs::msg::PoseStamped &a,const geometry_msgs::msg::PoseStamped &b) {
    const double yaw=tf2::getYaw(a.pose.orientation),delta=std::remainder(tf2::getYaw(b.pose.orientation)-yaw,2*M_PI);
    const double dx=b.pose.position.x-a.pose.position.x,dy=b.pose.position.y-a.pose.position.y;
    double radius=0;for(const auto &p:polygon_) radius=std::max(radius,std::hypot(p.x,p.y));
    const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(delta))/.004)));
    if(geometry_ && geometry_->willUseCuda(steps+1)) {
      if(collision({a.pose.position.x,a.pose.position.y,yaw},polygon_,obstacles_,.010))return false;
      std::vector<Pose> poses;poses.reserve(steps+1);
      for(int i=0;i<=steps;++i) {const double f=static_cast<double>(i)/steps;poses.push_back({a.pose.position.x+f*dx,a.pose.position.y+f*dy,yaw+f*delta});}
      const auto results=geometry_->evaluate(poses,.010);
      return std::none_of(results.collisions.begin(),results.collisions.end(),[](uint8_t hit){return hit!=0;});
    }
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
    const auto &configured=limits_.configured();
    const double extent=localSnapshotExtent(polygon_,configured.vx_max,-configured.vx_min,configured.vy_max,
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
    grid_.width=ux-lx+1;grid_.height=uy-ly+1;grid_.resolution=map->getResolution();
    grid_.origin_x=map->getOriginX();grid_.origin_y=map->getOriginY();
    grid_.crop_x=lx;grid_.crop_y=ly;
    grid_.data.resize(static_cast<size_t>(grid_.width)*grid_.height);
    for(unsigned int row=0;row<grid_.height;++row)
      std::memcpy(grid_.data.data()+row*grid_.width,
        map->getCharMap()+(ly+row)*map->getSizeInCellsX()+lx,grid_.width);
    lock.unlock();
    obstacles_.clear();
    for(unsigned int cy=0;cy<grid_.height;++cy) for(unsigned int cx=0;cx<grid_.width;++cx) {
      if(grid_.data[cy*grid_.width+cx]<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
      const double wx=grid_.origin_x+(cx+grid_.crop_x+.5)*grid_.resolution;
      const double wy=grid_.origin_y+(cy+grid_.crop_y+.5)*grid_.resolution;
      obstacles_.push_back({wx,wy,grid_.resolution*std::sqrt(2.0)/2});
    }
    if(geometry_ && !polygon_.empty())geometry_->setScene(polygon_,obstacles_);
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
      if(geometry_ && geometry_->available()) {
        if(collision({pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)},polygon_,obstacles_,.010))return false;
        std::vector<Pose> poses;std::vector<size_t> ends;
        const double yaw=tf2::getYaw(pose.pose.orientation);double radius=0;
        for(const auto &p:polygon_)radius=std::max(radius,std::hypot(p.x,p.y));
        for(const auto &candidate:candidates) {
          const auto &b=candidate.second;
          const double dx=b.pose.position.x-pose.pose.position.x,dy=b.pose.position.y-pose.pose.position.y;
          const double delta=std::remainder(tf2::getYaw(b.pose.orientation)-yaw,2*M_PI);
          const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(delta))/.004)));
          for(int i=0;i<=steps;++i) {const double f=static_cast<double>(i)/steps;poses.push_back({pose.pose.position.x+f*dx,pose.pose.position.y+f*dy,yaw+f*delta});}
          ends.push_back(poses.size());
        }
        if(geometry_->willUseCuda(poses.size())) {
          const auto results=geometry_->evaluate(poses,.010);size_t begin=0;
          for(size_t i=0;i<ends.size();++i) {
            const bool hit=std::any_of(results.collisions.begin()+begin,results.collisions.begin()+ends[i],[](uint8_t value){return value!=0;});
            if(!hit){target=candidates[i].second;return true;}begin=ends[i];
          }
          continue;
        }
      }
      for(const auto &candidate:candidates) if(directApproachClear(pose,candidate.second)) {target=candidate.second;return true;}
    }
    return false;
  }
  geometry_msgs::msg::TwistStamped checked(geometry_msgs::msg::TwistStamped result,const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed) {
    if(!std::isfinite(result.twist.linear.x+result.twist.linear.y+result.twist.angular.z)) return stopped(pose);
    const auto bounded=limits_.clamp({result.twist.linear.x,result.twist.linear.y,result.twist.angular.z});
    result.twist.linear.x=bounded.x;result.twist.linear.y=bounded.y;result.twist.angular.z=bounded.yaw;
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
    if(sampling_ && sampling_->available()) {
      try {
      auto tf=freshTransform(pose.header.frame_id,sample_path_.header.frame_id,pose.header.stamp);
      std::vector<Pose> transformed;transformed.reserve(sample_path_.poses.size());
      size_t nearest=0;double minimum=1e100;
      for(const auto &source:sample_path_.poses) {
        auto input=source;input.header.frame_id=sample_path_.header.frame_id;input.header.stamp=pose.header.stamp;
        geometry_msgs::msg::PoseStamped output;tf2::doTransform(input,output,tf);
        const Pose p{output.pose.position.x,output.pose.position.y,tf2::getYaw(output.pose.orientation)};
        const double d=std::hypot(p.x-pose.pose.position.x,p.y-pose.pose.position.y);
        if(d<minimum){minimum=d;nearest=transformed.size();}transformed.push_back(p);
      }
      if(transformed.empty())return stopped(pose);
      const size_t first=nearest ? nearest-1:0;size_t last=nearest;double distance=0;
      while(last+1<transformed.size() && distance<1.8){distance+=std::hypot(transformed[last+1].x-transformed[last].x,transformed[last+1].y-transformed[last].y);++last;}
      const size_t stride=std::max<size_t>(1,(last-first+63)/64);
      std::vector<Pose> path;for(size_t i=first;i<=last;i+=stride)path.push_back(transformed[i]);
      if((last-first)%stride)path.push_back(transformed[last]);
      LocalGridSnapshot full;
      auto map=costmap_ros_->getCostmap();
      {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
        const auto footprint=costmap_ros_->getRobotFootprint();
        if(footprint.size()!=polygon_.size())return stopped(pose);
        for(size_t i=0;i<footprint.size();++i)if(footprint[i].x!=polygon_[i].x || footprint[i].y!=polygon_[i].y)return stopped(pose);
        full.width=map->getSizeInCellsX();full.height=map->getSizeInCellsY();full.resolution=map->getResolution();
        full.origin_x=map->getOriginX();full.origin_y=map->getOriginY();
        if(static_cast<size_t>(full.width)*full.height>16000000)throw ComputeFailure("GPU sampling local map exceeds admitted cell count");
        full.data.assign(map->getCharMap(),map->getCharMap()+static_cast<size_t>(full.width)*full.height);
      }
      const auto sample=sampling_->optimize(full,polygon_,path,
        {pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)},
        {speed.linear.x,speed.linear.y,speed.angular.z},limits_.effective());
      ++cuda_samples_;best_sequences_+=sample.used_best_trajectory;
      if(!sample.valid)return stopped(pose);
      geometry_msgs::msg::TwistStamped result;result.header=pose.header;
      result.twist.linear.x=sample.command.x;result.twist.linear.y=sample.command.y;result.twist.angular.z=sample.command.yaw;
      return checked(result,pose,speed);
      } catch(const ComputeFailure &) {throw;}
      catch(const std::exception &e) {throw ComputeFailure(std::string("CUDA sampling input/result rejected: ")+e.what());}
    }
    // Humble's periodic reset restores base constraints. Reset before reapplying
    // the current request; checked() also bounds a native fallback retry that
    // performs its own internal reset or filtering.
    applyNativeLimit(true);
    last_time_called_=clock_->now();
    ++native_samples_;
    return checked(nav2_mppi_controller::MPPIController::computeVelocityCommands(pose,speed,checker),pose,speed);
  }
  void applyNativeLimit(bool periodic_reset=false) {
    std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
    if(periodic_reset && clock_->now()-last_time_called_>rclcpp::Duration::from_seconds(reset_period_))
      nav2_mppi_controller::MPPIController::reset();
    nav2_mppi_controller::MPPIController::setSpeedLimit(limits_.nativePercentage(),true);
  }
  bool refreshLimits() {
    auto node=parent_.lock();if(!node) return false;
    try {
      OmniVelocityLimits current;
      const auto parameters=node->get_parameters({name_+".vx_max",name_+".vx_min",
        name_+".vy_max",name_+".wz_max",name_+".narrow_speed_max"});
      current.configure({parameters[0].as_double(),parameters[1].as_double(),
        parameters[2].as_double(),parameters[3].as_double()});
      const double narrow=parameters[4].as_double();
      if(!std::isfinite(narrow) || narrow<0)
        throw std::invalid_argument("PreciseMPPI narrow_speed_max must be finite and nonnegative");
      current.update(requested_limit_,requested_percentage_);
      limits_=current;narrow_speed_=std::min(narrow,current.configured().vx_max);
      return true;
    } catch(const std::exception &error) {
      RCLCPP_WARN_THROTTLE(logger_,*clock_,2000,"Invalid speed configuration: motion stopped: %s",error.what());
      return false;
    }
  }
  Polygon polygon_;std::vector<Point> obstacles_;
  LocalGridSnapshot grid_;
  geometry_msgs::msg::PoseStamped goal_;PathTarget target_path_;bool have_goal_{false};
  OmniVelocityLimits limits_;std::mutex limits_mutex_;
  std::unique_ptr<GeometryCompute> geometry_;
  std::unique_ptr<SamplingCompute> sampling_;nav_msgs::msg::Path sample_path_;
  double gpu_deadline_ms_{40};
  uint64_t native_samples_{0},cuda_samples_{0},best_sequences_{0},gpu_failures_{0},gpu_deadline_misses_{0};
  bool statistics_reported_{false};
  double narrow_speed_{.5},requested_limit_{0};bool requested_percentage_{false};
};
}
PLUGINLIB_EXPORT_CLASS(duojin01_navigation::PreciseMPPI,nav2_core::Controller)
