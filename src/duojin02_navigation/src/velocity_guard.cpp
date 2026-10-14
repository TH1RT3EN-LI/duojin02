#include "duojin02_navigation/sweep.hpp"
#include "duojin02_navigation/scan_validity.hpp"
#include "duojin02_navigation/guard_query.hpp"
#include "duojin02_navigation/guard_input_validity.hpp"
#include "duojin02_navigation/prior_association_cache.hpp"
#include <deque>
#include <chrono>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

using namespace duojin02_navigation;
using Clock = std::chrono::steady_clock;
class VelocityGuard : public rclcpp::Node {
 public:
  VelocityGuard():Node("velocity_guard"),buffer_(get_clock()),listener_(buffer_,this,false) {
    auto flat=declare_parameter<std::vector<double>>("footprint",std::vector<double>{});
    if (flat.size()<6 || flat.size()%2) throw std::runtime_error("A measured convex CCW footprint is required");
    for (size_t i=0;i<flat.size();i+=2) polygon_.push_back({flat[i],flat[i+1]});
    for(size_t i=0;i<polygon_.size();++i) {
      const auto &a=polygon_[i],&b=polygon_[(i+1)%polygon_.size()],&c=polygon_[(i+2)%polygon_.size()];
      if(!std::isfinite(a.x)||!std::isfinite(a.y)||std::hypot(a.x-b.x,a.y-b.y)<1e-6 ||
         (b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x)<-1e-10)
        throw std::runtime_error("Footprint must be finite, convex, CCW and have distinct adjacent vertices");
    }
    margin_=declare_parameter("clearance_margin_m",.008);
    range_error_=declare_parameter("range_error_m",.008);
    reaction_=declare_parameter("reaction_time_s",.20);
    decel_=declare_parameter("braking_deceleration_mps2",.4);
    angular_decel_=declare_parameter("angular_deceleration_radps2",.8);
    scan_timeout_=declare_parameter("scan_timeout_s",.30);
    command_timeout_=declare_parameter("command_timeout_s",.25);
    odometry_timeout_=declare_parameter("odometry_timeout_s",.20);
    localization_tf_timeout_=declare_parameter("localization_tf_timeout_s",.50);
    memory_radius_=declare_parameter("obstacle_memory_radius_m",2.5);
    if (!std::isfinite(margin_+range_error_+decel_+angular_decel_+reaction_+scan_timeout_+command_timeout_+odometry_timeout_+localization_tf_timeout_+memory_radius_) ||
        margin_<0 || range_error_<0 || decel_<=0 || angular_decel_<=0 || reaction_<0 ||
        scan_timeout_<=0 || command_timeout_<=0 || odometry_timeout_<=0 || localization_tf_timeout_<=0 || memory_radius_<=0)
      throw std::runtime_error("Invalid guard limits");
    auto qos=rclcpp::QoS(1);
    output_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_base",qos);
    status_=create_publisher<std_msgs::msg::String>("/navigation/guard_status",qos);
    memory_output_=create_publisher<sensor_msgs::msg::PointCloud2>("/navigation/obstacle_memory",rclcpp::SensorDataQoS());
    command_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_requested",qos,
      [this](geometry_msgs::msg::Twist::SharedPtr m){command_=*m;last_command_=Clock::now();have_command_=true;});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m){
        if(m->header.frame_id!="odom" || m->child_frame_id!="base_footprint" ||
            !std::isfinite(m->pose.pose.position.x+m->pose.pose.position.y+
            m->twist.twist.linear.x+m->twist.twist.linear.y+m->twist.twist.angular.z) ||
            std::abs((now()-rclcpp::Time(m->header.stamp)).seconds())>odometry_timeout_) {have_odom_=false;return;}
        pose_={m->pose.pose.position.x,m->pose.pose.position.y,tf2::getYaw(m->pose.pose.orientation)};
        actual_={m->twist.twist.linear.x,m->twist.twist.linear.y,m->twist.twist.angular.z};
        if(!std::isfinite(pose_.yaw)) {have_odom_=false;return;}
        last_odom_=Clock::now();have_odom_=true;
      });
    // TF readiness is event driven. Listener and filter callbacks share this
    // node's single executor, so they cannot race the safety state or timer.
    buffer_.setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),get_node_timers_interface()));
    scan_filter_=std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      buffer_,"odom",2,get_node_logging_interface(),get_node_clock_interface(),std::chrono::milliseconds(20));
    scan_filter_->registerCallback([this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){++tf_ready_scans_;scan(m);});
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){receiveScan(m);});
    prior_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){
        if(m->header.frame_id!="map" || !guardGridValid(*m)) {prior_invalid_=true;return;}
        Pose origin{m->info.origin.position.x,m->info.origin.position.y,tf2::getYaw(m->info.origin.orientation)};
        if(!guardGridExtentValid(*m,origin,{},prior_index_bin_)) {prior_invalid_=true;return;}
        std::vector<Point> points;
        for(size_t i=0;i<m->data.size();++i) if(m->data[i]>=65) {
          Point p{(i%m->info.width+.5)*m->info.resolution,(i/m->info.width+.5)*m->info.resolution,
                  std::sqrt(2.0)*m->info.resolution/2};
          const auto point=transform(p,origin);
          if(!guardPointValid(point,prior_index_bin_)) {prior_invalid_=true;return;}
          points.push_back(point);
        }
        prior_points_=std::move(points);
        prior_index_.rebuild(prior_points_);
        ++prior_epoch_;
        have_prior_=true;prior_invalid_=false;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Filled footprint braking sweep: margin %.3f m, reaction %.2f s; all commands are gated",margin_,reaction_);
  }
 private:
  static double age(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
  Pose framePose(const std::string &frame,const builtin_interfaces::msg::Time &stamp) {
    if (frame=="odom") return {};
    // Humble Buffer's timeout overload requires a dedicated listener even
    // for a zero duration. BufferCore queries the already populated cache
    // immediately, as required by this single-executor MessageFilter owner.
    auto tf=static_cast<const tf2::BufferCore &>(buffer_).lookupTransform(
      "odom",frame,tf2_ros::fromRclcpp(rclcpp::Time(stamp)));
    Pose pose{tf.transform.translation.x,tf.transform.translation.y,tf2::getYaw(tf.transform.rotation)};
    if(!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))
      throw tf2::TransformException("Non-finite planar sensor transform");
    return pose;
  }
  void invalidScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &m) {
    scan_order_.fault(validScanStamp(m->header.stamp) ? scanStampNanoseconds(m->header.stamp) : -1);
    have_scan_=false;scan_invalid_=true;
    publish({},"scan_invalid",0,Clock::now());
  }
  void receiveScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &m) {
    // Malformed input stops immediately, even if its sensor TF is missing.
    if(!validScanStamp(m->header.stamp)) {invalidScan(m);return;}
    if(std::abs((now()-rclcpp::Time(m->header.stamp)).seconds())>scan_timeout_) return;
    if(scanValidity(*m)!=ScanValidity::Valid) {invalidScan(m);return;}
    const auto stamp=scanStampNanoseconds(m->header.stamp);
    if(!scan_order_.canAccept(stamp)) return;
    for(const auto &receipt:scan_receipts_) if(receipt.first==stamp) return;
    scan_receipts_.emplace_back(stamp,Clock::now());
    while(scan_receipts_.size()>2) scan_receipts_.pop_front();
    scan_filter_->add(m);
  }
  void scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &m) {
    const auto stamp=scanStampNanoseconds(m->header.stamp);
    auto receipt=std::find_if(scan_receipts_.begin(),scan_receipts_.end(),
      [stamp](const auto &entry){return entry.first==stamp;});
    if(receipt==scan_receipts_.end()) return;
    const auto received=receipt->second;scan_receipts_.erase(receipt);
    if(!validScanStamp(m->header.stamp)) {invalidScan(m);return;}
    try {
      if (std::abs((now()-rclcpp::Time(m->header.stamp)).seconds())>scan_timeout_) return;
      if(scanValidity(*m)!=ScanValidity::Valid) {invalidScan(m);return;}
      if(age(received)>scan_timeout_) return;
      if(!scan_order_.canAccept(scanStampNanoseconds(m->header.stamp))) return;
      auto transform_pose=framePose(m->header.frame_id,m->header.stamp);
      // Correct remembered surfaces when localization compensates odometry
      // drift. Fresh observations remain in sensor-time odom coordinates.
      // Without this anchoring, an old wall can drift into the footprint.
      try {
        if(!static_cast<const tf2::BufferCore &>(buffer_).canTransform(
            "odom","map",tf2_ros::fromRclcpp(rclcpp::Time(m->header.stamp)),nullptr)) {
          if(have_anchor_) return;
          throw tf2::TransformException("No global localization yet");
        }
        auto anchor=framePose("map",m->header.stamp);
        if(have_anchor_) {
          const double ac=std::cos(memory_anchor_.yaw),as=std::sin(memory_anchor_.yaw);
          Pose inverse{-ac*memory_anchor_.x-as*memory_anchor_.y,as*memory_anchor_.x-ac*memory_anchor_.y,-memory_anchor_.yaw};
          for(auto &p:scan_points_) p=transform(transform(p,inverse),anchor);
        }
        memory_anchor_=anchor;have_anchor_=true;
      } catch(const tf2::TransformException &) {if(have_anchor_) return;}
      std::unordered_map<uint64_t,Point> memory;
      auto retain=[&](const Point &p) {
        if(!std::isfinite(p.x+p.y+p.radius) || std::abs(p.x)>1e6 || std::abs(p.y)>1e6) return;
        Point index=p;
        if(have_anchor_) {
          const double ac=std::cos(memory_anchor_.yaw),as=std::sin(memory_anchor_.yaw);
          index=transform(p,{-ac*memory_anchor_.x-as*memory_anchor_.y,as*memory_anchor_.x-ac*memory_anchor_.y,-memory_anchor_.yaw});
        }
        // Fixed map bins prevent repeated rebinning from growing the radius
        // of a blind-zone point on every localization correction.
        if(!guardPointValid(index,.005)) throw tf2::TransformException("Invalid obstacle memory index coordinates");
        const auto x=static_cast<int32_t>(std::floor(index.x/.005)),y=static_cast<int32_t>(std::floor(index.y/.005));
        const uint64_t key=(static_cast<uint64_t>(static_cast<uint32_t>(x))<<32)|static_cast<uint32_t>(y);
        auto found=memory.find(key);
        if(found==memory.end()) memory.emplace(key,p);
        else found->second.radius=std::max(found->second.radius,p.radius+std::hypot(p.x-found->second.x,p.y-found->second.y));
      };
      const double c=std::cos(transform_pose.yaw),s=std::sin(transform_pose.yaw);
      for(const auto &p:scan_points_) {
        const double dx=p.x-transform_pose.x,dy=p.y-transform_pose.y,r=std::hypot(dx,dy);
        if(r>memory_radius_) continue;
        const double bearing=std::atan2(-s*dx+c*dy,c*dx+s*dy);
        int bin=static_cast<int>(std::round((bearing-m->angle_min)/m->angle_increment));
        if(std::abs(m->angle_increment)*m->ranges.size()>6.0) bin=(bin%static_cast<int>(m->ranges.size())+m->ranges.size())%m->ranges.size();
        // Blind / occluded / invalid rays do not prove an obstacle disappeared.
        // Only a valid farther return clears the remembered obstacle surface.
        bool observed_free=false,reobserved=false;
        if(r>=m->range_min+range_error_ && bin>=0 && bin<static_cast<int>(m->ranges.size())) {
          const double observed=m->ranges[bin];
          observed_free=std::isfinite(observed) && observed>=m->range_min && observed<=m->range_max && observed>r+.04+p.radius;
          reobserved=std::isfinite(observed) && observed>=m->range_min && observed<=m->range_max &&
            std::abs(observed-r)<=2*range_error_+r*std::abs(m->angle_increment)/2+.015;
        }
        // Replace a reobserved surface with its current sensor-time position,
        // instead of indefinitely accumulating odometry drift and noise minima.
        if(!observed_free&&!reobserved) retain(p);
      }
      for (size_t i=0;i<m->ranges.size();++i) {
        double r=m->ranges[i];
        if (!std::isfinite(r) || r<m->range_min || r>std::min(static_cast<float>(memory_radius_),m->range_max)) continue;
        const double angle=m->angle_min+i*m->angle_increment;
        retain(transform({r*std::cos(angle),r*std::sin(angle),
            range_error_+r*std::abs(m->angle_increment)/2},transform_pose));
      }
      scan_points_.clear();scan_points_.reserve(memory.size());
      for(const auto &entry:memory) scan_points_.push_back(entry.second);
      observed_index_.rebuild(scan_points_);
      ++observation_epoch_;
      sensor_msgs::msg::PointCloud2 cloud;cloud.header=m->header;
      sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(scan_points_.size());
      sensor_msgs::PointCloud2Iterator<float> cx(cloud,"x"),cy(cloud,"y"),cz(cloud,"z");
      for(const auto &p:scan_points_) {
        const double dx=p.x-transform_pose.x,dy=p.y-transform_pose.y;
        *cx=static_cast<float>(c*dx+s*dy);*cy=static_cast<float>(-s*dx+c*dy);*cz=0;
        ++cx;++cy;++cz;
      }
      memory_output_->publish(cloud);
      scan_order_.accepted(scanStampNanoseconds(m->header.stamp));
      last_scan_=received;have_scan_=true;scan_invalid_=false;
    } catch (const tf2::TransformException &) { /* Fail closed until fresh sensor TF is available. */ }
  }
  bool blocked(Velocity v,const std::vector<Point> &points) {
    return brakingSweep(pose_,v,polygon_,points,margin_,reaction_,decel_,angular_decel_);
  }
  std::vector<Point> obstacles(const Velocity &requested) {
    // The guard consumes measured geometry. Nav2's generated costmap and AMCL
    // quality estimates do not add a second obstacle or clearance envelope.
    std::vector<Point> points=scan_points_;
    if(!have_prior_) return points;
    auto tf=buffer_.lookupTransform("odom","map",tf2::TimePointZero);
    if(std::abs((now()-rclcpp::Time(tf.header.stamp)).seconds())>localization_tf_timeout_)
      throw tf2::TransformException("Stale localization transform");
    Pose map_pose{tf.transform.translation.x,tf.transform.translation.y,tf2::getYaw(tf.transform.rotation)};
    if(!guardPoseValid(map_pose)) throw tf2::TransformException("Non-finite localization transform");
    // Index once per map update. Transform only nearby cells per control tick,
    // so a large warehouse does not add a full-map scan to every brake check.
    const double c=std::cos(map_pose.yaw),s=std::sin(map_pose.yaw);
    const double mx=c*(pose_.x-map_pose.x)+s*(pose_.y-map_pose.y);
    const double my=-s*(pose_.x-map_pose.x)+c*(pose_.y-map_pose.y);
    query_radius_=guardQueryRadius(polygon_,actual_,requested,margin_,reaction_,decel_,
      prior_index_.maxRadius(),memory_radius_);
    if(!guardIndexQueryValid(mx,my,query_radius_,prior_index_bin_,true))
      throw tf2::TransformException("Invalid prior index query coordinates");
    association_cache_.begin(prior_points_.size(),
      {prior_epoch_,observation_epoch_,map_pose,range_error_});
    prior_examined_=prior_index_.forEachNearIndexed(prior_points_,mx,my,query_radius_,[&](const Point &q,size_t index) {
        Point p{map_pose.x+c*q.x-s*q.y,map_pose.y+s*q.x+c*q.y,q.radius};
        if(std::hypot(p.x-pose_.x,p.y-pose_.y)>memory_radius_) return;
        const double association=p.radius+range_error_+.015;
        const bool matched=association_cache_.match(index,[&](){
          if(!guardIndexQueryValid(p.x,p.y,association,observed_index_bin_,false))
            throw tf2::TransformException("Invalid observation association coordinates");
          return observed_index_.hasWithin(scan_points_,p,association);
        });
        if(!matched) points.push_back(p);
    });
    candidate_points_=points.size();
    return points;
  }
  void tick() {
    const auto start=Clock::now();
    geometry_msgs::msg::Twist result;
    std::string reason="clear";
    double scale=0;
    if (!have_command_ || age(last_command_)>command_timeout_) reason="command_stale";
    else if (scan_invalid_) reason="scan_invalid";
    else if (!have_scan_ || age(last_scan_)>scan_timeout_) reason="scan_or_tf_stale";
    else if (!have_odom_ || age(last_odom_)>odometry_timeout_) reason="odometry_stale";
    else if (prior_invalid_) reason="map_invalid";
    else {
      Velocity requested{command_.linear.x,command_.linear.y,command_.angular.z};
      if (!std::isfinite(requested.x)||!std::isfinite(requested.y)||!std::isfinite(requested.yaw)) {
        publish(result,"invalid_command",scale,start);return;
      }
      std::vector<Point> points;
      try {points=obstacles(requested);}
      catch(const tf2::TransformException &) {reason="localization_tf_stale";publish(result,reason,scale,start);return;}
      if (blocked(actual_,points)) {
        reason="measured_braking_collision";
        const double c=std::cos(pose_.yaw),s=std::sin(pose_.yaw);
        double closest=1e10;Point near{};
        for(const auto &p:points) {
          const double dx=p.x-pose_.x,dy=p.y-pose_.y;
          Point q{c*dx+s*dy,-s*dx+c*dy,p.radius};
          double distance=1e10;
          for(size_t i=0;i<polygon_.size();++i) distance=std::min(distance,distanceToSegment(q,polygon_[i],polygon_[(i+1)%polygon_.size()]));
          if(distance-p.radius<closest) {closest=distance-p.radius;near=q;}
        }
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Braking stop: local point (%.3f, %.3f), radius %.3f, edge gap %.3f, velocity (%.3f, %.3f, %.3f)",near.x,near.y,near.radius,closest,actual_.x,actual_.y,actual_.yaw);
      }
      else {
        for (double candidate:{1.0,.8,.6,.4,.25,.15,.08,.04}) {
          Velocity v{requested.x*candidate,requested.y*candidate,requested.yaw*candidate};
          bool safe=!blocked(v,points);
          // Validate intermediate velocities during a change of translation /
          // rotation. Endpoint sweeps alone can miss a curved transition.
          if(safe) for(double f:{.25,.5,.75}) {
            Velocity intermediate{actual_.x+f*(v.x-actual_.x),actual_.y+f*(v.y-actual_.y),actual_.yaw+f*(v.yaw-actual_.yaw)};
            if(blocked(intermediate,points)) {safe=false;break;}
          }
          if (safe) {scale=candidate;break;}
        }
        reason=scale==1 ? "clear" : scale>0 ? "braking_slowdown" : "swept_collision";
        result.linear.x=requested.x*scale;result.linear.y=requested.y*scale;result.angular.z=requested.yaw*scale;
      }
    }
    publish(result,reason,scale,start);
  }
  void publish(const geometry_msgs::msg::Twist &result,const std::string &reason,double scale,Clock::time_point start) {
    output_->publish(result);
    const double milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
    std::ostringstream stream;
    stream << "{\"reason\":\"" << reason << "\",\"scale\":" << scale << ",\"compute_ms\":" << milliseconds
           << ",\"remembered_points\":" << scan_points_.size()
           << ",\"prior_examined\":" << prior_examined_ << ",\"candidate_points\":" << candidate_points_
           << ",\"prior_association_hits\":" << association_cache_.hits()
           << ",\"prior_association_misses\":" << association_cache_.misses()
           << ",\"query_radius_m\":" << query_radius_ << ",\"tf_ready_scans\":" << tf_ready_scans_ << "}";
    std_msgs::msg::String status;status.data=stream.str();status_->publish(status);
  }
  Polygon polygon_; Pose pose_,memory_anchor_; Velocity actual_;geometry_msgs::msg::Twist command_;
  std::vector<Point> scan_points_,prior_points_;
  static constexpr double prior_index_bin_=.10,observed_index_bin_=.05;
  SpatialPointIndex prior_index_{prior_index_bin_},observed_index_{observed_index_bin_};
  PriorAssociationCache association_cache_;
  uint64_t prior_epoch_{0},observation_epoch_{0};
  size_t prior_examined_{0},candidate_points_{0},tf_ready_scans_{0};double query_radius_{0};
  std::deque<std::pair<int64_t,Clock::time_point>> scan_receipts_;
  double margin_,range_error_,reaction_,decel_,angular_decel_,scan_timeout_,command_timeout_,odometry_timeout_,localization_tf_timeout_,memory_radius_;
  Clock::time_point last_command_,last_scan_,last_odom_;
  ScanRecoveryOrder scan_order_;
  bool have_command_{false},have_scan_{false},have_odom_{false},have_prior_{false},have_anchor_{false},scan_invalid_{false};
  bool prior_invalid_{false};
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr memory_output_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prior_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VelocityGuard>());rclcpp::shutdown();return 0;
}
