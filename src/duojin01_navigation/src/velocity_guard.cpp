#include "duojin01_navigation/sweep.hpp"
#include <chrono>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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

using namespace duojin01_navigation;
using Clock = std::chrono::steady_clock;
class VelocityGuard : public rclcpp::Node {
 public:
  VelocityGuard():Node("velocity_guard"),buffer_(get_clock()),listener_(buffer_) {
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
    memory_radius_=declare_parameter("obstacle_memory_radius_m",2.5);
    if (!std::isfinite(margin_+range_error_+decel_+angular_decel_+reaction_+scan_timeout_+command_timeout_+memory_radius_) ||
        margin_<0 || range_error_<0 || decel_<=0 || angular_decel_<=0 || reaction_<0 ||
        scan_timeout_<=0 || command_timeout_<=0 || memory_radius_<=0) throw std::runtime_error("Invalid guard limits");
    auto qos=rclcpp::QoS(1);
    output_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_guarded",qos);
    status_=create_publisher<std_msgs::msg::String>("/navigation/guard_status",qos);
    memory_output_=create_publisher<sensor_msgs::msg::PointCloud2>("/navigation/obstacle_memory",rclcpp::SensorDataQoS());
    command_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_requested",qos,
      [this](geometry_msgs::msg::Twist::SharedPtr m){command_=*m;last_command_=Clock::now();have_command_=true;});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m){
        if(m->header.frame_id!="odom" || m->child_frame_id!="base_footprint" ||
            !std::isfinite(m->pose.pose.position.x+m->pose.pose.position.y+
            m->twist.twist.linear.x+m->twist.twist.linear.y+m->twist.twist.angular.z) ||
            std::abs((now()-rclcpp::Time(m->header.stamp)).seconds())>.20) {have_odom_=false;return;}
        pose_={m->pose.pose.position.x,m->pose.pose.position.y,tf2::getYaw(m->pose.pose.orientation)};
        actual_={m->twist.twist.linear.x,m->twist.twist.linear.y,m->twist.twist.angular.z};
        if(!std::isfinite(pose_.yaw)) {have_odom_=false;return;}
        last_odom_=Clock::now();have_odom_=true;
      });
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){scan(m);});
    costmap_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){costmap(m);});
    prior_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){
        if(m->header.frame_id!="map" || !m->info.width || !m->info.height ||
           !std::isfinite(m->info.resolution) || m->info.resolution<=0 ||
           m->data.size()!=static_cast<size_t>(m->info.width)*m->info.height) return;
        Pose origin{m->info.origin.position.x,m->info.origin.position.y,tf2::getYaw(m->info.origin.orientation)};
        prior_points_.clear();
        for(size_t i=0;i<m->data.size();++i) if(m->data[i]>=65) {
          Point p{(i%m->info.width+.5)*m->info.resolution,(i/m->info.width+.5)*m->info.resolution,
                  std::sqrt(2.0)*m->info.resolution/2};
          prior_points_.push_back(transform(p,origin));
        }
        have_prior_=true;
      });
    localization_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m){
        const auto &c=m->pose.covariance;
        const double xy=(c[1]+c[6])/2;
        if(!std::isfinite(c[0]+c[7]+c[35]+xy) || c[0]<0 || c[7]<0 || c[35]<0 || c[0]*c[7]-xy*xy < -1e-9) {
          localization_margin_=10;return;
        }
        const double eigenvalue=(c[0]+c[7]+std::hypot(c[0]-c[7],2*xy))/2;
        double radius=0;for(const auto &p:polygon_) radius=std::max(radius,std::hypot(p.x,p.y));
        localization_margin_=2*std::sqrt(std::max(0.0,eigenvalue))+2*radius*std::sqrt(c[35]);
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Filled footprint braking sweep: margin %.3f m, reaction %.2f s; all commands are gated",margin_,reaction_);
  }
 private:
  static double age(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
  Pose framePose(const std::string &frame,const builtin_interfaces::msg::Time &stamp) {
    if (frame=="odom") return {};
    auto tf=buffer_.lookupTransform("odom",frame,rclcpp::Time(stamp),rclcpp::Duration::from_seconds(.02));
    return {tf.transform.translation.x,tf.transform.translation.y,tf2::getYaw(tf.transform.rotation)};
  }
  void scan(const sensor_msgs::msg::LaserScan::SharedPtr &m) {
    try {
      if (std::abs((now()-rclcpp::Time(m->header.stamp)).seconds())>scan_timeout_) return;
      auto transform_pose=framePose(m->header.frame_id,m->header.stamp);
      if(m->ranges.empty() || !std::isfinite(m->angle_increment) || m->angle_increment==0) return;
      // Correct remembered surfaces when localization compensates odometry
      // drift. Fresh observations remain in sensor-time odom coordinates.
      // Without this anchoring, an old wall can drift into the footprint.
      try {
        if(!buffer_.canTransform("odom","map",rclcpp::Time(m->header.stamp),rclcpp::Duration::from_seconds(0))) {
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
      sensor_msgs::msg::PointCloud2 cloud;cloud.header=m->header;
      sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(scan_points_.size());
      sensor_msgs::PointCloud2Iterator<float> cx(cloud,"x"),cy(cloud,"y"),cz(cloud,"z");
      for(const auto &p:scan_points_) {
        const double dx=p.x-transform_pose.x,dy=p.y-transform_pose.y;
        *cx=static_cast<float>(c*dx+s*dy);*cy=static_cast<float>(-s*dx+c*dy);*cz=0;
        ++cx;++cy;++cz;
      }
      memory_output_->publish(cloud);
      last_scan_=Clock::now();have_scan_=true;
    } catch (const tf2::TransformException &) { /* Fail closed until fresh sensor TF is available. */ }
  }
  void costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr &m) {
    if(!m->info.width || !m->info.height || !std::isfinite(m->info.resolution) || m->info.resolution<=0 ||
       m->data.size()!=static_cast<size_t>(m->info.width)*m->info.height ||
       std::abs((now()-rclcpp::Time(m->header.stamp)).seconds())>.60) return;
    try {
      auto tf=framePose(m->header.frame_id,m->header.stamp);
      Pose origin{m->info.origin.position.x,m->info.origin.position.y,tf2::getYaw(m->info.origin.orientation)};
      std::vector<Point> points;
      for (size_t i=0;i<m->data.size();++i) {
        if (m->data[i]!=100) continue; // Filled LETHAL cells; inflation remains a soft preference.
        Point p{(i%m->info.width+.5)*m->info.resolution,(i/m->info.width+.5)*m->info.resolution,
                std::sqrt(2.0)*m->info.resolution/2};
        points.push_back(transform(transform(p,origin),tf));
      }
      map_points_=std::move(points);last_map_=Clock::now();have_map_=true;
    } catch (const tf2::TransformException &) { }
  }
  bool blocked(Velocity v,const std::vector<Point> &points,double reserve=0) {
    return brakingSweep(pose_,v,polygon_,points,margin_+reserve,reaction_,decel_,angular_decel_);
  }
  std::vector<Point> obstacles() {
    // Live scan obstacles are measured in odom: AMCL covariance is not an
    // additional independent error on those observations. A map prior needs
    // that margin only where its surface has not been directly observed.
    std::vector<Point> points=scan_points_;
    if(!have_prior_) {points.insert(points.end(),map_points_.begin(),map_points_.end());return points;}
    auto tf=buffer_.lookupTransform("odom","map",tf2::TimePointZero);
    if(std::abs((now()-rclcpp::Time(tf.header.stamp)).seconds())>.50)
      throw tf2::TransformException("Stale localization transform");
    Pose map_pose{tf.transform.translation.x,tf.transform.translation.y,tf2::getYaw(tf.transform.rotation)};
    std::unordered_multimap<int64_t,const Point *> observed;
    const auto key=[](int x,int y){return static_cast<int64_t>(x)*10000000LL+y;};
    constexpr double bin=.05;
    for(const auto &p:scan_points_) observed.emplace(key(static_cast<int>(std::floor(p.x/bin)),static_cast<int>(std::floor(p.y/bin))),&p);
    for(const auto &prior:prior_points_) {
      auto p=transform(prior,map_pose);
      if(std::hypot(p.x-pose_.x,p.y-pose_.y)>memory_radius_) continue;
      bool matched=false;
      const double association=p.radius+std::min(.10,localization_margin_)+range_error_+.015;
      const int x=static_cast<int>(std::floor(p.x/bin)),y=static_cast<int>(std::floor(p.y/bin));
      const int extent=static_cast<int>(std::ceil(association/bin));
      if(localization_margin_<.15) for(int dx=-extent;dx<=extent&&!matched;++dx) for(int dy=-extent;dy<=extent&&!matched;++dy) {
        auto range=observed.equal_range(key(x+dx,y+dy));
        for(auto it=range.first;it!=range.second;++it) if(std::hypot(p.x-it->second->x,p.y-it->second->y)<=association) {matched=true;break;}
      }
      if(!matched) {p.radius+=localization_margin_;points.push_back(p);}
    }
    return points;
  }
  void tick() {
    const auto start=Clock::now();
    geometry_msgs::msg::Twist result;
    std::string reason="clear";
    double scale=0;
    if (!have_command_ || age(last_command_)>command_timeout_) reason="command_stale";
    else if (!have_scan_ || age(last_scan_)>scan_timeout_) reason="scan_or_tf_stale";
    else if (!have_odom_ || age(last_odom_)>.20) reason="odometry_stale";
    else if (have_map_ && age(last_map_)>.60) reason="costmap_stale";
    else {
      std::vector<Point> points;
      try {points=obstacles();}
      catch(const tf2::TransformException &) {reason="localization_tf_stale";publish(result,reason,scale,start);return;}
      Velocity requested{command_.linear.x,command_.linear.y,command_.angular.z};
      if (!std::isfinite(requested.x)||!std::isfinite(requested.y)||!std::isfinite(requested.yaw)) reason="invalid_command";
      else if (blocked(actual_,points)) {
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
          // Inside the extra reserve, retain the complete core safety envelope
          // and allow only crawl speed. Otherwise even a safe lateral escape
          // would be rejected merely because its initial pose lacks reserve.
          if(safe && blocked(v,points,.005) && (std::hypot(v.x,v.y)>.04 || std::abs(v.yaw)>.08)) safe=false;
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
           << ",\"localization_margin_m\":" << localization_margin_ << ",\"remembered_points\":" << scan_points_.size() << "}";
    std_msgs::msg::String status;status.data=stream.str();status_->publish(status);
  }
  Polygon polygon_; Pose pose_,memory_anchor_; Velocity actual_;geometry_msgs::msg::Twist command_;
  std::vector<Point> scan_points_,map_points_,prior_points_;
  double margin_,range_error_,reaction_,decel_,angular_decel_,scan_timeout_,command_timeout_,memory_radius_,localization_margin_{0};
  Clock::time_point last_command_,last_scan_,last_odom_,last_map_;
  bool have_command_{false},have_scan_{false},have_odom_{false},have_map_{false},have_prior_{false},have_anchor_{false};
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr memory_output_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_,prior_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VelocityGuard>());rclcpp::shutdown();return 0;
}
