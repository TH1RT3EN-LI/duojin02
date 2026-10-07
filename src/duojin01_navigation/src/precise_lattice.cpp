#include "duojin01_navigation/sweep.hpp"
#include "duojin01_navigation/path_cleanup.hpp"
#include <nav2_smac_planner/smac_planner_lattice.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace duojin01_navigation {
// Lattice headings are discrete. Preserve the user's exact terminal pose only
// after checking the filled footprint through every interpolated path segment.
class PreciseLattice : public nav2_smac_planner::SmacPlannerLattice {
 public:
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                               const geometry_msgs::msg::PoseStamped &goal) override {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    Polygon footprint;
    for(const auto &p:_costmap_ros->getRobotFootprint()) footprint.push_back({p.x,p.y});
    const double resolution=_costmap->getResolution();
    double radius=0;
    for(const auto &p:footprint) radius=std::max(radius,std::hypot(p.x,p.y));
    // Index each current costmap snapshot once. Narrow long paths otherwise
    // rescan thousands of empty cells for every 4 mm validation sample.
    std::vector<std::vector<Point>> rows;
    auto index_map=[&]() {
      rows.assign(_costmap->getSizeInCellsY(),{});
      for(unsigned int y=0;y<_costmap->getSizeInCellsY();++y)
        for(unsigned int x=0;x<_costmap->getSizeInCellsX();++x) {
          const auto cost=_costmap->getCost(x,y);
          if(cost==nav2_costmap_2d::LETHAL_OBSTACLE || (cost==nav2_costmap_2d::NO_INFORMATION && !_allow_unknown)) {
            double wx=0,wy=0;_costmap->mapToWorld(x,y,wx,wy);
            rows[y].push_back({wx,wy,resolution*std::sqrt(2.0)/2});
          }
        }
    };
    index_map();
    auto valid=[&](const Pose &pose) {
      std::vector<Point> obstacles;
      const double extent=radius+resolution;
      int lx=0,ly=0,ux=0,uy=0;
      _costmap->worldToMapEnforceBounds(pose.x-extent,pose.y-extent,lx,ly);
      _costmap->worldToMapEnforceBounds(pose.x+extent,pose.y+extent,ux,uy);
      if(pose.x-extent<_costmap->getOriginX() || pose.y-extent<_costmap->getOriginY() ||
         pose.x+extent>_costmap->getOriginX()+_costmap->getSizeInMetersX() ||
         pose.y+extent>_costmap->getOriginY()+_costmap->getSizeInMetersY()) return false;
      double low=0,high=0,unused=0;_costmap->mapToWorld(lx,ly,low,unused);_costmap->mapToWorld(ux,ly,high,unused);
      for(int y=ly;y<=uy;++y) {
        const auto &row=rows[y];
        auto begin=std::lower_bound(row.begin(),row.end(),low,[](const Point &p,double x){return p.x<x;});
        for(auto it=begin;it!=row.end() && it->x<=high;++it) obstacles.push_back(*it);
      }
      return !collision(pose,footprint,obstacles,.002);
    };
    auto segment_valid=[&](const geometry_msgs::msg::Pose &a,const geometry_msgs::msg::Pose &b) {
      const double yaw=tf2::getYaw(a.orientation);
      const double dyaw=std::remainder(tf2::getYaw(b.orientation)-yaw,2*M_PI);
      const double dx=b.position.x-a.position.x,dy=b.position.y-a.position.y;
      const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(dyaw))/.004)));
      for(int j=0;j<=steps;++j) {
        double f=static_cast<double>(j)/steps;
        if(!valid({a.position.x+f*dx,a.position.y+f*dy,yaw+f*dyaw})) return false;
      }
      return true;
    };
    auto path_valid=[&](const nav_msgs::msg::Path &path) {
      for(size_t i=1;i<path.poses.size();++i)
        if(!segment_valid(path.poses[i-1].pose,path.poses[i].pose)) return false;
      return !path.poses.empty();
    };
    // An Omni base can translate along a checked SE2 chord. Avoid quantized
    // grid starts and large A* searches when an exact direct connection is
    // available. Bound this shortcut's work; longer / blocked routes use the
    // complete original lattice and receive the same filled validation.
    const double dx=goal.pose.position.x-start.pose.position.x,dy=goal.pose.position.y-start.pose.position.y;
    const double yaw=tf2::getYaw(start.pose.orientation),dyaw=std::remainder(tf2::getYaw(goal.pose.orientation)-yaw,2*M_PI);
    const double travel=std::hypot(dx,dy)+radius*std::abs(dyaw);
    if(!std::isfinite(travel)) throw nav2_core::PlannerException("Invalid planar start or goal");
    if(travel<=6.0 && segment_valid(start.pose,goal.pose)) {
      nav_msgs::msg::Path direct;direct.header.frame_id=_global_frame;direct.header.stamp=_clock->now();
      const int steps=std::max(1,static_cast<int>(std::ceil(travel/.01)));
      for(int i=0;i<=steps;++i) {
        const double f=static_cast<double>(i)/steps;
        auto point=start;point.header=direct.header;
        point.pose.position.x+=f*dx;point.pose.position.y+=f*dy;
        point.pose.orientation.x=point.pose.orientation.y=0;
        point.pose.orientation.z=std::sin((yaw+f*dyaw)/2);point.pose.orientation.w=std::cos((yaw+f*dyaw)/2);
        direct.poses.push_back(point);
      }
      direct.poses.back()=goal;direct.poses.back().header=direct.header;
      return direct;
    }
    if(!cached_.poses.empty() && cached_.header.frame_id==_global_frame &&
       std::hypot(goal.pose.position.x-cached_.poses.back().pose.position.x,goal.pose.position.y-cached_.poses.back().pose.position.y)<.001 &&
       std::abs(std::remainder(tf2::getYaw(goal.pose.orientation)-tf2::getYaw(cached_.poses.back().pose.orientation),2*M_PI))<.001) {
      size_t nearest=0;double distance=1e100;
      for(size_t i=0;i<cached_.poses.size();++i) {
        const auto &p=cached_.poses[i].pose.position;
        const double d=std::hypot(p.x-start.pose.position.x,p.y-start.pose.position.y);
        if(d<distance) {nearest=i;distance=d;}
      }
      if(distance<.20) {
        auto remaining=cached_;remaining.header.stamp=_clock->now();
        remaining.poses.erase(remaining.poses.begin(),remaining.poses.begin()+nearest);
        remaining.poses.insert(remaining.poses.begin(),start);
        // Never trust a cached path after the map changes. Check the complete
        // remaining path, including the new start connector, every replan.
        if(path_valid(remaining)) {cached_=remaining;return remaining;}
      }
    }
    lock.unlock();
    auto path=SmacPlannerLattice::createPlan(start,goal);
    if(path.poses.empty()) return path;
    lock.lock();
    index_map(); // The map can change while the native planner held its lock.
    path.poses.insert(path.poses.begin(),start);
    path.poses.push_back(goal);
    std::vector<Pose> states;
    for(const auto &p:path.poses) states.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
    auto clean=path;clean.poses.clear();
    for(const auto i:loopFreeIndices(states)) clean.poses.push_back(path.poses[i]);
    path=clean;
    // Use constant-heading Omni travel only when the entire route and exact
    // final rotation fit. This is derived from the native plan, never waypoints
    // from a fixture, and removes needless heading changes in narrow bends.
    auto fixed=path;
    for(auto &p:fixed.poses) p.pose.orientation=start.pose.orientation;
    fixed.poses.push_back(goal);
    if(path_valid(fixed)) path=fixed;
    else if(!path_valid(path))
      throw nav2_core::PlannerException("Filled footprint / exact start or terminal pose intersects an obstacle or unknown cell");
    path.poses.back().header=path.header;
    cached_=path;
    return path;
  }
 private:
  nav_msgs::msg::Path cached_;
};
}
PLUGINLIB_EXPORT_CLASS(duojin01_navigation::PreciseLattice,nav2_core::GlobalPlanner)
