#include "duojin01_navigation/sweep.hpp"
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
    auto path=SmacPlannerLattice::createPlan(start,goal);
    if (path.poses.empty()) return path;
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    Polygon footprint;
    for(const auto &p:_costmap_ros->getRobotFootprint()) footprint.push_back({p.x,p.y});
    const double resolution=_costmap->getResolution();
    double radius=0;
    for(const auto &p:footprint) radius=std::max(radius,std::hypot(p.x,p.y));
    auto valid=[&](const Pose &pose) {
      std::vector<Point> obstacles;
      const double extent=radius+resolution;
      int lx=0,ly=0,ux=0,uy=0;
      _costmap->worldToMapEnforceBounds(pose.x-extent,pose.y-extent,lx,ly);
      _costmap->worldToMapEnforceBounds(pose.x+extent,pose.y+extent,ux,uy);
      if(pose.x-extent<_costmap->getOriginX() || pose.y-extent<_costmap->getOriginY() ||
         pose.x+extent>_costmap->getOriginX()+_costmap->getSizeInMetersX() ||
         pose.y+extent>_costmap->getOriginY()+_costmap->getSizeInMetersY()) return false;
      for(int y=ly;y<=uy;++y) for(int x=lx;x<=ux;++x) {
        auto cost=_costmap->getCost(x,y);
        if(cost==nav2_costmap_2d::LETHAL_OBSTACLE || (cost==nav2_costmap_2d::NO_INFORMATION && !_allow_unknown)) {
          double wx=0,wy=0;_costmap->mapToWorld(x,y,wx,wy);
          obstacles.push_back({wx,wy,resolution*std::sqrt(2.0)/2});
        }
      }
      return !collision(pose,footprint,obstacles,.002);
    };
    path.poses.push_back(goal);
    for(size_t i=1;i<path.poses.size();++i) {
      const auto &a=path.poses[i-1].pose,&b=path.poses[i].pose;
      const double yaw=tf2::getYaw(a.orientation);
      const double dyaw=std::remainder(tf2::getYaw(b.orientation)-yaw,2*M_PI);
      const double dx=b.position.x-a.position.x,dy=b.position.y-a.position.y;
      const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(dyaw))/.004)));
      for(int j=0;j<=steps;++j) {
        double f=static_cast<double>(j)/steps;
        if(!valid({a.position.x+f*dx,a.position.y+f*dy,yaw+f*dyaw}))
          throw nav2_core::PlannerException("Filled footprint / precise terminal pose intersects an obstacle or unknown cell");
      }
    }
    path.poses.back().header=path.header;
    return path;
  }
};
}
PLUGINLIB_EXPORT_CLASS(duojin01_navigation::PreciseLattice,nav2_core::GlobalPlanner)
