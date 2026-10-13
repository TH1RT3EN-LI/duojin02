#include "duojin01_navigation/path_target.hpp"
#include "path_target_reference.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>

using namespace duojin01_navigation;
static bool equalBits(double a,double b) {return std::memcmp(&a,&b,sizeof(double))==0;}
static void equalPose(const geometry_msgs::msg::PoseStamped &a,const geometry_msgs::msg::PoseStamped &b) {
  assert(a.header==b.header);
  assert(equalBits(a.pose.position.x,b.pose.position.x));assert(equalBits(a.pose.position.y,b.pose.position.y));
  assert(equalBits(a.pose.position.z,b.pose.position.z));assert(equalBits(a.pose.orientation.x,b.pose.orientation.x));
  assert(equalBits(a.pose.orientation.y,b.pose.orientation.y));assert(equalBits(a.pose.orientation.z,b.pose.orientation.z));
  assert(equalBits(a.pose.orientation.w,b.pose.orientation.w));
}
static geometry_msgs::msg::Quaternion rotation(double r,double p,double y,double scale=1) {
  tf2::Quaternion q;q.setRPY(r,p,y);auto result=tf2::toMsg(q);
  result.x*=scale;result.y*=scale;result.z*=scale;result.w*=scale;return result;
}
int main() {
  std::mt19937_64 rng(20261010);std::uniform_real_distribution<double> unit(-1,1);
  size_t comparisons=0,accepted=0,rejected=0;
  auto compare=[&](const nav_msgs::msg::Path &path,const PathTarget &candidate,
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::TransformStamped &tf,
      const geometry_msgs::msg::Twist &speed) {
    geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="sentinel";
    a.pose.position.x=b.pose.position.x=123;
    const bool old=referencePathTarget(path,pose,tf,speed,a),now=candidate.select(pose,tf,speed,b);
    assert(old==now);equalPose(a,b);++comparisons;old ? ++accepted : ++rejected;
  };
  PathTarget candidate;nav_msgs::msg::Path path;
  for(int family=0;family<60;++family) {
    path.header.frame_id=family%2 ? "map" : "changed_map";path.poses.clear();
    const int count=family%10==0 ? 0 : (family%10==1 ? 1 : 20+family*9);
    for(int i=0;i<count;++i) {
      geometry_msgs::msg::PoseStamped p;p.header.frame_id="ignored_individual_frame";p.header.stamp.sec=123;
      p.pose.position.x=std::sin(i*.07)*(1+.02*family);p.pose.position.y=std::sin(i*.14)*.6;
      p.pose.position.z=family%3 ? 0 : .04*std::sin(i*.02);
      p.pose.orientation=rotation(family%3 ? 0 : .04*unit(rng),family%4 ? 0 : .03*unit(rng),i*.07);
      if(i%17==0 && i>0) p=path.poses.back();
      path.poses.push_back(p);
    }
    candidate.setPath(path);assert(candidate.frameId()==path.header.frame_id);assert(candidate.empty()==path.poses.empty());
    for(int trial=0;trial<80;++trial) {
      geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="odom";tf.child_frame_id=path.header.frame_id;
      tf.header.stamp.sec=trial+1;tf.header.stamp.nanosec=123456;
      tf.transform.translation.x=4*unit(rng);tf.transform.translation.y=4*unit(rng);tf.transform.translation.z=.1*unit(rng);
      tf.transform.rotation=rotation(family%3 ? 0 : .02*unit(rng),family%4 ? 0 : .03*unit(rng),3*unit(rng));
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";pose.header.stamp.sec=trial+2;
      pose.pose.orientation=rotation(0,0,3*unit(rng));
      if(!path.poses.empty()) {
        const auto &p=path.poses[(trial*19)%path.poses.size()];tf2::doTransform(p,pose,tf);
        pose.pose.position.x+=(trial%3==0 ? .35 : .05)*unit(rng);
        pose.pose.position.y+=(trial%3==0 ? .35 : .05)*unit(rng);
      }
      geometry_msgs::msg::Twist speed;speed.linear.x=unit(rng);speed.linear.y=unit(rng);
      compare(path,candidate,pose,tf,speed);
    }
  }
  // Strict .20 offset, .18/.35 lookahead and ties at repeated / crossing poses.
  geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="odom";tf.transform.rotation.w=1;
  geometry_msgs::msg::PoseStamped pose;pose.pose.orientation.w=1;geometry_msgs::msg::Twist speed;
  path.poses.clear();path.header.frame_id="map";
  for(double x:{0.,0.,.18,.18,.35,.6,0.}) {
    auto p=pose;p.pose.position.x=x;path.poses.push_back(p);
  }
  for(double scale:{1.,2.,.3}) {
    for(auto &p:path.poses)p.pose.orientation=rotation(0,0,.0,scale);
    candidate.setPath(path);
    for(double offset:{0.,-0.,std::nextafter(.20,0.),.20,std::nextafter(.20,1.),-.20})
      for(double vx:{0.,std::nextafter(.425,0.),.425,std::nextafter(.425,1.),-.6}) {
        pose.pose.position.y=offset;speed.linear.x=vx;compare(path,candidate,pose,tf,speed);
      }
  }
  path.poses.clear();candidate.setPath(path);compare(path,candidate,pose,tf,speed);
  std::cout<<"PATH_TARGET_EQUIVALENCE comparisons="<<comparisons<<" accepted="<<accepted<<" rejected="<<rejected
    <<" exact pose/header bits; changing TF, path replacement, empty/single/repeated/crossing, roll/pitch, thresholds passed\n";
}
