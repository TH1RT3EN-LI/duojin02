#pragma once
#include "duojin01_navigation/gpu_sampling.hpp"
#include <cfloat>
#ifdef __CUDACC__
#define DUOJIN_HD __host__ __device__
#else
#define DUOJIN_HD
#endif
namespace duojin01_navigation {
struct SamplingView {
  unsigned int width,height,crop_x,crop_y;
  double resolution,origin_x,origin_y,radius;
  const unsigned char *cells;
  const unsigned int *rows,*columns;
  const Point *polygon;size_t vertices;
  const Pose *path;size_t path_size;
  Pose start,goal;Velocity measured;OmniVelocityBounds bounds;
  SamplingSettings settings;
};
DUOJIN_HD inline double smMin(double a,double b){return a<b ? a:b;}
DUOJIN_HD inline double smMax(double a,double b){return a>b ? a:b;}
DUOJIN_HD inline double smClamp(double a,double low,double high){return smMin(high,smMax(low,a));}
DUOJIN_HD inline double smSquare(double a){return a*a;}
DUOJIN_HD inline double smAngle(double a){return atan2(sin(a),cos(a));}
DUOJIN_HD inline Velocity sampleControl(const SamplingView &v,const Velocity *nominal,
    const Velocity *noise,size_t trajectory,size_t step,Velocity previous) {
  const auto &s=v.settings;
  const double position=s.time_steps>1 ? static_cast<double>(step)*(s.support_points-1)/(s.time_steps-1):0;
  const size_t lo=static_cast<size_t>(position),hi=lo+1<s.support_points ? lo+1:lo;
  const double f=position-lo;
  const auto &a=noise[trajectory*s.support_points+lo],&b=noise[trajectory*s.support_points+hi];
  const auto &n=nominal[step];
  Velocity u{n.x+a.x+(b.x-a.x)*f,n.y+a.y+(b.y-a.y)*f,n.yaw+a.yaw+(b.yaw-a.yaw)*f};
  u.x=smClamp(u.x,v.bounds.vx_min,v.bounds.vx_max);u.y=smClamp(u.y,-v.bounds.vy_max,v.bounds.vy_max);
  u.yaw=smClamp(u.yaw,-v.bounds.wz_max,v.bounds.wz_max);
  const double dx=u.x-previous.x,dy=u.y-previous.y;
  const double delta=sqrt(dx*dx+dy*dy),limit=s.acceleration*s.dt;
  if(delta>limit && delta>0){u.x=previous.x+dx*limit/delta;u.y=previous.y+dy*limit/delta;}
  u.yaw=smClamp(u.yaw,previous.yaw-s.angular_acceleration*s.dt,previous.yaw+s.angular_acceleration*s.dt);
  // An out-of-bound measured speed may require immediate command clamping.
  u.x=smClamp(u.x,v.bounds.vx_min,v.bounds.vx_max);u.y=smClamp(u.y,-v.bounds.vy_max,v.bounds.vy_max);
  u.yaw=smClamp(u.yaw,-v.bounds.wz_max,v.bounds.wz_max);return u;
}
DUOJIN_HD inline Pose sampleAdvance(Pose p,Velocity u,double dt) {
  const double mid=p.yaw+u.yaw*dt/2,c=cos(mid),s=sin(mid);
  p.x+=(c*u.x-s*u.y)*dt;p.y+=(s*u.x+c*u.y)*dt;p.yaw+=u.yaw*dt;return p;
}
DUOJIN_HD inline double sampleSegmentDistance(double x,double y,const Point &a,const Point &b) {
  const double dx=b.x-a.x,dy=b.y-a.y;
  const double t=smClamp(((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy),0.,1.);
  return sqrt(smSquare(x-a.x-t*dx)+smSquare(y-a.y-t*dy));
}
DUOJIN_HD inline double sampleSegmentSquared(double x,double y,const Point &a,const Point &b) {
  const double dx=b.x-a.x,dy=b.y-a.y;
  const double t=smClamp(((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy),0.,1.);
  return smSquare(x-a.x-t*dx)+smSquare(y-a.y-t*dy);
}
DUOJIN_HD inline bool rasterCollision(const SamplingView &v,Pose pose,double margin) {
  const double c=cos(pose.yaw),s=sin(pose.yaw);
  double lx=DBL_MAX,ly=DBL_MAX,ux=-DBL_MAX,uy=-DBL_MAX;
  double local_lx=DBL_MAX,local_ly=DBL_MAX,local_ux=-DBL_MAX,local_uy=-DBL_MAX;
  for(size_t i=0;i<v.vertices;++i) {
    const auto &a=v.polygon[i];const double x=pose.x+c*a.x-s*a.y,y=pose.y+s*a.x+c*a.y;
    lx=smMin(lx,x);ly=smMin(ly,y);ux=smMax(ux,x);uy=smMax(uy,y);
    local_lx=smMin(local_lx,a.x);local_ly=smMin(local_ly,a.y);local_ux=smMax(local_ux,a.x);local_uy=smMax(local_uy,a.y);
  }
  const double padding=margin+v.resolution*1.4142135623730951/2+1e-9;
  lx-=padding;ly-=padding;ux+=padding;uy+=padding;
  const double map_lx=v.origin_x+v.crop_x*v.resolution,map_ly=v.origin_y+v.crop_y*v.resolution;
  if(lx<map_lx || ly<map_ly || ux>=map_lx+v.width*v.resolution || uy>=map_ly+v.height*v.resolution)return true;
  // Conservative row/column range followed by the exact filled polygon test.
  const int min_x=static_cast<int>(smMax(0,static_cast<int>(floor((lx-v.origin_x)/v.resolution))-static_cast<int>(v.crop_x)));
  const int max_x=static_cast<int>(smMin(v.width-1,static_cast<int>(floor((ux-v.origin_x)/v.resolution))-static_cast<int>(v.crop_x)));
  const int min_y=static_cast<int>(smMax(0,static_cast<int>(floor((ly-v.origin_y)/v.resolution))-static_cast<int>(v.crop_y)));
  const int max_y=static_cast<int>(smMin(v.height-1,static_cast<int>(floor((uy-v.origin_y)/v.resolution))-static_cast<int>(v.crop_y)));
  for(int row=min_y;row<=max_y;++row) {
    unsigned int low=v.rows[row],high=v.rows[row+1];
    while(low<high){const auto mid=low+(high-low)/2;if(v.columns[mid]<static_cast<unsigned int>(min_x))low=mid+1;else high=mid;}
    for(unsigned int j=low;j<v.rows[row+1] && v.columns[j]<=static_cast<unsigned int>(max_x);++j) {
      const double wx=v.origin_x+(v.crop_x+v.columns[j]+.5)*v.resolution;
      const double wy=v.origin_y+(v.crop_y+row+.5)*v.resolution;
      const double dx=wx-pose.x,dy=wy-pose.y,x=c*dx+s*dy,y=-s*dx+c*dy;
      if(x<local_lx-padding || x>local_ux+padding || y<local_ly-padding || y>local_uy+padding)continue;
      bool inside=true;
      for(size_t i=0;i<v.vertices;++i) {
        const auto &a=v.polygon[i],&b=v.polygon[(i+1)%v.vertices];
        if((b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x)<-1e-10)inside=false;
        if(sampleSegmentSquared(x,y,a,b)<=padding*padding)return true;
      }
      if(inside)return true;
    }
  }
  return false;
}
DUOJIN_HD inline bool rasterSweep(const SamplingView &v,Pose a,Pose b) {
  const double dx=b.x-a.x,dy=b.y-a.y,da=b.yaw-a.yaw;
  const int steps=static_cast<int>(ceil((sqrt(dx*dx+dy*dy)+v.radius*fabs(da))/.004));
  const int count=steps>0 ? steps:1;
  // 2mm covers sample spacing; curvature bounds the midpoint-model chord.
  const double curve=sqrt(smSquare(smMax(v.bounds.vx_max,-v.bounds.vx_min))+smSquare(v.bounds.vy_max))*v.bounds.wz_max*v.settings.dt*v.settings.dt/8;
  for(int i=0;i<=count;++i) {
    const double f=static_cast<double>(i)/count;
    if(rasterCollision(v,{a.x+dx*f,a.y+dy*f,a.yaw+da*f},v.settings.margin+.002+curve))return true;
  }
  return false;
}
DUOJIN_HD inline double sampleCost(const SamplingView &v,Pose previous,Pose pose,
    Velocity control,Velocity nominal,size_t step) {
  if(rasterSweep(v,previous,pose))return 1e10;
  double path_squared=DBL_MAX;
  for(size_t i=1;i<v.path_size;++i) {
    const Point a{v.path[i-1].x,v.path[i-1].y,0},b{v.path[i].x,v.path[i].y,0};
    if(smSquare(b.x-a.x)+smSquare(b.y-a.y)<1e-20)continue;
    path_squared=smMin(path_squared,sampleSegmentSquared(pose.x,pose.y,a,b));
  }
  if(path_squared==DBL_MAX)path_squared=smSquare(pose.x-v.goal.x)+smSquare(pose.y-v.goal.y);
  const int x=static_cast<int>((pose.x-v.origin_x)/v.resolution)-static_cast<int>(v.crop_x);
  const int y=static_cast<int>((pose.y-v.origin_y)/v.resolution)-static_cast<int>(v.crop_y);
  const double cost=x>=0 && y>=0 && x<static_cast<int>(v.width) && y<static_cast<int>(v.height) ? v.cells[y*v.width+x]/252.:1.;
  const auto &s=v.settings;
  double result=s.path_weight*path_squared+s.cost_weight*cost*cost;
  result+=s.gamma*(nominal.x*(control.x-nominal.x)/(s.noise.x*s.noise.x)+
    nominal.y*(control.y-nominal.y)/(s.noise.y*s.noise.y)+nominal.yaw*(control.yaw-nominal.yaw)/(s.noise.yaw*s.noise.yaw));
  if(step+1==s.time_steps)result+=s.goal_weight*(smSquare(pose.x-v.goal.x)+smSquare(pose.y-v.goal.y))+
    s.yaw_weight*smSquare(smAngle(pose.yaw-v.goal.yaw));
  return result;
}
}
#undef DUOJIN_HD
