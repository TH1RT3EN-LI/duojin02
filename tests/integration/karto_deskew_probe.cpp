// Production Karto integration: geometry must survive cache invalidation and binary reload.
#include <cmath>
#include <iostream>
#include <sstream>
#include <karto_sdk/Karto.h>
using namespace karto;
void check(bool value, const char * why) {if (!value) {throw std::runtime_error(why);}}
void near(double a, double b, const char * why) {check(std::abs(a-b)<1e-10,why);}
struct CaptureGrid : OccupancyGrid
{
  using OccupancyGrid::AddScan;
  PointVectorDouble from, to;
  CaptureGrid() : OccupancyGrid(100,100,Vector2<double>(-5,-5),0.1) {}
  kt_bool RayTrace(const Vector2<double> & a, const Vector2<double> & b,
    kt_bool, kt_bool=false) override {from.push_back(a);to.push_back(b);return true;}
};
int main()
{
  auto * laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("deskew_probe"));
  laser->SetMinimumRange(0.1);laser->SetMaximumRange(10);laser->SetRangeThreshold(4);
  laser->SetMinimumAngle(0);laser->SetMaximumAngle(1);laser->SetAngularResolution(1);
  laser->SetOffsetPose(Pose2(0.305,0,0));
  SensorManager::GetInstance()->RegisterSensor(laser);
  LocalizedRangeScan scan(laser->GetName(),{2,6},LocalizedRangeScan::LegacyRawTag{});
  scan.SetRawSensorLimits(0.1,10);
  scan.SetScanTimeProvenance(1000000000LL,1025000000LL,"test-v1","analytic test");
  scan.SetDeskewGeometry({{2.305,0},{0.5,6}},{{0.305,0},{0.5,0}},
    {1000000000LL,1050000000LL},{0,1},{1,1});
  scan.SetCorrectedPose(Pose2(1,2,0));scan.SetOdometricPose(Pose2(1,2,0));
  near(scan.GetPointReadings(false)[0].GetX(),3.305,"production endpoint");
  check(scan.GetPointReadings(true).size()==1,"original range threshold filtering");
  near(scan.GetRayOrigin(1).GetX(),1.5,"second moving ray origin");
  // Dirty updates after optimization must not rebuild uniform-angle points.
  scan.SetCorrectedPose(Pose2(2,3,0));
  near(scan.GetPointReadings(false)[1].GetX(),2.5,"dirty geometry retained");
  near(scan.GetPointReadings(false)[1].GetY(),9,"dirty endpoint retained");
  CaptureGrid grid;grid.AddScan(&scan);
  check(grid.from.size()==2,"all production rays");
  near(grid.from[1].GetX(),2.5,"raytrace origin");
  near(grid.to[1].GetX(),2.5,"threshold clipping x");
  near(grid.to[1].GetY(),7,"threshold clipping starts at beam origin");
  const auto sensor_points=laser->GetPointReadings(&scan,nullptr,false);
  near(sensor_points[1].GetX(),2.5,"sensor point API preserves deskew");
  near(sensor_points[1].GetY(),7,"sensor point API clips from real origin");
  std::stringstream bytes;
  {boost::archive::binary_oarchive archive(bytes);archive << scan;}
  LocalizedRangeScan restored;
  {boost::archive::binary_iarchive archive(bytes);archive >> restored;}
  check(restored.HasDeskewGeometry(),"geometry binary reload");
  restored.SetCorrectedPose(Pose2(3,4,0));
  near(restored.GetPointReadings(false)[1].GetX(),3.5,"binary dirty point x");
  near(restored.GetPointReadings(false)[1].GetY(),10,"binary dirty point y");
  near(restored.GetRayOrigin(1).GetX(),3.5,"binary moving origin");
  check(restored.GetBeamStampNs()[1]==1050000000LL,"binary original beam stamp");
  check(restored.GetRawScanStampNs()==1000000000LL,"binary raw source stamp");
  check(restored.GetTimeMappingId()=="test-v1","binary mapping provenance");
  std::cout<<"Karto production deskew cache, thresholds, moving origins, ray clipping and versioned binary roundtrip passed\n";
  SensorManager::GetInstance()->UnregisterSensor(laser);delete laser;
}
