// Link this against instrumented production Karto.cpp and Mapper.cpp, not a witness formula.
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <type_traits>
#include <karto_sdk/Mapper.h>
#include "scan_geometry_fixture.hpp"
using namespace karto;
void check(bool ok, const char * what) {if (!ok) {throw std::runtime_error(what);}}
void near(double a, double b, const char * what, double eps=1e-9) {check(std::abs(a-b)<eps,what);}
template<class F> void rejects(F f, const char * what) {bool rejected=false;try {f();}catch(const Exception &){rejected=true;}check(rejected,what);}
struct RayCapture : OccupancyGrid {
  using OccupancyGrid::AddScan;
  int rays=0,hits=0;
  RayCapture() : OccupancyGrid(200,200,{-10,-10},.1) {}
  bool RayTrace(const Vector2<double>&, const Vector2<double>&,bool hit,bool=false) override
  {++rays;hits+=hit;return true;}
};
struct InsertGrid : OccupancyGrid {using OccupancyGrid::AddScan;InsertGrid():OccupancyGrid(200,200,{-10,-10},.1){OccupancyGrid::CreateFromScans(LocalizedRangeScanVector{});}};
std::unique_ptr<LocalizedRangeScan> prepared(LaserRangeFinder * laser, const RangeReadingsVector & raw,
 const PointVectorDouble & endpoints, const PointVectorDouble & origins, const std::vector<uint8_t> & states,
 const Pose2 & pose)
{
  auto scan=std::make_unique<LocalizedRangeScan>(laser->GetName(),raw,LocalizedRangeScan::LegacyRawTag{});
  std::vector<int64_t> stamps(raw.size());std::vector<kt_int32u> ids(raw.size());
  PointVectorDouble local,ray;
  Transform transform(pose);
  for(size_t i=0;i<raw.size();++i){stamps[i]=1000000000LL+i*1000000LL;ids[i]=i;
    local.push_back(transform.InverseTransformPose(Pose2(endpoints[i],0)).GetPosition());
    ray.push_back(transform.InverseTransformPose(Pose2(origins[i],0)).GetPosition());}
  scan->SetRawSensorLimits(laser->GetMinimumRange(),laser->GetMaximumRange());
  scan->SetScanTimeProvenance(1000000000LL,1000000000LL+llround(pose.GetHeading()*1e9),"synthetic-independent-motion","analytic fixed world rays");
  scan->SetDeskewGeometry(local,ray,stamps,ids,states);
  scan->SetCorrectedPose(pose);scan->SetOdometricPose(pose);return scan;
}
int run(int argc, char ** argv)
{
  static_assert(std::is_const_v<std::remove_pointer_t<decltype(std::declval<LocalizedRangeScan>().GetRangeReadings())>>);
  const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
  auto * laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("typed_contract"));
  laser->SetMinimumRange(.05);laser->SetMaximumRange(8);laser->SetRangeThreshold(7.5);
  laser->SetMinimumAngle(-1.5);laser->SetMaximumAngle(1.5);laser->SetAngularResolution(.02);
  laser->SetOffsetPose(Pose2(.305,.01,.02));SensorManager::GetInstance()->RegisterSensor(laser);
  RangeReadingsVector raw(151,4);PointVectorDouble world,origins;std::vector<uint8_t> states(151,1);
  for(size_t i=0;i<raw.size();++i){const double a=-1.5+.02*i;
    origins.emplace_back(.305+.001*i,.01);world.emplace_back(origins.back().GetX()+4*cos(a),origins.back().GetY()+4*sin(a));}
  // Actual recording failure types 0 and 0.002 m, plus all non-finite/no-return classes.
  for(size_t i=0;i<7;++i){raw[i]=RangeReadingsVector{0,.002,nan,-inf,8.1,inf,8}[i];states[i]=i<5?0:2;}
  for(size_t i=5;i<=6;++i){const double a=-1.5+.02*i;world[i]={origins[i].GetX()+8*cos(a),origins[i].GetY()+8*sin(a)};}
  auto first=prepared(laser,raw,world,origins,states,Pose2(0,0,0));
  check(first->GetPointReadings(true).size()==144,"only sensor-valid hits match");
  for(size_t i=0;i<5;++i){check(!first->IsBeamMatchable(i),"invalid not matchable");check(!first->IsBeamRayTraceable(i),"invalid not free-space ray");}
  check(first->GetBeamState(5)==LocalizedRangeScan::BeamState::NoReturn,"explicit positive infinity no return");
  check(!first->IsBeamMatchable(5),"no-return not hit");
  RayCapture capture;capture.AddScan(first.get());check(capture.rays==146,"invalid rays excluded before origin access");check(capture.hits==144,"no-return only clears");
  InsertGrid actual;check(actual.AddScan(first.get(),true),"native occupancy inserts finite valid rays");
  std::unique_ptr<Grid<uint8_t>> grid(Grid<uint8_t>::CreateGrid(200,200,.1));
  GridIndexLookup<uint8_t> lookup(grid.get());lookup.ComputeOffsets(first.get(),0,.1,.1);
  for(unsigned a=0;a<3;++a){const auto * values=lookup.GetLookupArray(a)->GetArrayPointer();for(size_t i=0;i<7;++i)check(values[i]==INVALID_SCAN,"typed lookup must exclude invalid and no-return");}
  rejects([&]{first->SetRangeReadings(raw);},"prepared raw mutation forbidden");
  rejects([&]{PointVectorDouble p;first->SetPointReadings(p);},"prepared cache replacement forbidden");
  rejects([&]{first->GetRayOrigin(0);},"invalid origin access must reject");
  rejects([&]{auto forged=states;forged[7]=0;prepared(laser,raw,world,origins,forged,Pose2());},"sensor-valid hit cannot be disguised as invalid by algorithm gate");
  rejects([&]{Pose2 p;first->SetBarycenterPose(p);},"prepared barycenter cannot be overwritten");
  rejects([&]{BoundingBox2 b;first->SetBoundingBox(b);},"prepared bbox cannot be overwritten");
  rejects([&]{bool clean=false;first->SetIsDirty(clean);},"prepared cache freshness cannot be forged");
  LocalizedRangeScan clone(*first,LocalizedRangeScan::MeasurementCloneTag{});
  check(clone.HasDeskewGeometry(),"fine clone preserves authoritative geometry");
  check(clone.GetBeamStampNs()==first->GetBeamStampNs(),"fine clone beam identity");
  check(clone.GetRawScanStampNs()==first->GetRawScanStampNs(),"fine clone source identity");
  std::stringstream bytes;{boost::archive::binary_oarchive a(bytes);a<<clone;}
  LocalizedRangeScan restored;{boost::archive::binary_iarchive a(bytes);a>>restored;}
  {
    std::string corrupted=bytes.str();std::string statePayload(5,'\0');statePayload.append(2,'\2');statePayload.append(144,'\1');
    const size_t offset=corrupted.find(statePayload);
    check(offset!=std::string::npos && corrupted.find(statePayload,offset+1)==std::string::npos,"locate unique archived authoritative state array");
    corrupted[offset+7]='\0';
    rejects([&]{std::istringstream stream(corrupted);boost::archive::binary_iarchive a(stream);LocalizedRangeScan bad;a>>bad;},
      "archive load must execute same raw/state validator as constructor");
  }
  check(restored.GetBeamState(5)==LocalizedRangeScan::BeamState::NoReturn,"archive retains no-return state");
  check(restored.GetBeamStampNs()==clone.GetBeamStampNs(),"archive retains beam time");
  auto middle=prepared(laser,raw,world,origins,states,Pose2(.1,-.1,.1));
  auto last=prepared(laser,raw,world,origins,states,Pose2(.2,-.2,.2));
  for(auto * reference : {middle.get(),last.get()}){const auto & p=reference->GetPointReadings(false);const auto & q=first->GetPointReadings(false);
    for(size_t i=7;i<raw.size();++i){near(p[i].GetX(),q[i].GetX(),"reference invariant world hit x");near(p[i].GetY(),q[i].GetY(),"reference invariant world hit y");near(reference->GetRayOrigin(i).GetX(),first->GetRayOrigin(i).GetX(),"reference invariant origin");}}
  Mapper mapper;std::unique_ptr<ScanMatcher> matcher(ScanMatcher::Create(&mapper,.3,.01,.03,7.5));
  LocalizedRangeScanVector chain{first.get()};Pose2 mean;Matrix3 covariance;
  const double coarse=matcher->MatchScan(first.get(),chain,mean,covariance,false,false);
  clone.SetSensorPose(mean);const double fine=matcher->MatchScan(&clone,chain,mean,covariance,false);
  const double restoredScore=matcher->MatchScan(&restored,chain,mean,covariance,false,false);
  near(coarse,restoredScore,"native archive/coarse score identity",1e-12);
  check(std::isfinite(coarse)&&std::isfinite(fine)&&coarse>.5&&fine>.5,"native coarse/fine matching finite high response");
  // Same physical pose with reference reparameterization: finite grid permits a small discretization error.
  const double middleScore=matcher->MatchScan(middle.get(),chain,mean,covariance,false,false);
  const double lastScore=matcher->MatchScan(last.get(),chain,mean,covariance,false,false);
  near(coarse,middleScore,"native midpoint reference score",.03);near(coarse,lastScore,"native last reference score",.03);
  CoordinateConverter converter;
  rejects([&]{converter.WorldToGrid({nan,0});},"nan conversion rejects before integer cast");
  rejects([&]{converter.WorldToGrid({inf,0});},"inf conversion rejects before integer cast");
  rejects([&]{converter.WorldToGrid({1e300,0});},"large finite conversion rejects before integer cast");
  rejects([&]{grid->GridIndex({INT32_MAX,INT32_MAX},false);},"grid multiply add rejects overflow");
  rejects([&]{std::unique_ptr<Grid<uint8_t>> huge(Grid<uint8_t>::CreateGrid(INT32_MAX,2,.1));},"grid dimensions reject overflow");
  auto extremeWorld=world;extremeWorld[7]={1e300,1e300};
  rejects([&]{prepared(laser,raw,extremeWorld,origins,states,Pose2(0,0,0));},"extreme finite ray rejected by measurement adapter");
  {
    auto* boundsLaser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("bbox_contract"));
    boundsLaser->SetMinimumRange(.31);boundsLaser->SetMaximumRange(100);boundsLaser->SetRangeThreshold(8);
    boundsLaser->SetMinimumAngle(0);boundsLaser->SetMaximumAngle(.3);boundsLaser->SetAngularResolution(.1);
    SensorManager::GetInstance()->RegisterSensor(boundsLaser);
    RangeReadingsVector boundsRaw{.2,inf,100,4};
    LocalizedRangeScan rigid(boundsLaser->GetName(),boundsRaw,LocalizedRangeScan::LegacyRawTag{});
    rigid.SetRawSensorLimits(.05,100);rigid.SetCorrectedPose(Pose2(0,0,0));
    check(rigid.GetBeamState(0)==LocalizedRangeScan::BeamState::Hit,"algorithm min cannot change sensor hit identity");
    check(!rigid.IsBeamMatchable(0),"algorithm min excludes valid short hit from matching");
    LocalizedRangeScan typed(boundsLaser->GetName(),boundsRaw,LocalizedRangeScan::LegacyRawTag{});
    typed.SetRawSensorLimits(.05,100);PointVectorDouble ends,from;
    for(size_t i=0;i<4;++i){const double length=i==1||i==2?100:boundsRaw[i];ends.emplace_back(length*cos(.1*i),length*sin(.1*i));from.emplace_back(0,0);}
    typed.SetDeskewGeometry(ends,from,{0,1,2,3},{0,1,2,3},{1,2,2,1});typed.SetCorrectedPose(Pose2(0,0,0));
    const auto a=rigid.GetBoundingBox(),b=typed.GetBoundingBox();
    near(a.GetMinimum().GetX(),b.GetMinimum().GetX(),"raw and typed bbox minimum x");
    near(a.GetMaximum().GetX(),b.GetMaximum().GetX(),"raw and typed bbox maximum x");
    near(a.GetMaximum().GetY(),b.GetMaximum().GetY(),"raw and typed bbox maximum y");
    check(b.GetSize().GetWidth()<=4.000001 && b.GetSize().GetHeight()<=4.000001,"no-return rays do not expand legacy matching-hit extent");
    near(b.GetMaximum().GetX(),4*cos(.3),"bbox uses selected hit rather than no-return clearing limit");
    near(b.GetMaximum().GetY(),4*sin(.3),"bbox selected hit y");
    LocalizedRangeScan moving(boundsLaser->GetName(),boundsRaw,LocalizedRangeScan::LegacyRawTag{});
    moving.SetRawSensorLimits(.05,100);
    from[3]={-1,-2};ends[3]={-1+4*cos(.3),-2+4*sin(.3)};
    moving.SetDeskewGeometry(ends,from,{0,1,2,3},{0,1,2,3},{1,2,2,1});moving.SetCorrectedPose(Pose2(0,0,0));
    near(moving.GetBoundingBox().GetMinimum().GetX(),-1,"selected compensated hit retains physical origin in bbox x");
    near(moving.GetBoundingBox().GetMinimum().GetY(),-2,"selected compensated hit retains physical origin in bbox y");
    rejects([&]{lookup.ComputeOffsets(&typed,nan,.1,.1);},"non-finite lookup angle must reject");
    rejects([&]{lookup.ComputeOffsets(&typed,0,.1,0);},"zero lookup angle step must reject");
    SensorManager::GetInstance()->UnregisterSensor(boundsLaser);delete boundsLaser;
  }
  check(argc == 3,"pass original recorded scan fixture and archive-v1 fixture paths");
  {
    LocalizedRangeScan oldPrepared;
    rejects([&]{std::ifstream input(argv[2],std::ios::binary);boost::archive::binary_iarchive archive(input);archive>>oldPrepared;},
      "historical v1 prepared archive must reject missing typed validity rather than raw fallback");
  }
  {
    RecordedScanFixture fixture(argv[1]);
    auto* realLaser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("original_recorded_contract"));
    realLaser->SetMinimumRange(.31);realLaser->SetMaximumRange(fixture.range_max);realLaser->SetRangeThreshold(8);
    realLaser->SetMinimumAngle(fixture.angle_min);realLaser->SetMaximumAngle(fixture.angle_min+(fixture.ranges.size()-1)*fixture.angle_increment);
    realLaser->SetAngularResolution(fixture.angle_increment);
    const auto extrinsic=fixture.history.lookup("base_footprint",fixture.frame,fixture.stamp,100000000);
    realLaser->SetOffsetPose(Pose2(extrinsic.x,extrinsic.y,extrinsic.yaw));
    SensorManager::GetInstance()->RegisterSensor(realLaser);
    RangeReadingsVector originalRaw(fixture.ranges.begin(),fixture.ranges.end());
    std::vector<std::unique_ptr<LocalizedRangeScan>> references;
    const auto start=fixture.prepare(0);const auto origin=start.odom_from_base_reference;
    int invalid=0;
    for(double fraction : {0.,.5,1.}) {
      const auto timed=fixture.prepare(fraction);
      auto scan=std::make_unique<LocalizedRangeScan>(realLaser->GetName(),originalRaw,LocalizedRangeScan::LegacyRawTag{});
      scan->SetRawSensorLimits(fixture.range_min,fixture.range_max);
      PointVectorDouble ends,from;std::vector<int64_t> times;std::vector<kt_int32u> indices;std::vector<uint8_t> kind;
      for(const auto& beam:timed.beams){kind.push_back(static_cast<uint8_t>(beam.state));times.push_back(beam.stamp_ns);indices.push_back(beam.original_index);
        if(!beam.valid()){ends.emplace_back(0,0);from.emplace_back(0,0);if(fraction==0)++invalid;}
        else{const auto p=beam.endpointPoint(),o=beam.originPoint();ends.emplace_back(p.x,p.y);from.emplace_back(o.x,o.y);}}
      scan->SetScanTimeProvenance(timed.raw_stamp_ns,timed.reference_stamp_ns,timed.mapping_id,timed.mapping_source);
      scan->SetDeskewGeometry(ends,from,times,indices,kind);
      const auto relative=duojin02_scan_geometry::compose(duojin02_scan_geometry::inverse(origin),timed.odom_from_base_reference);
      scan->SetCorrectedPose(Pose2(relative.x,relative.y,relative.yaw));references.push_back(std::move(scan));
    }
    check(invalid==4,"recorded 811-beam scan contains exactly four finite short invalid returns");
    for(size_t i=0;i<fixture.ranges.size();++i){
      const auto &firstPoints=references.front()->GetPointReadings(false);
      for(size_t ref=1;ref<3;++ref){
        check(references[ref]->GetBeamState(i)==references[0]->GetBeamState(i),"real source selection reference invariant");
        if(!references[0]->IsBeamRayTraceable(i))continue;
        const auto &points=references[ref]->GetPointReadings(false);
        near(points[i].GetX(),firstPoints[i].GetX(),"real world endpoint reference invariant",1e-10);
        near(points[i].GetY(),firstPoints[i].GetY(),"real world endpoint reference invariant",1e-10);
        near(references[ref]->GetRayOrigin(i).GetX(),references[0]->GetRayOrigin(i).GetX(),"real world ray reference invariant",1e-10);
      }
    }
    std::unique_ptr<ScanMatcher> realMatcher(ScanMatcher::Create(&mapper,.3,.01,.03,8));
    LocalizedRangeScanVector realChain{references[0].get()};
    double responses[3];
    for(size_t ref=0;ref<3;++ref){auto& scan=*references[ref];GridIndexLookup<uint8_t> realLookup(grid.get());realLookup.ComputeOffsets(&scan,0,.1,.1);
      for(size_t i=0;i<fixture.ranges.size();++i)if(!scan.IsBeamMatchable(i))check(realLookup.GetLookupArray(1)->GetArrayPointer()[i]==INVALID_SCAN,"actual short/no-return excluded at native lookup");
      InsertGrid occupancy;occupancy.AddScan(&scan,true);
      LocalizedRangeScan temporary(scan,LocalizedRangeScan::MeasurementCloneTag{});
      responses[ref]=realMatcher->MatchScan(&scan,realChain,mean,covariance,false,false);
      const double cloned=realMatcher->MatchScan(&temporary,realChain,mean,covariance,false,false);
      near(responses[ref],cloned,"actual coarse/fine cloned query score identity",1e-12);
      std::stringstream archive;{boost::archive::binary_oarchive a(archive);a<<temporary;}
      LocalizedRangeScan loaded;{boost::archive::binary_iarchive a(archive);a>>loaded;}
      const double loadedScore=realMatcher->MatchScan(&loaded,realChain,mean,covariance,false,false);
      near(responses[ref],loadedScore,"actual archive query score identity",1e-12);
    }
    near(responses[0],responses[1],"actual first/mid score within grid discretization",.03);
    near(responses[0],responses[2],"actual first/last score within grid discretization",.03);
    std::cout<<"recorded fixture source="<<fixture.stamp<<" beams="<<fixture.ranges.size()<<" invalid="<<invalid
      <<" scores="<<responses[0]<<","<<responses[1]<<","<<responses[2]<<"\n";
    SensorManager::GetInstance()->UnregisterSensor(realLaser);delete realLaser;
  }
  std::cout<<"{\"sanitized_production_karto\":true,\"beams\":151,\"invalid\":5,\"no_return\":2,\"hit\":144,\"coarse\":"<<coarse<<",\"fine\":"<<fine<<",\"reference_mid\":"<<middleScore<<",\"reference_last\":"<<lastScore<<"}\n";
  SensorManager::GetInstance()->UnregisterSensor(laser);delete laser;
  return 0;
}
int main(int argc,char**argv) {try{return run(argc,argv);}catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<"\n";return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
