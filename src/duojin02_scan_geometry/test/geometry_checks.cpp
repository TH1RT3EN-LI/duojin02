#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include "duojin02_scan_geometry/geometry.hpp"
using namespace duojin02_scan_geometry;
void check(bool ok, const char * message) {if (!ok) {throw std::runtime_error(message);}}
void near(double a, double b, const char * message) {check(std::abs(a-b)<1e-10, message);}
template<class F> void rejects(F fn, Failure reason)
{try {fn();} catch (const Error & e) {check(e.reason==reason, "wrong failure reason"); return;} throw std::runtime_error("accepted invalid input");}
int main()
{
  Config config; config.enabled=true; config.max_interpolation_gap_sec=2;
  config.tf_history_sec=120;
  const auto declared=motionModelJson(config);
  check(declared.find("\"uncertainty\":\"conditional_not_marginalized\"")!=std::string::npos,
    "conditional uncertainty status missing");
  check(declared.find("\"temporal_covariance\":\"not_supplied\"")!=std::string::npos,
    "missing temporal covariance misrepresented");
  check(declared.find("\"beam_noise_model\":\"consumer_defined\"")!=std::string::npos &&
    declared.find("beam_independence")==std::string::npos,
    "geometry must not declare consumer beam independence");
  check(declared.find("\"shared_motion\":\"conditioned_not_marginalized\"")!=std::string::npos,
    "shared motion conditioning status missing");
  check(contractJson(config).find("\"motion_model\":"+declared)!=std::string::npos,
    "contract and diagnostic models differ");
  History history;
  history.insert("base", "laser", 0, {0.305,0,0}, true, 120000000000LL);
  history.insert("odom", "base", 1000000000LL, {}, false, 120000000000LL);
  history.insert("odom", "base", 2000000000LL, {0,0,1.0}, false, 120000000000LL);
  auto scan=prepare(1000000000LL,"laser",0,0,0.5,0.05,10,{2,2,2},config,history,"odom","base");
  near(scan.beams[0].endpointPoint().x,2.305,"static first endpoint");
  near(scan.beams[2].endpointPoint().x,2.305*std::cos(1.0),"rotation endpoint lever arm");
  near(scan.beams[2].originPoint().x,0.305*std::cos(1.0),"rotation origin lever arm");
  near(scan.beams[2].originPoint().y,0.305*std::sin(1.0),"moving origin retained");
  check(scan.reference_stamp_ns==scan.raw_stamp_ns,"D0 reference preservation");
  check(scan.beams[2].original_index==2,"original index retained");
  const Pose a{1,2,0.7}, b{2,5,-0.4};
  const Pose mid=interpolate(a,b,0.5);
  const Pose delta=compose(inverse(a),mid);
  const Pose repeated=compose(mid,delta);
  near(repeated.x,b.x,"constant twist x"); near(repeated.y,b.y,"constant twist y");
  const Pose mirrored{0.2,0.1,0.4,true};
  const Point mapped=apply(inverse(mirrored),apply(mirrored,{1,2}));
  near(mapped.x,1,"upside-down inverse x"); near(mapped.y,2,"upside-down inverse y");
  rejects([&]{history.lookup("odom","base",999999999LL,2000000000LL);},Failure::MissingPast);
  rejects([&]{history.lookup("odom","base",2000000001LL,2000000000LL);},Failure::MissingFuture);
  rejects([&]{history.lookup("odom","base",1500000000LL,100000000LL);},Failure::InterpolationGap);
  near(history.lookup("odom","base",1000000000LL,1).x,0,"exact knot allowed beside gap");
  rejects([&]{history.insert("odom","base",1500000000LL,{},false,1000000000LL);},Failure::InvalidInput);
  rejects([&]{history.insert("odom","base",2000000000LL,{},false,1000000000LL);},Failure::InvalidInput);
  rejects([&]{history.lookup("odom","missing",1500000000LL,1000000000LL);},Failure::MissingFrame);
  Config offset=config; offset.capture_time_offset_sec=-0.11;
  rejects([&]{validate(offset);},Failure::InvalidInput);
  offset.mapping_id="sensitivity-minus-110ms-v1"; offset.mapping_source="explicit uncalibrated sensitivity experiment";
  near(referenceStampNs(1000000000LL,0.5,3,offset),890000000LL,"integer physical offset");
  History linear; linear.insert("base","laser",0,{0.305,0,0},true,120000000000LL);
  linear.insert("odom","base",1000000000LL,{},false,120000000000LL);
  linear.insert("odom","base",2000000000LL,{1,0,0},false,120000000000LL);
  auto invalid=prepare(1000000000LL,"laser",0,0,0.5,0.05,10,
    {2,std::numeric_limits<float>::quiet_NaN(),2},config,linear,"odom","base");
  check(!invalid.beams[1].valid(),"invalid beam retained");
  near(invalid.beams[2].endpointPoint().x,3.305,"invalid index not compacted");
  config.reference_fraction=0.5;
  auto centered=prepare(1000000000LL,"laser",0,0,0.5,0.05,10,{2,2,2},config,linear,"odom","base");
  near(centered.beams[0].originPoint().x,-0.195,"reference pose applied");
  near(centered.beams[2].originPoint().x,0.805,"last reference origin");
  rejects([&]{prepare(1000000000LL,"laser",0,0,0,0.05,10,{2,2},config,linear,"odom","base");},Failure::InvalidInput);
  History arc;
  arc.insert("base","laser",0,{0.305,0,0},true,120000000000LL);
  arc.insert("odom","base",1000000000LL,{},false,120000000000LL);
  arc.insert("odom","base",2000000000LL,{1,1,kPi/2},false,120000000000LL);
  auto turning=prepare(1000000000LL,"laser",0,0,0.5,0.05,10,{2,2,2},config,arc,"odom","base");
  near(turning.odom_from_base_reference.x,std::sqrt(0.5),"turning reference uses SE2 x");
  near(turning.odom_from_base_reference.y,1-std::sqrt(0.5),"turning reference uses SE2 y");
  auto reference_only=prepareReference(1000000000LL,"laser",0.5,3,config,arc,"odom","base");
  near(reference_only.odom_from_base_reference.x,turning.odom_from_base_reference.x,"P1 reference interpolation agrees");
  check(reference_only.beams.empty(),"P1 does not alter beam geometry");
  rejects([&]{interpolate({}, {0,0,kPi},0.5);},Failure::InvalidInput);
  // Recorded edge values must remain invalid measurements, never NaN geometry.
  auto edges=prepare(1000000000LL,"laser",0,.001,.1,.05,10,
    {0,.002f,-1,11,std::numeric_limits<float>::quiet_NaN(),
     -std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity(),10,2},
    config,arc,"odom","base");
  for (size_t i=0;i<6;++i) {
    check(edges.beams[i].state==BeamState::Invalid,"invalid state lost");
    rejects([&]{edges.beams[i].endpointPoint();},Failure::InvalidInput);
    rejects([&]{edges.beams[i].originPoint();},Failure::InvalidInput);
  }
  check(edges.beams[0].invalidReason()==InvalidReason::BelowSensorMinimum,"finite zero reason");
  check(edges.beams[3].invalidReason()==InvalidReason::AboveSensorMaximum,"above-max reason");
  check(edges.beams[6].state==BeamState::NoReturn && edges.beams[7].state==BeamState::NoReturn,
    "explicit max-or-positive-infinity no-return policy");
  check(edges.beams[8].state==BeamState::Hit,"hit classification");
  rejects([&]{FinitePoint({std::numeric_limits<double>::quiet_NaN(),0});},Failure::InvalidInput);
  // The same physical rays must be invariant to first/middle/last reference.
  for (double fraction : {0.,.5,1.}) {
    Config reference=config; reference.reference_fraction=fraction;
    auto sample=prepare(1000000000LL,"laser",-.2,.2,.5,.05,10,{2,3,4},
      reference,arc,"odom","base");
    for (const auto & beam : sample.beams) {
      const auto world_endpoint=apply(sample.odom_from_base_reference,beam.endpointPoint());
      const auto world_origin=apply(sample.odom_from_base_reference,beam.originPoint());
      const auto physical=arc.lookup("odom","laser",beam.stamp_ns,2000000000LL);
      const double angle=-.2+.2*beam.original_index;
      const auto expected=apply(physical,{beam.raw_range*std::cos(angle),beam.raw_range*std::sin(angle)});
      near(world_endpoint.x,expected.x,"reference-invariant world endpoint x");
      near(world_endpoint.y,expected.y,"reference-invariant world endpoint y");
      near(world_origin.x,physical.x,"reference-invariant world ray origin x");
      near(world_origin.y,physical.y,"reference-invariant world ray origin y");
    }
  }
  std::cout<<"scan geometry: analytic lever arm, moving origins, constant twist, invalid indices, mirrored mount, integer timing, strict TF boundaries/gaps passed\n";
}
