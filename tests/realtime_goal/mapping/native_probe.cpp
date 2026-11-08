#include <karto_sdk/Mapper.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <tbb/global_control.h>
using namespace karto;
using J=nlohmann::json;
void require(bool b,const char*s){if(!b)throw std::runtime_error(s);}
Pose2 pose(const J&a){return Pose2(a[0],a[1],a[2]);}
J jp(const Pose2&p){return {p.GetX(),p.GetY(),p.GetHeading()};}
J diagnostic(ScanMatcher*m) {
 const auto&d=m->GetLastMatchDiagnostics();J modes=J::array();
 for(const auto&x:d.modes)modes.push_back({{"pose",jp(x.pose)},{"plateau_size",x.plateau_size},{"centroid",jp(x.centroid)},{"search_center_cost",x.search_center_cost}});
 J stages=J::array(); for(const auto&s:d.stages)stages.push_back({{"center",jp(s.search_center)},{"selected_pose",jp(s.selected_pose)},{"peak_score",s.peak_score},{"selected_score",s.selected_pose_score},{"unresolved",s.unresolved},{"selection_tied",s.selection_tied},{"fine",s.doing_fine_match},{"mode_count",s.modes.size()},{"lattice",s.lattice_context}});
 return {{"selection_policy",d.selection_policy},{"selection_tied",d.selection_tied},{"projected_to_attained",d.projected_to_attained},{"stages",stages},{"peak_score",d.peak_score},{"returned_pose_score",d.selected_pose_score},{"pose",jp(d.selected_pose)},{"unresolved",d.unresolved},{"modes",modes},{"prior_weighted",d.prior_weighted},{"mode_diagnostics_collected",d.mode_diagnostics_collected},{"score_evaluated",d.score_evaluated},{"boundary_hit",d.boundary_hit},{"degeneracy_status",d.degeneracy_status},{"support_identity_status",d.support_identity_status},{"support_known",d.support.known()},{"support_raw_sources",d.support.raw_sources},{"support_context",d.support.context},{"grid_payload_bytes",d.correlation_grid_payload?d.correlation_grid_payload->bytes.size():0}};
}
J observation(LocalizedRangeScan*s) {
 // Local endpoints derive from the actual raw/prepared scan, never occupancy cells.
 const auto p=s->GetCorrectedPose();const double c=cos(p.GetHeading()),sn=sin(p.GetHeading());
 J points=J::array(),indices=J::array(),origins=J::array();const auto&world=s->GetPointReadings(false);
 for(unsigned i=0;i<world.size();++i)if(s->IsBeamMatchable(i)){
  double x=world[i].GetX()-p.GetX(),y=world[i].GetY()-p.GetY();
  points.push_back({c*x+sn*y,-sn*x+c*y});indices.push_back(i);
  const auto origin=s->GetRayOrigin(i);double ox=origin.GetX()-p.GetX(),oy=origin.GetY()-p.GetY();origins.push_back({c*ox+sn*oy,-sn*ox+c*oy});
 }
 const std::string bytes=s->GetMeasurementIdentityPayload();std::string hex;hex.reserve(bytes.size()*2);const char*alphabet="0123456789abcdef";for(unsigned char v:bytes){hex+=alphabet[v>>4];hex+=alphabet[v&15];}
 const auto*l=s->GetLaserRangeFinder();J config={{"minimum_angle",l->GetMinimumAngle()},{"maximum_angle",l->GetMaximumAngle()},{"angular_resolution",l->GetAngularResolution()},{"minimum_range",s->GetRawSensorMinimumRange()},{"maximum_range",s->GetRawSensorMaximumRange()},{"sensor_offset_pose",jp(l->GetOffsetPose())},{"time_mapping_id",s->GetTimeMappingId()}};
 return {{"source_id",s->GetSensorName().ToString()+":"+std::to_string(s->GetRawScanStampNs())},{"raw_stamp_ns",s->GetRawScanStampNs()},{"physical_reference_ns",s->GetPhysicalReferenceNs()},{"geometry_kind",s->GetGeometryVersion()},{"points",points},{"beam_indices",indices},{"ray_origins",origins},{"measurement_payload_hex",hex},{"sensor_geometry_config",config},{"base_pose",jp(p)}};
}
void synthetic(std::ofstream&out){
 auto*l=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("realtime-modes"));
 l->SetMinimumRange(.05);l->SetMaximumRange(4);l->SetRangeThreshold(2);l->SetMinimumAngle(0);l->SetMaximumAngle(0);l->SetAngularResolution(.01);l->SetOffsetPose(Pose2());SensorManager::GetInstance()->RegisterSensor(l);
 {
 LocalizedRangeScan q(l->GetName(),RangeReadingsVector{1},LocalizedRangeScan::LegacyRawTag{});q.SetRawSensorLimits(.05,4);q.SetCorrectedPose(Pose2());q.SetOdometricPose(Pose2());Mapper mapper;
 std::unique_ptr<ScanMatcher>m(ScanMatcher::Create(&mapper,.4,.01,.03,2));m->SetCollectMatchDiagnostics(true);auto*g=m->GetCorrelationGrid();g->GetCoordinateConverter()->SetOffset(Vector2<double>(-2.2,-2.2));
 // Same lattice: isolated single mode, separated equal peaks, weak/empty, boundary.
 for(std::string kind:{"single","separated","weak","boundary"})for(int policy=0;policy<3;++policy){
 g->Clear();std::vector<double>xs;if(kind=="single")xs={1.02};if(kind=="separated")xs={.95,1.05};if(kind=="boundary")xs={1.10};
 for(double x:xs){auto c=g->WorldToGrid(Vector2<double>(x,0));g->GetDataPointer()[g->GridIndex(c)]=GridStates_Occupied;}
 m->SetSelectAttainedMode(policy==2);m->SetReturnSelectedPoseScore(policy>=1);Pose2 mean;Matrix3 cov;
 double returned=m->CorrelateScan(&q,Pose2(0,0,1e-9),Vector2<double>(.1,0),Vector2<double>(.01,.01),1e-9,.01,false,mean,cov,false);
 J d=diagnostic(m.get());double actual=m->EvaluatePoseOnCurrentGrid(&q,mean);require(fabs(actual-d["returned_pose_score"].get<double>())<1e-12,"actual pose score agrees");
 if(policy>=1)require(fabs(returned-actual)<1e-12,"score-only and attained policy report actual score");
 if(kind=="separated"){require(d["unresolved"]&&d["modes"].size()==2,"separate maxima visible");require(policy==2?actual==1:actual==0,"native midpoint counterexample and attained fix");}
 if(kind=="single"||kind=="boundary")require(actual==1,"single/boundary mode attained");
 if(kind=="weak")require(actual==0&&d["unresolved"].get<bool>(),"empty support degrades");
 out<<J{{"type","native_synthetic"},{"kind",kind},{"policy",policy},{"returned_scalar",returned},{"diagnostics",d},{"passed",true}}.dump()<<'\n';
 }
 }SensorManager::GetInstance()->UnregisterSensor(l);delete l;
}
int main(int argc,char**argv){try{
 tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism,2);
 require(argc==4,"pool epoch output");std::string outdir=argv[3];std::ofstream out(outdir+"/results.jsonl"),frozen(outdir+"/frozen-real-scans.jsonl");synthetic(out);J pool;std::ifstream in(argv[1]);in>>pool;
 for(std::string arm:{"T0D0","T0D1"}){
 std::string base=std::string(argv[2])+"/mapping/"+arm+"/maps/mapping_default";Dataset dataset;dataset.LoadFromFile(base+".data");for(auto*o:dataset.GetLasers())SensorManager::GetInstance()->RegisterSensor(dynamic_cast<Sensor*>(o));Mapper mapper;mapper.LoadFromFile(base+".posegraph");mapper.Initialize(8.);std::map<int,LocalizedRangeScan*>scans;for(auto&s:mapper.GetGraph()->GetVertices())for(auto&v:s.second)scans[v.first]=v.second->GetObject();
 bool sawPositive=false,sawNegative=false;
 for(const auto&a:pool["arms"][arm]["attempts"]){bool positive=a.contains("fine")&&a["fine"]["decision"]=="accepted";if(positive?sawPositive:sawNegative)continue;if(positive)sawPositive=true;else sawNegative=true;
 for(auto it=a["reference_base_poses"].begin();it!=a["reference_base_poses"].end();++it)scans.at(std::stoi(it.key()))->SetCorrectedPoseAndUpdate(pose(it.value()));
 auto*q=scans.at(a["query_id"].get<int>());q->SetCorrectedPoseAndUpdate(pose(a["query_base_before"]));LocalizedRangeScanVector refs;J refjson=J::array();for(auto&i:a["coarse"]["chain_ids"]){auto*s=scans.at(i.get<int>());require(s!=q,"query-before-insert");refs.push_back(s);refjson.push_back(observation(s));}
 LocalizedRangeScan query(*q,LocalizedRangeScan::MeasurementCloneTag{});query.SetSensorPose(pose(a["coarse"]["search_center_sensor_pose"]));auto*m=mapper.GetGraph()->GetLoopScanMatcher();m->SetCollectMatchDiagnostics(true);m->SetCaptureCorrelationGrid(true);J rows=J::array();
 for(int policy=0;policy<3;++policy){m->SetSelectAttainedMode(policy==2);m->SetReturnSelectedPoseScore(policy>=1);Pose2 mean;Matrix3 cov;double score=m->MatchLidarEvidence(&query,refs,mean,cov,false);require(m->GetLastMatchDiagnostics().support.known()&&m->GetLastMatchDiagnostics().support.views.size()==refs.size()+1&&m->GetLastMatchDiagnostics().correlation_grid_payload&&!m->GetLastMatchDiagnostics().correlation_grid_payload->bytes.empty(),"actual result owns complete source and grid support");J d=diagnostic(m);double actual=m->EvaluatePoseOnCurrentGrid(&query,mean);require(fabs(actual-d["returned_pose_score"].get<double>())<1e-12,"real actual rescore");if(policy>=1)require(fabs(score-actual)<1e-12,"real scalar consistency");rows.push_back({{"policy",policy},{"scalar",score},{"diagnostics",d},{"coarse_gate",score>.45&&cov(0,0)<9&&cov(1,1)<9}});}
 out<<J{{"type","native_frozen_actual"},{"arm",arm},{"attempt_index",a["attempt_index"]},{"query_id",a["query_id"]},{"historical_gate_label",positive?"positive":"negative"},{"support_sidecar","frozen-real-scans.jsonl, same arm/attempt; source payload/pose hashes in manifest.json"},{"label_scope","archived matcher gate, not external physical correctness"},{"policies",rows},{"passed",true}}.dump()<<'\n';
 frozen<<J{{"arm",arm},{"attempt_index",a["attempt_index"]},{"label",positive?"positive":"negative"},{"query",observation(&query)},{"references",refjson},{"no_GT",true},{"geometry_scope","actual raw/prepared endpoints with historical frozen reference poses; no independent ground truth"}}.dump()<<'\n';
 if(sawPositive&&sawNegative)break;
 }
 require(sawPositive&&sawNegative,"both actual frozen gate labels represented");
 }
 std::ifstream maps("/proc/self/maps");std::ofstream saved(outdir+"/process.maps");saved<<maps.rdbuf();return 0;
}catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
