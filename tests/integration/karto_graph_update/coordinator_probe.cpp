#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <karto_sdk/Mapper.h>
#include "solvers/ceres_solver.hpp"
#include KARTO_MAPPER_IMPLEMENTATION
using namespace karto;
void check(bool value,const char * why) {if (!value) {throw std::runtime_error(why);}}
struct ControlledSolver : solver_plugins::CeresSolver
{
  int prepares{0},malformed{0},aborts{0},record_throws{0},snapshots{0},statuses{0};
  bool diagnostic_interest{true};
  bool WantsMatchDiagnostics() override {return diagnostic_interest;}
  BackendSnapshot GetBackendSnapshot() override {++snapshots;return CeresSolver::GetBackendSnapshot();}
  BackendStatus GetBackendStatus() override {++statuses;return CeresSolver::GetBackendStatus();}
  bool reject{false},reject_commit{false},throw_commit_record_once{false};PreparedHandle original;
  std::vector<std::string> recorded;
  void RecordMatchDiagnostic(const std::string & json) override
  {
    if (throw_commit_record_once && json.find("\"phase\":\"result\"")!=std::string::npos &&
      json.find("\"joint_committed\":true")!=std::string::npos) {
      throw_commit_record_once=false;++record_throws;
      throw std::runtime_error("injected joint commit recording failure");
    }
    recorded.push_back(json);CeresSolver::RecordMatchDiagnostic(json);
  }
  PreparedHandle PrepareSolve(const std::string & profile="configured") override
  {
    ++prepares;original=CeresSolver::PrepareSolve(profile);
    if (!original) {return original;}
    if (!reject && malformed==0) {return original;}
    auto copy=std::make_shared<PreparedSolve>(*original);
    if (reject) {copy->numerical_accepted=false;}
    if (malformed==1) {copy->candidate.erase(copy->candidate.begin()+1);}
    if (malformed==2) {copy->candidate.push_back(copy->candidate.front());}
    if (malformed==3) {copy->candidate.back().first=999999;}
    if (malformed==4) {copy->candidate.back().second=Pose2(std::numeric_limits<double>::quiet_NaN(),0,0);}
    if (malformed==5) {++copy->before.topology_revision;}
    return copy;
  }
  bool CommitPrepared(const PreparedHandle &) override
  {return !reject_commit && CeresSolver::CommitPrepared(original);}
  bool FinalizePrepared(const PreparedHandle &) override
  {return CeresSolver::FinalizePrepared(original);}
  bool AbortPrepared(const PreparedHandle &) override
  {++aborts;return CeresSolver::AbortPrepared(original);}
};
struct ThrowingScan : LocalizedRangeScan
{
  using LocalizedRangeScan::LocalizedRangeScan;
  ThrowingScan():LocalizedRangeScan() {}
  int throws_left{0};bool permanent{false};
  void Update() override
  {
    if (permanent || throws_left>0) {if (throws_left>0) {--throws_left;}throw std::runtime_error("injected scan cache rebuild failure");}
    LocalizedRangeScan::Update();
  }
  template<class Archive> void serialize(Archive & ar,const unsigned int)
  {ar & boost::serialization::base_object<LocalizedRangeScan>(*this);}
};
BOOST_CLASS_EXPORT(ThrowingScan)
struct Fixture
{
  ControlledSolver & solver;Mapper mapper;LaserRangeFinder * laser;
  std::vector<std::unique_ptr<ThrowingScan>> scans;
  Fixture(ControlledSolver & backend,const std::string & name,bool keyframe=false,int initial_scan_count=6):solver(backend)
  {
    laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name(name));
    laser->SetMinimumRange(.05);laser->SetMaximumRange(20);laser->SetRangeThreshold(8);
    laser->SetMinimumAngle(-1.5);laser->SetMaximumAngle(1.5);laser->SetAngularResolution(.02);
    laser->SetOffsetPose(Pose2(.305,.03,.07));SensorManager::GetInstance()->RegisterSensor(laser);
    mapper.SetScanSolver(&solver);mapper.SetGraphUpdatePolicy(keyframe?"keyframe":"loop_only");
    mapper.setParamDoLoopClosing(false);mapper.setParamLinkScanMaximumDistance(1.5);
    mapper.setParamMinimumTravelDistance(0);mapper.setParamMinimumTravelHeading(0);mapper.setParamMinimumTimeInterval(0);
    for (int i=0;i<initial_scan_count;++i) {add(i);}
  }
  void add(int index)
  {
    RangeReadingsVector ranges;for(int i=0;i<151;++i) {double a=-1.5+.02*i;ranges.push_back(3.5+.7*sin(3*a)+.4*cos(7*a));}
    auto scan=std::make_unique<ThrowingScan>(laser->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});
    scan->SetRawSensorLimits(.05,20);scan->SetTime(1+.1*index);
    scan->SetScanTimeProvenance(1000000000LL+index*100000000LL,1000000000LL+index*100000000LL,"coordinator-fixture.v1","synthetic test input");
    scan->SetOdometricPose(Pose2(.02*index,0,0));check(mapper.Process(scan.get()),"fixture scan rejected");scans.push_back(std::move(scan));
  }
  ~Fixture() {for(auto & scan:scans) {scan->permanent=false;}mapper.Reset();SensorManager::GetInstance()->UnregisterSensor(laser);delete laser;}
  MapperGraph * graph() {return mapper.GetGraph();}
  ThrowingScan * query() {return scans.back().get();}
  Matrix3 covariance() {Matrix3 p;p.SetToIdentity();p(0,0)=.01;p(1,1)=.02;p(2,2)=.03;return p;}
  MapperGraph::LoopClosureResult link(int source,double offset=.2)
  {
    const auto p=query()->GetSensorPose();Pose2 mean(p.GetX()+offset,p.GetY(),p.GetHeading());
    return graph()->CommitLoopClosureCandidate({scans.at(source).get()},query(),mean,covariance(),.8,.9,p);
  }
  ScanSolver::IdPoseVector reference() {
    ScanSolver::IdPoseVector result;for(const auto & scan:scans) {result.emplace_back(scan->GetUniqueId(),scan->GetCorrectedPose());}return result;
  }
};
std::shared_ptr<rclcpp::Node> configure(ControlledSolver & solver,const std::string & name)
{auto node=std::make_shared<rclcpp::Node>(name);node->declare_parameter("debug_logging",false);solver.Configure(node);return node;}
bool same(const ScanSolver::IdPoseVector & a,const ScanSolver::IdPoseVector & b)
{if(a.size()!=b.size())return false;for(std::size_t i=0;i<a.size();++i)if(a[i].first!=b[i].first||!UpdatePoseEqual(a[i].second,b[i].second))return false;return true;}
void scheduler()
{
  ControlledSolver solver;auto node=configure(solver,"scheduler_contract");Fixture f(solver,"scheduler_laser");
  check(solver.prepares==0,"S0 introduced boundary solve");
  auto revision=f.graph()->GetUpdateState().graph_revision;
  auto rejected=f.link(0,5);check(!rejected.factor.added()&&!rejected.solve_attempted,"rejected loop invoked candidate solve");
  check(f.graph()->GetUpdateState().graph_revision==revision,"rejected loop changed graph revision");
  auto refreshed=f.graph()->RequestUpdate("accepted_keyframe_boundary",f.query());
  check(refreshed.attempted&&refreshed.committed&&solver.prepares==1,"rejected loop suppressed dirty sequential update");
  auto covered=f.graph()->RequestUpdate("new_loop_factor",f.query());
  check(!covered.attempted&&solver.prepares==1&&covered.decision=="already_covered","seed change retriggered covered graph");
  auto first=f.mapper.FinalizeGraph("one_budget");check(first.attempted&&first.committed,"explicit same-version finalize not allowed");
  const auto count=solver.prepares;auto repeat=f.mapper.FinalizeGraph("one_budget");
  check(repeat.reused&&!repeat.attempted&&solver.prepares==count,"same finalize id spent another budget");
  auto owned=f.link(0,.1);check(owned.factor.added()&&owned.committed,"new loop did not update registered graph");
  check(f.mapper.FinalizeGraph("one_budget").decision=="finalize_request_id_stale","stale finalize id certified new graph");
  std::cout<<"{\"test\":\"scheduler_all_deltas_and_budgets\",\"passed\":true}\n";
}
void failures()
{
  for(int mode=0;mode<8;++mode) {
    ControlledSolver solver;auto node=configure(solver,"joint_failure_"+std::to_string(mode));Fixture f(solver,"joint_laser_"+std::to_string(mode));
    const auto before=solver.GetBackendSnapshot();const auto mapper=f.reference();const auto edges=f.graph()->GetEdges().size();
    solver.reject=mode==0;solver.malformed=mode>=1&&mode<=5?mode:0;solver.reject_commit=mode==6;
    if(mode==7) {f.scans.at(3)->throws_left=1;}
    auto failed=f.link(0,.3);
    check(failed.factor.added()&&!failed.committed&&f.graph()->GetEdges().size()==edges+1,"failed update lost retained factor or committed");
    check(same(before.nodes,solver.GetBackendSnapshot().nodes)&&same(mapper,f.reference()),"joint failure did not restore both containers");
    auto state=f.graph()->GetUpdateState();check(state.covered_graph_revision<state.graph_revision&&!state.faulted,"failed dirty state lost or wrongly faulted");
    const auto count=solver.prepares;auto duplicate=f.graph()->RequestUpdate("accepted_keyframe_boundary",f.query());
    check(!duplicate.attempted&&solver.prepares==count,"ordinary reason secretly retried same failed graph");
    solver.reject=false;solver.malformed=0;solver.reject_commit=false;
    auto recovery=f.link(1,.15);check(recovery.committed,"new independent graph version did not recover");
    auto backend=solver.GetBackendSnapshot();check(same(backend.nodes,f.reference()),"accepted joint states differ");
  }
  ControlledSolver solver;auto node=configure(solver,"rollback_fault");Fixture f(solver,"rollback_fault_laser");
  f.scans.at(3)->permanent=true;bool threw=false;
  try {f.link(0,.3);} catch(const std::runtime_error &) {threw=true;}
  check(threw&&f.graph()->GetUpdateState().faulted,"failed rollback did not fail stop");
  bool capture_failed=false;try {f.mapper.CaptureGraphCheckpoint();}catch(...) {capture_failed=true;}
  check(capture_failed,"faulted graph emitted snapshot");
  for(auto & scan:f.scans) {scan->permanent=false;}f.mapper.Reset();
  check(solver.GetBackendSnapshot().nodes.empty(),"Reset left old backend nodes");
  f.scans.clear();f.add(10);check(f.graph()->GetUpdateState().reset_epoch>1,"Reset did not advance graph epoch");
  std::cout<<"{\"test\":\"joint_failures_and_recovery\",\"passed\":true,\"mapper_cache_failure_tested\":true}\n";
}
void commit_recording_failure()
{
  ControlledSolver solver;auto node=configure(solver,"commit_recording_failure");Fixture f(solver,"commit_recording_laser");
  const auto before=solver.GetBackendSnapshot();const auto reference=f.reference();
  solver.throw_commit_record_once=true;bool threw=false;
  try {f.link(0,.3);}catch(const std::runtime_error&) {threw=true;}
  const auto backend=solver.GetBackendSnapshot();const auto state=f.graph()->GetUpdateState();
  check(threw&&solver.record_throws==1,"one-shot commit record injection not observed");
  check(state.faulted&&state.last_decision=="joint_commit_recording_failed","committed recording failure mislabeled as rollback failure");
  check(!backend.transaction_active&&solver.aborts==0,"finalized commit attempted an invalid rollback");
  check(same(backend.nodes,f.reference())&&!same(reference,f.reference()),"committed states differ or candidate was rolled back");
  check(backend.pose_revision==before.pose_revision+1&&state.commit_id==1&&state.covered_graph_revision==state.graph_revision,"committed versions lost after recording failure");
  bool refused=false;try {f.graph()->RequestUpdate("manual",f.query());}catch(...) {refused=true;}
  check(refused,"recording failure continued graph publication");
  bool fault_recorded=false;for(const auto&json:solver.recorded) {if(json.find("joint_commit_recording_failed")!=std::string::npos)fault_recorded=true;check(json.find("joint_rollback_failed")==std::string::npos,"successful commit claimed rollback failure");}
  check(fault_recorded,"accurate fail-stop event not recorded after transient diagnostic fault");
  std::cout<<"{\"test\":\"post_finalize_recording_fail_stop\",\"passed\":true,\"both_states_consistent\":true,\"abort_calls\":0}\n";
}
void pending_retraction_and_empty_entries()
{
  Mapper empty;check(!empty.HasGraphCheckpoint(),"new mapper falsely has checkpoint");
  bool correction_refused=false,loop_refused=false;
  try {empty.CorrectPoses();}catch(const std::logic_error&) {correction_refused=true;}
  try {empty.TryCloseLoop(nullptr,Name("empty"));}catch(const std::logic_error&) {loop_refused=true;}
  check(correction_refused&&loop_refused,"empty inline graph entry did not explicitly reject");
  ControlledSolver solver;auto node=configure(solver,"pending_retraction");Fixture f(solver,"pending_retraction_laser");
  const auto before=solver.GetBackendSnapshot();const auto revision=f.graph()->GetUpdateState().graph_revision;const auto edge_count=f.graph()->GetEdges().size();
  auto pending=std::make_unique<ThrowingScan>(*f.query(),LocalizedRangeScan::MeasurementCloneTag{});
  const auto original=pending->GetSensorPose();f.mapper.GetMapperSensorManager()->AddScan(pending.get());f.graph()->AddVertex(pending.get());
  check(f.graph()->GetUpdateState().graph_revision==revision+1,"pending registration revision missing");
  check(!f.graph()->RejectPendingScan(pending.get(),original),"pending retraction falsely accepted");
  check(f.graph()->GetUpdateState().graph_revision==revision+2,"pending retraction did not advance graph revision");
  check(same(before.nodes,solver.GetBackendSnapshot().nodes)&&f.graph()->GetEdges().size()==edge_count&&f.graph()->RegistryMatchesBackend(),"pending retraction left unregistered state");
  check(pending->GetUniqueId()==-1&&pending->GetStateId()==-1,"pending identity not undone");
  bool recorded=false;for(const auto&json:solver.recorded)if(json.find("pending_node_retracted")!=std::string::npos)recorded=true;check(recorded,"pending retraction delta not observable");
  std::cout<<"{\"test\":\"pending_retraction_revision_and_empty_entries\",\"passed\":true}\n";
}
void checkpoint(const std::string & directory)
{
  ControlledSolver solver;auto node=configure(solver,"checkpoint_original");Fixture f(solver,"checkpoint_laser");
  // An explicit frontend reference change is not an optimizer warm-start edit.
  auto pose=f.query()->GetCorrectedPose();f.query()->SetCorrectedPoseAndUpdate(Pose2(pose.GetX()+.07,pose.GetY(),pose.GetHeading()));
  f.graph()->ObserveReferenceChange("fixture_frontend_only",f.query());
  const auto before=solver.GetBackendSnapshot();const auto references=f.reference();
  check(!same(before.nodes,references),"fixture did not distinguish warm start from references");
  check(!f.mapper.HasGraphCheckpoint(),"fresh graph checkpoint falsely marked present");
  f.mapper.SaveToFile(directory+"/checkpoint.posegraph");
  check(f.mapper.HasGraphCheckpoint(),"captured checkpoint getter false");
  ControlledSolver restored;auto node2=configure(restored,"checkpoint_restored");Mapper mapper;
  mapper.LoadFromFile(directory+"/checkpoint.posegraph");check(mapper.HasGraphCheckpoint(),"archive checkpoint getter false");mapper.SetScanSolver(&restored);
  for(const auto & sensor:mapper.GetGraph()->GetVertices()) for(const auto & item:sensor.second) {restored.AddNode(item.second);}
  for(auto * edge:mapper.GetGraph()->GetEdges()) {restored.AddConstraint(edge);}
  check(mapper.RestoreGraphCheckpoint(),"pure snapshot restore rejected");
  check(same(before.nodes,restored.GetBackendSnapshot().nodes),"load silently replaced backend seed with mapper state");
  check(restored.prepares==0,"LoadSnapshot implicitly computed");
  ScanSolver::IdPoseVector loaded;for(const auto & sensor:mapper.GetGraph()->GetVertices()) for(const auto & item:sensor.second) {loaded.emplace_back(item.second->GetObject()->GetUniqueId(),item.second->GetObject()->GetCorrectedPose());}
  std::sort(loaded.begin(),loaded.end(),[](const auto&a,const auto&b){return a.first<b.first;});
  check(same(loaded,references),"pure load changed frontend reference");
  auto finalized=mapper.FinalizeGraph("loaded_finalize");check(finalized.attempted&&finalized.committed,"explicit loaded finalize failed");
  auto repeated=mapper.FinalizeGraph("loaded_finalize");check(repeated.reused&&!repeated.attempted,"loaded finalize not idempotent");
  mapper.Reset();
  std::cout<<"{\"test\":\"checkpoint_two_states_and_pure_load\",\"passed\":true}\n";
}
void checkpoint_registration_gap(const std::string & directory)
{
  ControlledSolver solver;auto node=configure(solver,"checkpoint_gap_original");Fixture f(solver,"checkpoint_gap_laser");
  check(!f.mapper.HasGraphCheckpoint()&&f.mapper.RestoreGraphCheckpoint()&&!f.mapper.HasGraphCheckpoint(),"legacy fallback invented checkpoint state");
  check(f.graph()->GetUpdateState().seed_policy=="mapper_seed_fallback","legacy fallback provenance missing");
  {
    auto owner=f.mapper.LockGraphState();auto * removed=f.graph()->GetEdges().front();
    solver.RemoveConstraint(removed->GetSource()->GetObject()->GetUniqueId(),removed->GetTarget()->GetObject()->GetUniqueId());
    for(auto * vertex:{removed->GetSource(),removed->GetTarget()}) {
      const auto & adjacent=vertex->GetEdges();auto found=std::find(adjacent.begin(),adjacent.end(),removed);
      check(found!=adjacent.end(),"deleted factor adjacency missing");vertex->RemoveEdge(static_cast<int>(found-adjacent.begin()));
    }
    f.graph()->RemoveEdge(0);delete removed;
  }
  const auto before=solver.GetBackendSnapshot();check(f.graph()->RegistryMatchesBackend(),"gap fixture graph/backend mismatch");
  std::set<std::uint64_t> ids;for(const auto & record:before.constraints)ids.insert(record.registration_id);
  check(!ids.empty()&&*ids.begin()>1&&before.registration_id_watermark>*ids.begin(),"fixture has no active registration ID gap");
  f.mapper.SaveToFile(directory+"/registration-gap.posegraph");check(f.mapper.HasGraphCheckpoint(),"capture retained stale legacy-presence status");
  ControlledSolver restored;auto node2=configure(restored,"checkpoint_gap_restored");Mapper mapper;mapper.LoadFromFile(directory+"/registration-gap.posegraph");mapper.SetScanSolver(&restored);
  for(const auto & sensor:mapper.GetGraph()->GetVertices())for(const auto & item:sensor.second)restored.AddNode(item.second);
  for(auto * edge:mapper.GetGraph()->GetEdges())restored.AddConstraint(edge);
  check(mapper.HasGraphCheckpoint(),"typed checkpoint mistaken for historical fallback label");
  const auto initialized=restored.GetBackendSnapshot();check(!UpdateBackendSame(initialized,before),"fresh registration unexpectedly preserved gap without restore");
  const auto saved_ids=mapper.m_CheckpointRegistrationIds;
  for(int mode=0;mode<5;++mode) {
    mapper.m_CheckpointRegistrationIds=saved_ids;auto & altered=mapper.m_CheckpointRegistrationIds;
    if(mode==0)altered.erase(altered.begin());
    if(mode==1){auto second=std::next(altered.begin());second->second=altered.begin()->second;}
    if(mode==2)altered.begin()->second=0;
    if(mode==3)altered.begin()->second=mapper.m_CheckpointRegistrationWatermark+1;
    if(mode==4){const auto id=altered.begin()->second;altered.erase(altered.begin());altered.emplace(std::make_pair(99999,100000),id);}
    check(!mapper.RestoreGraphCheckpoint()&&UpdateBackendSame(initialized,restored.GetBackendSnapshot()),"malformed checkpoint identity map changed backend or accepted");
  }
  mapper.m_CheckpointRegistrationIds=saved_ids;check(mapper.RestoreGraphCheckpoint(),"valid registration gap restore rejected");
  check(UpdateBackendSame(before,restored.GetBackendSnapshot()),"Mapper save/load silently renumbered active backend registrations");
  auto * source=mapper.GetMapperSensorManager()->GetScan(0);auto * target=mapper.GetMapperSensorManager()->GetScan(1);
  const auto readded=mapper.GetGraph()->LinkScans(source,target,target->GetSensorPose(),f.covariance());
  const auto continued=restored.GetBackendSnapshot();
  check(readded.added()&&continued.registration_id_watermark==before.registration_id_watermark+1,"loaded graph reused removed registration ID on readd");
  bool new_identity=false;for(const auto & record:continued.constraints)if(record.source_id==0&&record.target_id==1)new_identity=record.registration_id==continued.registration_id_watermark;
  check(new_identity&&mapper.GetGraph()->RegistryMatchesBackend(),"readded graph identity missing");
  mapper.CaptureGraphCheckpoint();check(mapper.m_CheckpointRegistrationIds.at({0,1})==continued.registration_id_watermark,"continued checkpoint lost new registration identity");
  check(restored.prepares==0,"registration restoration implicitly solved");mapper.Reset();check(!mapper.HasGraphCheckpoint(),"Reset retained checkpoint-presence flag");
  std::cout<<"{\"test\":\"checkpoint_active_registration_gap\",\"passed\":true,\"malformed_identity_maps_rejected\":5,\"legacy_fallback_presence_separate\":true,\"readd_uses_fresh_identity\":true}\n";
}
void cheap_diagnostic_contract()
{
  ControlledSolver solver;auto node=configure(solver,"cheap_diagnostic_contract");Fixture f(solver,"cheap_diagnostic_laser");
  const auto before=solver.GetBackendSnapshot();const int prepares=solver.prepares;
  solver.diagnostic_interest=false;
  auto snapshot_calls=solver.snapshots,status_calls=solver.statuses;auto records=solver.recorded.size();
  auto graph_revision=f.graph()->GetUpdateState().graph_revision;
  auto reference_revision=f.graph()->GetUpdateState().reference_pose_revision;
  f.graph()->ObserveGraphDelta("fixture_light_delta");f.graph()->ObserveReferenceChange("fixture_light_reference",f.query());
  check(solver.snapshots==snapshot_calls&&solver.statuses==status_calls&&solver.recorded.size()==records,"disabled telemetry retrieved state or constructed event");
  check(f.graph()->GetUpdateState().graph_revision==graph_revision+1&&f.graph()->GetUpdateState().reference_pose_revision==reference_revision+1,"disabled telemetry lost state versions");
  solver.diagnostic_interest=true;
  f.graph()->ObserveGraphDelta("fixture_light_delta");f.graph()->ObserveReferenceChange("fixture_light_reference",f.query());
  check(solver.snapshots==snapshot_calls&&solver.statuses==status_calls+2&&solver.recorded.size()==records+2,"ordinary telemetry performed a full snapshot or lost event");
  auto delta=nlohmann::json::parse(solver.recorded.back());
  check(delta.at("state_scope")=="maintained_status"&&delta.at("state").at("backend_status_complete")==true,"light status scope missing");
  check(!delta.at("state").contains("backend_known")&&!delta.at("state").contains("solver_options")&&!delta.at("state").contains("graph_content_digest_fnv1a64"),"light event pretends full verification or serializes full metadata");
  const auto & pose=f.query()->GetCorrectedPose();auto reference=delta.at("reference_delta");
  check(reference.at("node_id")==f.query()->GetUniqueId()&&reference.at("raw_stamp_ns")==f.query()->GetRawScanStampNs()&&reference.at("base_pose")==nlohmann::json::array({pose.GetX(),pose.GetY(),pose.GetHeading()}),"reference delta lost exact actual pose/source");
  const auto full=nlohmann::json::parse(f.mapper.GetGraphUpdateMetadataJson());
  check(solver.snapshots==snapshot_calls+1&&full.at("backend_known")==true&&full.at("integrity_scope")=="full_snapshot_actual_registry_checked","explicit full metadata did not validate full snapshot");
  snapshot_calls=solver.snapshots;graph_revision=f.graph()->GetUpdateState().graph_revision;const auto request_count=f.graph()->GetUpdateState().request_count;
  f.mapper.CaptureGraphCheckpoint();
  check(solver.snapshots==snapshot_calls+1&&solver.prepares==prepares&&f.graph()->GetUpdateState().request_count==request_count&&f.graph()->GetUpdateState().graph_revision==graph_revision,"Snapshot duplicate capture, Solve, or version/request mutation");
  auto marker=nlohmann::json::parse(solver.recorded.back());check(marker.at("phase")=="checkpoint"&&marker.at("state_scope")=="full_boundary"&&marker.at("state")==full,"checkpoint fullmarker differs from full state");
  check(UpdateBackendSame(before,solver.GetBackendSnapshot()),"telemetry changed native objective/values/registry");
  std::cout<<"{\"test\":\"constant_work_delta_and_full_snapshot_boundary\",\"passed\":true,\"disabled_full_snapshots\":0,\"disabled_status_reads\":0,\"enabled_delta_full_snapshots\":0,\"snapshot_boundary_captures\":1,\"reference_pose_delta_exact\":true}\n";
}
void base_reference_roundtrip_contract(const std::filesystem::path & directory)
{
  struct Case {const char * name;Pose2 base;Pose2 offset;};
  const std::vector<Case> cases={
    {"recorded_finite_roundtrip",Pose2(-17.733405102961388,5.230360446266328,2.5550929564911837),Pose2(.30455923000105006,-.0059768664988773873,.019208065981137109)},
    {"positive_angle",Pose2(1.125,-2.25,.7),Pose2(.3,.04,.2)},
    {"negative_angle",Pose2(-1.125,2.25,-.7),Pose2(.3,.04,.2)},
    {"positive_near_zero",Pose2(.1,-.2,1e-12),Pose2(-.2,.1,-.15)},
    {"negative_near_zero",Pose2(-.1,.2,-1e-12),Pose2(.2,-.1,.15)},
    {"zero_angle_nonzero_extrinsic",Pose2(1,-2,0),Pose2(.305,.03,.07)},
    {"zero_extrinsic_identity",Pose2(1,-2,.7),Pose2(0,0,0)},
    {"zero_extrinsic_near_zero",Pose2(.1,-.2,-1e-12),Pose2(0,0,0)},
    {"zero_base_identity",Pose2(0,0,0),Pose2(0,0,0)}
  };
  nlohmann::json rows=nlohmann::json::array();int changed_cases=0,unchanged_cases=0,sensor_equal_base_changed=0;
  for(std::size_t index=0;index<cases.size();++index) {
    const auto & c=cases[index];ControlledSolver solver;
    auto node=configure(solver,"roundtrip_reference_"+std::to_string(index));
    Fixture f(solver,"roundtrip_laser_"+std::to_string(index),false,0);
    f.laser->SetOffsetPose(c.offset);f.mapper.Initialize(f.laser->GetRangeThreshold());
    f.mapper.GetMapperSensorManager()->RegisterSensor(f.laser->GetName());
    auto add=[&](int n) {
      RangeReadingsVector ranges(151,3.5);
      auto scan=std::make_unique<ThrowingScan>(f.laser->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});
      scan->SetRawSensorLimits(.05,20);scan->SetTime(1+.1*n);
      scan->SetScanTimeProvenance(1000000000LL+n*100000000LL,1000000000LL+n*100000000LL,"roundtrip-reference-test.v1","synthetic finite native fixture");
      scan->SetOdometricPose(c.base);scan->SetCorrectedPose(c.base);
      f.mapper.GetMapperSensorManager()->AddScan(scan.get());f.graph()->AddVertex(scan.get());f.scans.push_back(std::move(scan));
    };
    add(0);auto * first=f.query();f.mapper.GetMapperSensorManager()->SetLastScan(first);f.mapper.GetMapperSensorManager()->AddRunningScan(first);
    add(1);auto * query=f.query();const auto before=query->GetCorrectedPose();const auto sensor_before=query->GetSensorPose();
    // Exercise the actual native setter on a measurement clone; no copied scalar inverse formula.
    LocalizedRangeScan preview(*query,LocalizedRangeScan::MeasurementCloneTag{});preview.SetSensorPose(sensor_before);
    const auto expected=preview.GetCorrectedPose();const bool changed=before!=expected;
    const auto revision=f.graph()->GetUpdateState().reference_pose_revision;solver.recorded.clear();
    check(f.graph()->AddEdges(query,f.covariance(),.9,sensor_before),"native AddEdges fixture rejected");
    check(UpdatePoseEqual(query->GetCorrectedPose(),expected),"single-input CI differs from actual native sensor setter");
    const bool sensor_equal=sensor_before==query->GetSensorPose();
    int updates=0;for(const auto & text:solver.recorded) {
      auto event=nlohmann::json::parse(text);
      if(event.value("stage","")=="graph_update"&&event.value("phase","")=="reference_update") {
        ++updates;check(event.at("reason")=="frontend_ci","unexpected frontend change reason");
        const auto & delta=event.at("reference_delta");
        check(delta.at("node_id")==query->GetUniqueId()&&delta.at("raw_stamp_ns")==query->GetRawScanStampNs(),"reference delta lost native source identity");
        check(delta.at("base_pose")==nlohmann::json::array({expected.GetX(),expected.GetY(),expected.GetHeading()}),"reference delta does not contain exact stored base pose");
      }
    }
    rows.push_back({{"name",c.name},{"base_before",{before.GetX(),before.GetY(),before.GetHeading()}},
      {"base_after",{query->GetCorrectedPose().GetX(),query->GetCorrectedPose().GetY(),query->GetCorrectedPose().GetHeading()}},
      {"sensor_before",{sensor_before.GetX(),sensor_before.GetY(),sensor_before.GetHeading()}},
      {"sensor_after",{query->GetSensorPose().GetX(),query->GetSensorPose().GetY(),query->GetSensorPose().GetHeading()}},
      {"base_changed",changed},{"sensor_exactly_equal",sensor_equal},{"frontend_reference_events",updates},
      {"reference_revision_before",revision},{"reference_revision_after",f.graph()->GetUpdateState().reference_pose_revision}});
    {std::ofstream observed(directory/"reference-roundtrip-observations.json");observed<<rows.dump(2)<<'\n';}
    check(updates==int(changed),"base mutation journal must follow stored corrected coordinates even when sensor is equal");
    check(f.graph()->GetUpdateState().reference_pose_revision==revision+int(changed),"base mutation reference revision missing/spurious");
    changed_cases+=changed;unchanged_cases+=!changed;sensor_equal_base_changed+=sensor_equal&&changed;
    const auto reference_before=f.reference();const auto joint_revision=f.graph()->GetUpdateState().reference_pose_revision;
    auto result=f.graph()->RequestUpdate("manual",query);check(result.committed,"roundtrip joint update rejected");
    check(same(f.reference(),solver.original->candidate),"joint candidate and actual stored references differ");
    const bool joint_changed=!same(reference_before,solver.original->candidate);
    check(f.graph()->GetUpdateState().reference_pose_revision==joint_revision+int(joint_changed),"joint reference revision and exact base delta differ");
    rows.back()["joint_reference_changed"]=joint_changed;
  }
  check(changed_cases>0&&unchanged_cases>0&&sensor_equal_base_changed>0,"roundtrip controls did not cover hidden-base and true-identity cases");
  std::ofstream record(directory/"reference-roundtrip-cases.json");record<<rows.dump(2)<<'\n';
  std::cout<<"{\"test\":\"stored_base_reference_roundtrip_contract\",\"passed\":true,\"cases\":"<<cases.size()<<",\"base_changed\":"<<changed_cases<<",\"base_unchanged\":"<<unchanged_cases<<",\"sensor_equal_base_changed\":"<<sensor_equal_base_changed<<",\"real_native_setter_and_AddEdges\":true}\n";
}

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);try {
    check(argc==2,"output directory required");std::filesystem::create_directories(argv[1]);
    base_reference_roundtrip_contract(argv[1]);scheduler();failures();commit_recording_failure();pending_retraction_and_empty_entries();checkpoint(argv[1]);checkpoint_registration_gap(argv[1]);cheap_diagnostic_contract();
  } catch(const Exception & e) {std::cerr<<e.GetErrorMessage()<<'\n';rclcpp::shutdown();return 1;}
  catch(const std::exception & e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
