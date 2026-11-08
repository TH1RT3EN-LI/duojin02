#include <math.h>
#include <ceres/local_parameterization.h>
#include <ceres/ceres.h>
#include <vector>
#include <fstream>
#include <map>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <limits>
#include <set>
#include <memory>
#include <iostream>
#include <karto_sdk/Mapper.h>
#include "solvers/ceres_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "slam_toolbox/toolbox_types.hpp"
#define private public
#include "solvers/ceres_solver.hpp"
#undef private
#include KARTO_MAPPER_IMPLEMENTATION
using namespace karto;
void require(bool b,const char * reason){if(!b)throw std::runtime_error(reason);}
bool equal(const Pose2&a,const Pose2&b){return a.GetX()==b.GetX()&&a.GetY()==b.GetY()&&a.GetHeading()==b.GetHeading();}
struct MockSolver:ScanSolver {
 std::map<int,Pose2> nodes;std::set<std::pair<int,int>> factors;IdPoseVector corrections;
 std::vector<std::string> events;int computes=0;bool accept=true,throw_compute=false,throw_registration=false,unknown_registration=false,unknown_compute=false;
 int malformed=0;ComputeState state=ComputeState::Unknown;
 void Configure(rclcpp::Node::SharedPtr)override{}
 void AddNode(Vertex<LocalizedRangeScan>*v)override{nodes[v->GetObject()->GetUniqueId()]=v->GetObject()->GetCorrectedPose();}
 void AddConstraint(Edge<LocalizedRangeScan>*e)override{if(throw_registration)throw std::runtime_error("injected registration failure before mutation");factors.insert({e->GetSource()->GetObject()->GetUniqueId(),e->GetTarget()->GetObject()->GetUniqueId()});}
 ConstraintRegistration GetConstraintRegistration(int a,int b)override{return unknown_registration?ConstraintRegistration::Unknown:(factors.count({a,b})?ConstraintRegistration::Registered:ConstraintRegistration::Rejected);}
 void Compute()override{++computes;corrections.clear();state=ComputeState::Rejected;if(throw_compute)throw std::runtime_error("injected atomic Compute failure");if(!accept)return;state=unknown_compute?ComputeState::Unknown:ComputeState::Accepted;for(const auto&e:nodes)corrections.push_back(e);corrections.back().second=Pose2(.12,.03,.01);if(malformed==1)corrections.clear();else if(malformed==2)corrections.push_back(corrections.front());else if(malformed==3)corrections.push_back({99999,Pose2()});else if(malformed==4)corrections.pop_back();else if(malformed==5)corrections.back().second=Pose2(std::numeric_limits<double>::quiet_NaN(),0,0);}
 ComputeState GetComputeState()const override{return state;}
 const IdPoseVector&GetCorrections()const override{return corrections;}
 void Clear()override{corrections.clear();}
 void RecordMatchDiagnostic(const std::string&s)override{events.push_back(s);}
};
struct Fixture {
 ScanSolver &solver;LaserRangeFinder *laser;Mapper mapper;std::vector<std::unique_ptr<LocalizedRangeScan>> scans;
 Fixture(ScanSolver&s,const std::string&name,bool barycenter=false):solver(s){
  laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name(name));
  laser->SetMinimumRange(.05);laser->SetMaximumRange(20);laser->SetRangeThreshold(8);laser->SetMinimumAngle(-1.5);laser->SetMaximumAngle(1.5);laser->SetAngularResolution(.02);laser->SetOffsetPose(Pose2(.305,.03,.07));SensorManager::GetInstance()->RegisterSensor(laser);
  mapper.SetScanSolver(&solver);mapper.setParamDoLoopClosing(false);mapper.setParamLinkScanMaximumDistance(1.5);mapper.setParamUseScanBarycenter(barycenter);mapper.setParamMinimumTravelDistance(0);mapper.setParamMinimumTravelHeading(0);mapper.setParamMinimumTimeInterval(0);
  for(int k=0;k<6;++k){RangeReadingsVector ranges;PointVectorDouble points,origins;std::vector<int64_t> stamps;std::vector<kt_int32u>indices;std::vector<uint8_t>states;
   for(int i=0;i<151;++i){double a=-1.5+.02*i,r=3.5+.7*sin(3*a)+.4*cos(7*a);ranges.push_back(r);auto o=laser->GetOffsetPose();points.emplace_back(o.GetX()+r*cos(a+o.GetHeading()),o.GetY()+r*sin(a+o.GetHeading()));origins.emplace_back(o.GetX(),o.GetY());stamps.push_back(1000000000LL+k*100000000LL+i*10000LL);indices.push_back(i);states.push_back(1);}
   auto q=std::make_unique<LocalizedRangeScan>(laser->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});q->SetRawSensorLimits(.05,20);q->SetScanTimeProvenance(1000000000LL+k*100000000LL,1000000000LL+k*100000000LL,"native-lifecycle.v1","synthetic fixed geometry");q->SetDeskewGeometry(points,origins,stamps,indices,states);q->SetOdometricPose(Pose2(.02*k,0,0));require(mapper.Process(q.get()),"fixture Process rejected");scans.push_back(std::move(q));}
 }
 ~Fixture(){mapper.Reset();SensorManager::GetInstance()->UnregisterSensor(laser);delete laser;}
 MapperGraph* graph(){return mapper.GetGraph();}LocalizedRangeScan*query(){return scans.back().get();}
 Matrix3 covariance(){Matrix3 p;p.SetToIdentity();p(0,0)=.01;p(1,1)=.02;p(2,2)=.03;return p;}
 Pose2 mean(double x=.1){auto p=query()->GetSensorPose();return Pose2(p.GetX()+x,p.GetY(),p.GetHeading());}
 MapperGraph::LoopClosureResult submit(int source,double x=.1,const measurement::Evidence&e={}){return graph()->CommitLoopClosureCandidate({scans.at(source).get()},query(),mean(x),covariance(),.8,.9,query()->GetSensorPose(),e);}
};
void mock_cases(){
 for(bool bary:{false,true}){MockSolver s;Fixture f(s,bary?"mock_bary":"mock_sensor",bary);auto pose=f.query()->GetCorrectedPose();const size_t before=f.graph()->GetEdges().size();
  auto rejected=f.submit(0,5);require(rejected.factor.status==MapperGraph::LinkStatus::Rejected&&!rejected.solve_attempted&&!rejected.committed,"distance reject lifecycle");require(s.computes==0&&equal(pose,f.query()->GetCorrectedPose())&&f.graph()->GetEdges().size()==before,"distance reject mutated state");
  // Compare the old temporary-write association geometry against the private clone.
  LocalizedRangeScan copy(*f.query(),LocalizedRangeScan::MeasurementCloneTag{});copy.SetSensorPose(f.mean());auto candidate=copy.GetReferencePose(bary);auto old=f.query()->GetSensorPose();f.query()->SetSensorPose(f.mean());require(equal(candidate,f.query()->GetReferencePose(bary)),"candidate reference differs from old temporary-write geometry");f.query()->SetCorrectedPose(pose);
  auto added=f.submit(0);require(added.factor.added()&&added.factor.backend_registered&&added.committed&&s.computes==1,"added candidate not committed");require(equal(f.query()->GetCorrectedPose(),Pose2(.12,.03,.01)),"candidate pose committed instead of accepted solver correction");
  auto evidence=dynamic_cast<LinkInfo*>(added.factor.edge->GetLabel())->GetEvidence();auto owned=f.submit(0,.1,evidence);require(owned.factor.status==MapperGraph::LinkStatus::AlreadyOwned&&!owned.solve_attempted&&!owned.committed&&s.computes==1,"owned candidate solved or committed");
  auto different=evidence;different.context+="different independent candidate";auto occupied=f.submit(0,.1,different);require(occupied.factor.status==MapperGraph::LinkStatus::Rejected&&!occupied.solve_attempted&&s.computes==1,"occupied endpoint mistaken for new evidence");
  require(f.graph()->FindMeasurementOwner(evidence)==added.factor.edge,"retained factor owner missing");
  auto empty=f.graph()->CommitLoopClosureCandidate({},f.query(),f.mean(),f.covariance(),.8,.9,f.mean());require(empty.factor.status==MapperGraph::LinkStatus::Rejected&&!empty.solve_attempted,"empty chain did not reject");
 }
 for(int mode=0;mode<9;++mode){MockSolver s;Fixture f(s,"mock_failure_"+std::to_string(mode));s.accept=mode!=0;s.throw_compute=mode==1;s.unknown_compute=mode==2;s.unknown_registration=mode==3;s.malformed=mode>=4?mode-3:0;
  auto pose=f.query()->GetCorrectedPose();auto old=f.graph()->GetEdges().size();auto a=f.submit(0);require(a.factor.added()&&!a.committed&&equal(pose,f.query()->GetCorrectedPose())&&f.graph()->GetEdges().size()==old+1,"failure did not retain factor with original state");require(s.computes==(mode==3?0:1),"unknown backend was solved");
  auto evidence=dynamic_cast<LinkInfo*>(a.factor.edge->GetLabel())->GetEvidence();require(f.graph()->FindMeasurementOwner(evidence)==a.factor.edge,"failed solve lost valid factor owner");s.accept=true;s.throw_compute=false;s.unknown_compute=false;s.unknown_registration=false;s.malformed=0;
  auto b=f.submit(1);require(b.factor.added()&&b.committed,"subsequent independent candidate did not recover");require(f.graph()->FindMeasurementOwner(evidence)==a.factor.edge,"recovery removed retained factor");
 }
 {MockSolver s;Fixture f(s,"registration_failure");s.throw_registration=true;const size_t edges=f.graph()->GetEdges().size(),adj=f.graph()->GetVertices().at(f.laser->GetName()).at(0)->GetEdges().size();auto result=f.submit(0);require(!result.factor.added()&&!result.solve_attempted&&f.graph()->GetEdges().size()==edges,"failed registration left graph factor");require(f.graph()->GetVertices().at(f.laser->GetName()).at(0)->GetEdges().size()==adj,"failed registration left adjacency");s.throw_registration=false;require(f.submit(0).committed,"recovery after failed registration");}
 {MockSolver s;Fixture f(s,"mandatory_registration_failure");s.throw_registration=true;
  auto * source=f.scans.back().get();const auto * readings=source->GetRangeReadings();
  RangeReadingsVector ranges(readings,readings+source->GetNumberOfRangeReadings());
  auto pending=std::make_unique<LocalizedRangeScan>(f.laser->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});
  pending->SetRawSensorLimits(.05,20);pending->SetTime(4);pending->SetOdometricPose(Pose2(.2,0,0));
  const auto edgeCount=f.graph()->GetEdges().size();bool caught=false;bool publishedSuccess=false;
  try{publishedSuccess=f.mapper.Process(pending.get());}catch(const std::runtime_error&){caught=true;}
  require(caught&&!publishedSuccess&&f.graph()->GetEdges().size()==edgeCount,"mandatory registration failure was silently published as a successful scan");
  // Propagated frontend errors are fail-fast, not a claim that the whole scan
  // transaction supports continuation after arbitrary partial insertion.
  f.mapper.Reset();
 }
 std::cout<<"{\"test\":\"native_mapper_lifecycle_mock\",\"passed\":true,\"barycenter_and_lever_arm_equivalent\":true,\"no_factor_no_solve\":true,\"retained_factor_no_state_commit\":true,\"unknown_plugins_fail_closed\":true,\"registration_cleanup\":true,\"mandatory_registration_fail_fast\":true}\n";
}
void unobservable_alias(){
 // Two translated identical rooms give the same local prepared scan and the
 // same local registration context. The hidden room identity is deliberately
 // absent from the estimator; candidate lifecycle cannot infer that identity.
 MockSolver solver;Fixture f(solver,"unobservable_repeated_room");
 std::unique_ptr<ScanMatcher> matcher(ScanMatcher::Create(&f.mapper,.4,.01,.03,8));
 Pose2 mean;Matrix3 covariance;LocalizedRangeScanVector refs{f.scans[0].get()};
 double response=matcher->MatchLidarEvidence(f.query(),refs,mean,covariance,true);
 require(response>=f.mapper.getParamLoopMatchMinimumResponseFine(),"identical observation did not pass native fine score");
 auto result=f.graph()->CommitLoopClosureCandidate(refs,f.query(),mean,covariance,.8,response,f.query()->GetSensorPose());
 require(result.factor.added()&&result.committed,"alias counterexample did not pass unchanged association/lifecycle");
 std::cout<<"{\"test\":\"native_unobservable_repeated_room\",\"passed\":true,\"native_fine_response\":"<<response<<",\"hidden_room_displacement_m\":20,\"observations_identical\":true,\"unchanged_gate_accepts\":true,\"physical_identity_not_observed\":true,\"association_correctness_established\":false,\"scope\":\"identifiability limitation counterexample, not false-association repair\"}\n";
}
struct Abort:ceres::IterationCallback{solver_plugins::CeresSolver&s;bool throws,called=false;Abort(solver_plugins::CeresSolver&v,bool t):s(v),throws(t){}ceres::CallbackReturnType operator()(const ceres::IterationSummary&i)override{if(i.iteration<1)return ceres::SOLVER_CONTINUE;called=true;s.nodes_->at(1)(0)=std::numeric_limits<double>::quiet_NaN();if(throws)throw std::runtime_error("injected callback exception");return ceres::SOLVER_ABORT;}};
void native_ceres(const std::string&out){for(bool throwing:{false,true}){
 solver_plugins::CeresSolver solver;rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("ceres_diagnostics_path",out+(throwing?"/exception.jsonl":"/abort.jsonl")),rclcpp::Parameter("ceres_diagnostics_state_mode","delta")});auto node=std::make_shared<rclcpp::Node>(throwing?"lifecycle_exception":"lifecycle_abort",options);node->declare_parameter("debug_logging",false);solver.Configure(node);
 Fixture f(solver,throwing?"native_exception":"native_abort");auto formal=f.query()->GetCorrectedPose();auto nodes=*solver.getGraph();Abort callback(solver,throwing);solver.options_.update_state_every_iteration=true;solver.options_.callbacks.push_back(&callback);auto failed=f.submit(0,.3);require(callback.called,"native callback not reached");require(failed.factor.added()&&failed.factor.backend_registered&&!failed.committed&&failed.decision=="factor_retained_solve_rejected","native rejection lifecycle wrong");require(equal(formal,f.query()->GetCorrectedPose()),"native failure changed mapper state");for(const auto&e:nodes)require((solver.getGraph()->at(e.first).array()==e.second.array()).all(),"native failure did not restore exact solver state");require(solver.GetCorrections().empty(),"native failure left stale corrections");auto evidence=dynamic_cast<LinkInfo*>(failed.factor.edge->GetLabel())->GetEvidence();require(f.graph()->FindMeasurementOwner(evidence)==failed.factor.edge,"native rejection lost evidence owner");
 solver.options_.callbacks.clear();auto recovered=f.submit(1,.2);require(recovered.committed&&solver.GetComputeState()==ScanSolver::ComputeState::Accepted,"native subsequent candidate recovery failed");require(f.graph()->FindMeasurementOwner(evidence)==failed.factor.edge,"native recovery lost retained factor");
 // A duplicate registration must preserve the pre-existing residual.
 const auto oldCount=solver.problem_->NumResidualBlocks();bool duplicateCaught=false;
 try{solver.AddConstraint(recovered.factor.edge);}catch(const std::invalid_argument&){duplicateCaught=true;}
 require(duplicateCaught&&solver.problem_->NumResidualBlocks()==oldCount&&
   solver.GetConstraintRegistration(recovered.factor.edge->GetSource()->GetObject()->GetUniqueId(),f.query()->GetUniqueId())==ScanSolver::ConstraintRegistration::Registered,
   "duplicate registration removed an existing residual");
 // Throw after AddResidualBlock/map insertion by explicitly making the optional
 // diagnostic stream throw. No testing branch/hook enters production.
 auto * sourceVertex=f.graph()->GetVertices().at(f.laser->GetName()).at(2);
 auto * targetVertex=f.graph()->GetVertices().at(f.laser->GetName()).at(5);
 Edge<LocalizedRangeScan> failingEdge(sourceVertex,targetVertex);
 failingEdge.SetLabel(new LinkInfo(Pose2(),Pose2(.4,0,.1),f.covariance()));
 solver.diagnostics_.setstate(std::ios::badbit);
 try{solver.diagnostics_.exceptions(std::ios::badbit);}catch(const std::ios_base::failure&){}
 bool setupCaught=false;try{solver.AddConstraint(&failingEdge);}catch(const std::ios_base::failure&){setupCaught=true;}
 solver.diagnostics_.exceptions(std::ios::goodbit);solver.diagnostics_.clear();
 require(setupCaught&&solver.problem_->NumResidualBlocks()==oldCount&&
   solver.GetConstraintRegistration(2,5)==ScanSolver::ConstraintRegistration::Rejected,
   "post-registration exception retained a new residual");
 // Checked invalid registration does not mutate Ceres residual bookkeeping.
 const auto count=solver.problem_->NumResidualBlocks();LocalizedRangeScan rogue(Name("missing"),{},LocalizedRangeScan::LegacyRawTag{});rogue.SetUniqueId(99999);Vertex<LocalizedRangeScan> rogueVertex(&rogue);Edge<LocalizedRangeScan> edge(f.graph()->GetVertices().at(f.laser->GetName()).at(0),&rogueVertex);auto * info=new LinkInfo(Pose2(),Pose2(1,0,0),f.covariance());edge.SetLabel(info);bool caught=false;try{solver.AddConstraint(&edge);}catch(const std::invalid_argument&){caught=true;}require(caught&&solver.problem_->NumResidualBlocks()==count,"missing endpoint registration not atomic");
 }
 std::cout<<"{\"test\":\"native_ceres_mapper_lifecycle\",\"passed\":true,\"abort_and_exception_exact_rollback\":true,\"factor_retained\":true,\"subsequent_candidate_recovery\":true,\"missing_endpoint_rejected\":true,\"duplicate_preserves_old_residual\":true,\"post_registration_exception_removes_new_residual\":true}\n";
}
int main(int argc,char**argv){rclcpp::init(argc,argv);try{require(argc==2,"output directory argument");mock_cases();unobservable_alias();native_ceres(argv[1]);}catch(const Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';rclcpp::shutdown();return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
