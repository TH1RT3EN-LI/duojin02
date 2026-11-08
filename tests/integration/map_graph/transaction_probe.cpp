// Native solver transaction fault injection. No testing branches/hooks enter production code.
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
#include "karto_sdk/Mapper.h"
#include "solvers/ceres_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "slam_toolbox/toolbox_types.hpp"
#define private public
#include "solvers/ceres_solver.hpp"
#undef private
void require(bool ok,const char * s){if(!ok)throw std::runtime_error(s);}
struct Fixture {
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> scans;
  std::vector<std::unique_ptr<karto::Vertex<karto::LocalizedRangeScan>>> vertices;
  std::vector<std::unique_ptr<karto::Edge<karto::LocalizedRangeScan>>> edges;
  solver_plugins::CeresSolver solver;
  Fixture(const std::string & loss,const std::string & path,double scale=.7,bool explicit_scale=true,const std::string & mode="full") {
    rclcpp::NodeOptions options;std::vector<rclcpp::Parameter> params={rclcpp::Parameter("ceres_loss_function",loss),rclcpp::Parameter("ceres_diagnostics_path",path),rclcpp::Parameter("ceres_diagnostics_state_mode",mode)};if(explicit_scale)params.emplace_back("ceres_loss_scale",scale);options.parameter_overrides(params);auto node=std::make_shared<rclcpp::Node>("graph_probe",options);node->declare_parameter("debug_logging",false);solver.Configure(node);
    for(int i=0;i<3;++i){auto scan=std::make_unique<karto::LocalizedRangeScan>(karto::Name("probe"),karto::RangeReadingsVector{},karto::LocalizedRangeScan::LegacyRawTag{});scan->SetUniqueId(i);scan->SetTime(i*.1);scan->SetCorrectedPose(karto::Pose2(i,0,0));auto v=std::make_unique<karto::Vertex<karto::LocalizedRangeScan>>(scan.get());solver.AddNode(v.get());scans.push_back(std::move(scan));vertices.push_back(std::move(v));}
  }
  void edge(int i,int j,double x,const std::string & source){karto::Matrix3 covariance;covariance.SetToIdentity();covariance(0,0)=.01;covariance(1,1)=.02;covariance(2,2)=.03;covariance(0,1)=covariance(1,0)=.003;auto e=std::make_unique<karto::Edge<karto::LocalizedRangeScan>>(vertices[i].get(),vertices[j].get());auto * info=new karto::LinkInfo(karto::Pose2(),karto::Pose2(x,0,0),covariance);info->SetProvenance(source,{0,1,2},.81,.92,karto::Pose2(j,0,0));e->SetLabel(info);solver.AddConstraint(e.get());edges.push_back(std::move(e));}
  double run(){edge(0,1,1,"sequential");edge(1,2,1,"running_chain");edge(0,2,20,"explicit_loop");solver.Compute();return solver.getGraph()->at(2)(0);}
};
struct AbortAfterMutation:ceres::IterationCallback {
 solver_plugins::CeresSolver &solver;bool throwing;bool mutated=false;
 AbortAfterMutation(solver_plugins::CeresSolver&s,bool t):solver(s),throwing(t){}
 ceres::CallbackReturnType operator()(const ceres::IterationSummary&s)override {
  if(s.iteration<1)return ceres::SOLVER_CONTINUE;
  solver.nodes_->at(1)(0)=std::numeric_limits<double>::quiet_NaN();mutated=true;
  if(throwing)throw std::runtime_error("injected solver callback exception");
  return ceres::SOLVER_ABORT;
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{require(argc==2,"output dir");std::string dir=argv[1];
 for(bool throwing:{false,true}){
  Fixture f("None",dir+(throwing?"/exception.jsonl":"/aborted.jsonl"),.7,true,"delta");f.run();require(!f.solver.GetCorrections().empty(),"successful corrections missing");
  auto warm=f.solver.GetLastSolveResult();require(warm.accepted,"normal solve not accepted");require(warm.before.valid&&warm.candidate.valid,"independent evaluate absent");
  f.solver.ModifyNode(1,Eigen::Vector3d(6,.4,.2));auto before=*f.solver.getGraph();AbortAfterMutation abort(f.solver,throwing);f.solver.options_.update_state_every_iteration=true;f.solver.options_.callbacks.push_back(&abort);f.solver.Compute();require(abort.mutated,"fault callback never invoked");require(!f.solver.GetLastSolveResult().accepted,"aborted solve accepted");require(f.solver.GetCorrections().empty(),"old corrections leaked");for(const auto&e:before)require((f.solver.getGraph()->at(e.first).array()==e.second.array()).all(),"failed solve did not restore exact node values");
  f.solver.options_.callbacks.clear();f.solver.Compute();require(f.solver.GetLastSolveResult().accepted,"subsequent valid solve failed; parameter addresses invalidated by rollback");
 }
 {Fixture f("None",dir+"/budget.jsonl",.7,true,"delta");f.run();f.solver.ModifyNode(1,Eigen::Vector3d(6,.4,.2));f.solver.options_.max_num_iterations=0;f.solver.Compute();const auto &r=f.solver.GetLastSolveResult();require(r.accepted&&!r.free_gradient_tolerance_met&&r.native_summary.termination_type==ceres::NO_CONVERGENCE,"budget exhaustion hidden as converged or usable valid state rejected");require(r.decision=="accepted_not_stationary","budget state not explicitly labelled nonstationary");}
 {Fixture f("None",dir+"/invalid-initial.jsonl",.7,true,"delta");f.run();f.solver.ModifyNode(1,Eigen::Vector3d(std::numeric_limits<double>::infinity(),0,0));f.solver.Compute();require(!f.solver.GetLastSolveResult().accepted&&!f.solver.GetLastSolveResult().attempted,"invalid initial objective reached solver");require(f.solver.GetCorrections().empty(),"invalid initial state leaked stale corrections");}
 {Fixture f("None",dir+"/empty.jsonl",.7,true,"delta");f.run();f.solver.Reset();f.solver.Compute();require(f.solver.GetCorrections().empty()&&f.solver.GetLastSolveResult().decision=="no_nodes","empty graph leaked correction/result");}
 std::cout<<"transaction rollback=user abort+exception; exact values+stable parameter addresses=recovered; stale corrections=cleared; budget=accepted-but-nonstationary; invalid initial=pre-solve rejected; empty graph=cleared; passed\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
