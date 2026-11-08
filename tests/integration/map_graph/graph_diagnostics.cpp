#include "solvers/ceres_solver.hpp"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <limits>
#include <fstream>
#include <sstream>
#include <memory>
#include <iostream>
#include <stdexcept>
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
int main(int argc,char ** argv){rclcpp::init(argc,argv);try{require(argc>1,"output dir required");std::string dir=argv[1];double plain=0,robust=0,legacy=0,explicit_default=0;{Fixture f("None",dir+"/none.jsonl");plain=f.run();}{Fixture f("HuberLoss",dir+"/huber.jsonl",.7);robust=f.run();f.solver.RemoveConstraint(0,2);f.solver.Compute();require(std::abs(f.solver.getGraph()->at(2)(0)-2)<.005,"removed bad edge still active");f.solver.RemoveNode(1);f.solver.Compute();f.solver.Reset();}{Fixture f("HuberLoss","",.7,false);legacy=f.run();}{Fixture f("HuberLoss","",.7,true);explicit_default=f.run();}require(std::abs(robust-2)<std::abs(plain-2),"robust kernel did not bound injected outlier influence");require(legacy==explicit_default,"default .7 no longer reproduces explicit .7");
{Fixture f("None","",2.1);require(std::abs(f.run()-plain)<1e-10,"loss scale changed ordinary squared loss");}
{Fixture f("HuberLoss","",2.1);require(std::abs(f.run()-2)>std::abs(robust-2),"configurable loss scale has no effect");}
{Fixture f("CauchyLoss",dir+"/cauchy.jsonl",.7);require(std::abs(f.run()-2)<std::abs(plain-2),"Cauchy loss did not bound outlier influence");}
{Fixture f("None",dir+"/delta.jsonl",.7,true,"delta");require(std::abs(f.run()-plain)<1e-10,"delta diagnostics changed solver state");f.edge(2,0,-2,"near_chain");f.solver.Compute();f.solver.RemoveConstraint(2,0);f.solver.Compute();bool rejected=false;try{f.solver.AddConstraint(f.edges.front().get());}catch(const std::invalid_argument &){rejected=true;}require(rejected,"duplicate directed constraint was silently accepted");f.solver.RemoveNode(1);f.solver.Compute();}
for(double scale:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){bool rejected=false;try{Fixture f("None","",scale);}catch(const std::invalid_argument &){rejected=true;}require(rejected,"invalid loss scale accepted");}
  karto::Matrix3 cov;cov.SetToIdentity();karto::LinkInfo before(karto::Pose2(),karto::Pose2(1,2,.3),cov);before.SetProvenance("near_chain",{5,7,9},.8,.9,karto::Pose2(2,3,.4));std::stringstream stream;{boost::archive::binary_oarchive out(stream);out<<before;}karto::LinkInfo after;{boost::archive::binary_iarchive in(stream);in>>after;}require(after.GetSource()=="near_chain" && after.GetChain()==std::vector<int>({5,7,9}) && after.GetFineResponse()==.9,"constraint provenance serialization changed");
  std::cout<<"invalid_scales=4; default_scale_reproduces_legacy=passed; outlier_influence=passed; remove_reset=passed; provenance_roundtrip=passed; plain_node2="<<plain<<"; robust_node2="<<robust<<'\n';}catch(const std::exception & e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();}
