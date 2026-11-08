#include <filesystem>
#include <fstream>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include "rclcpp/executors/single_threaded_executor.hpp"
extern char ** environ;
#include <stdexcept>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/serialization.hpp"
#include "solvers/ceres_solver.hpp"
using namespace karto;
static void require(bool b, const char * message) {if (!b) throw std::runtime_error(message);}
static std::string state(const ScanSolver::BackendSnapshot & s) {
  std::ostringstream out; out << std::setprecision(17) << s.known << ':' << s.anchor_valid << ':'
    << s.epoch << ':' << s.topology_revision << ':' << s.pose_revision << ':' << s.anchor_id << ':'
    << s.anchor_pose.GetX() << ':' << s.anchor_pose.GetY() << ':' << s.anchor_pose.GetHeading() << ':' << s.objective_model << s.solver_options_complete << s.solver_options;
  for (const auto & n:s.nodes) out << '\n' << n.first << ':' << n.second.GetX() << ':' << n.second.GetY() << ':' << n.second.GetHeading();
  for (const auto & e:s.constraints) {out << '\n' << e.source_id << ':' << e.target_id << ':' << e.registration_id << ':' << e.mean.GetX() << ':' << e.mean.GetY() << ':' << e.mean.GetHeading();for(int i=0;i<3;++i)for(int j=0;j<3;++j)out << ':' << e.covariance(i,j);}
  return out.str();
}
static int nativeSolves(const std::filesystem::path & path) {
  std::ifstream file(path);std::string line;int count=0;
  while(std::getline(file,line))if(line.find("\"event\":\"optimization_begin\"")!=std::string::npos)++count;
  return count;
}
class KartoFailingFactory : public solver_plugins::CeresSolver {
public:
  std::unique_ptr<karto::ScanSolver> CreateStagingSolver() override {
    throw karto::Exception("injected non-std staging factory failure");
  }
};
// Faults enter through the real virtual diagnostic sink, with no production
// branch or altered numerical backend implementation.
class DiagnosticFailingSolver : public solver_plugins::CeresSolver {
public:
  std::string fail_phase, fail_decision;
  int injections{0};
  void RecordMatchDiagnostic(const std::string & json) override {
    if (!fail_phase.empty()) {
      boost::property_tree::ptree event;
      std::istringstream input(json);boost::property_tree::read_json(input,event);
      if (event.get<std::string>("stage","")=="graph_update" &&
        event.get<std::string>("phase","")==fail_phase &&
        (fail_decision.empty() || event.get<std::string>("decision","")==fail_decision))
      {
        fail_phase.clear();++injections;
        throw std::runtime_error("injected graph update result diagnostic failure");
      }
    }
    solver_plugins::CeresSolver::RecordMatchDiagnostic(json);
  }
};
class Probe : public slam_toolbox::SlamToolbox {
public:
  explicit Probe(const rclcpp::NodeOptions & options):SlamToolbox(options) {}
  void setup(bool fail_factory=false) {
    setParams(); setROSInterfaces();
    if(fail_factory)solver_=std::make_shared<KartoFailingFactory>();
    else solver_=std::make_shared<DiagnosticFailingSolver>();
    solver_->Configure(shared_from_this());
    smapper_->getMapper()->SetScanSolver(solver_.get());
    scan_holder_=std::make_unique<laser_utils::ScanHolder>(lasers_);
    closure_assistant_=std::make_unique<loop_closure_assistant::LoopClosureAssistant>(shared_from_this(),smapper_->getMapper(),scan_holder_.get(),state_,processor_type_,smapper_mutex_);
  }
  void seed(const std::string & name="snapshot_ros_laser", int count=6) {
    auto * laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name(name));
    laser->SetMinimumRange(.05);laser->SetMaximumRange(20);laser->SetRangeThreshold(8);laser->SetMinimumAngle(-1.5);laser->SetMaximumAngle(1.5);laser->SetAngularResolution(.02);laser->SetOffsetPose(Pose2(.305,.03,.07));
    dataset_->Add(laser);lasers_[name]=laser_utils::LaserMetadata(laser,false);
    auto * mapper=smapper_->getMapper();mapper->setParamDoLoopClosing(false);mapper->setParamMinimumTravelDistance(0);mapper->setParamMinimumTravelHeading(0);mapper->setParamMinimumTimeInterval(0);
    for(int k=0;k<count;++k) {
      RangeReadingsVector ranges;
      for(int i=0;i<151;++i){double a=-1.5+.02*i;ranges.push_back(3.5+.7*std::sin(3*a)+.4*std::cos(7*a));}
      auto * scan=new LocalizedRangeScan(laser->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});
      scan->SetRawSensorLimits(.05,20);scan->SetTime(1.+.1*k);scan->SetOdometricPose(Pose2(.02*k,0,.01*k));
      require(mapper->Process(scan),"seed Process rejected");dataset_->Add(scan);
    }
    sensor_msgs::msg::LaserScan old; old.header.frame_id="stale-before-load";scan_holder_->addScan(0,old);
  }
  ScanSolver::BackendSnapshot backend() {return solver_->GetBackendSnapshot();}
  std::string metadata() {return smapper_->getMapper()->GetGraphUpdateMetadataJson();}
  std::string references() {std::ostringstream out;out << std::setprecision(17);for(auto * scan:smapper_->getMapper()->GetAllProcessedScans()){auto p=scan->GetCorrectedPose();out<<scan->GetUniqueId()<<':'<<p.GetX()<<':'<<p.GetY()<<':'<<p.GetHeading()<<'\n';}return out.str();}
  void failDiagnostic(const std::string & phase, const std::string & decision="") {
    auto target=std::dynamic_pointer_cast<DiagnosticFailingSolver>(solver_);
    require(bool(target),"Diagnostic fixture is not active");target->fail_phase=phase;target->fail_decision=decision;
  }
  int diagnosticInjections() {return std::dynamic_pointer_cast<DiagnosticFailingSolver>(solver_)->injections;}
  bool jointConsistent() {
    auto owner=smapper_->getMapper()->LockGraphState();std::map<int,Pose2> poses;
    for(auto * scan:smapper_->getMapper()->GetAllProcessedScans())poses.emplace(scan->GetUniqueId(),scan->GetCorrectedPose());
    const auto actual=backend();if(poses.size()!=actual.nodes.size())return false;
    for(const auto & node:actual.nodes){const auto found=poses.find(node.first);if(found==poses.end()||found->second!=node.second)return false;}
    return true;
  }
  bool faulted() {return smapper_->getMapper()->GetGraph()->GetUpdateState().faulted;}
  uint64_t graphRevision() {return smapper_->getMapper()->GetGraph()->GetUpdateState().graph_revision;}
  void changeOnlyReference() {
    auto * mapper=smapper_->getMapper();auto owner=mapper->LockGraphState();
    auto scans=mapper->GetAllProcessedScans();require(!scans.empty(),"Missing reference mutation fixture");
    auto * scan=scans.back();const auto pose=scan->GetCorrectedPose();
    scan->SetCorrectedPoseAndUpdate(Pose2(pose.GetX()+.125,pose.GetY(),pose.GetHeading()));
    mapper->GetGraph()->ObserveReferenceChange("explicit_reference_edit",scan);
  }
  void changeOnlyBackend() {
    const auto snapshot=backend();require(snapshot.nodes.size()>1,"Missing backend mutation fixture");const auto & node=snapshot.nodes.back();
    smapper_->getMapper()->ModifyGraphNode(node.first,Eigen::Vector3d(node.second.GetX()+.2,node.second.GetY(),0.));
  }
  const void * mapperIdentity() {return smapper_->getMapper();}
  const void * solverIdentity() {return solver_.get();}
  std::shared_ptr<slam_toolbox::srv::SaveSnapshot::Response> save(const std::filesystem::path & directory, bool finalize=false, std::string id="") {
    auto request=std::make_shared<slam_toolbox::srv::SaveSnapshot::Request>();auto response=std::make_shared<slam_toolbox::srv::SaveSnapshot::Response>();
    request->filename=directory.string();request->include_graph=true;request->mode=finalize?request->FINALIZE:request->SNAPSHOT;request->request_id=id;
    saveSnapshotCallback(nullptr,request,response);return response;
  }
  bool load(const std::filesystem::path & directory) {
    auto request=std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();auto response=std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();request->filename=directory.string();request->match_type=request->START_AT_FIRST_NODE;
    return deserializePoseGraphCallback(nullptr,request,response);
  }
  void pause() {state_.set(toolbox_types::NEW_MEASUREMENTS,true);}
  void checkPausedFirstMeasurement() {
    sensor_msgs::msg::LaserScan scan;scan.header.stamp.sec=2;scan.range_min=.05f;scan.range_max=20.f;
    scan.angle_min=0.f;scan.angle_max=.1f;scan.angle_increment=.1f;scan.time_increment=.001f;scan.scan_time=.002f;scan.ranges={1.f,1.f};
    auto source=std::make_shared<sensor_msgs::msg::LaserScan>(scan);
    first_measurement_=true;state_.set(toolbox_types::NEW_MEASUREMENTS,true);
    require(!shouldProcessScan(source,Pose2())&&first_measurement_,"Paused first measurement bypassed admission boundary");
    state_.set(toolbox_types::NEW_MEASUREMENTS,false);
    require(shouldProcessScan(source,Pose2())&&!first_measurement_,"Unpaused first measurement no longer eligible");
  }
  void checkKeyedHolder() {
    sensor_msgs::msg::LaserScan raw;raw.header.frame_id="holder-test";raw.ranges={1.25f};
    lasers_[raw.header.frame_id]=laser_utils::LaserMetadata(nullptr,false);
    scan_holder_->clear();scan_holder_->addScan(901,raw);
    require(scan_holder_->getCorrectedScan(901).ranges==raw.ranges,"Non-contiguous graph ID was not retained");
    bool unknown=false;try{scan_holder_->getCorrectedScan(0);}catch(const std::out_of_range&){unknown=true;}
    require(unknown,"Unknown graph ID returned unrelated ROS scan");scan_holder_->clear();lasers_.clear();
  }
  void inputBusy(bool value) {busy_=value;}
  bool holderEmpty() {try{scan_holder_->getCorrectedScan(0);return false;}catch(const std::out_of_range &){return true;}}
  void cleanup() {smapper_.reset();dataset_.reset();closure_assistant_.reset();solver_.reset();}
protected:
  void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr, toolbox_types::GeometryPtr) override {}
  bool snapshotInputIdle() const override {return !busy_;}
private: bool busy_{false};
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    require(argc==2,"output directory argument required");std::filesystem::path output(argv[1]);std::filesystem::create_directories(output);
    rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("ceres_diagnostics_path",(output/"backend.jsonl").string()),rclcpp::Parameter("ceres_diagnostics_state_mode","full"),rclcpp::Parameter("graph_update_policy","loop_only")});
    auto probe=std::make_shared<Probe>(opts);probe->setup();probe->checkPausedFirstMeasurement();probe->checkKeyedHolder();probe->seed();
    // Deliberately preserve the real pre-CI backend/mapper difference in F0.
    const auto original=state(probe->backend()), reference=probe->references(), metadata=probe->metadata();
    const int before_solves=nativeSolves(output/"backend.jsonl");
    auto first=probe->save(output/"f0");require(first->result==first->RESULT_SUCCESS&&!first->finalized,"Snapshot failed");
    require(original==state(probe->backend())&&reference==probe->references()&&metadata==probe->metadata(),"Snapshot requested an update or changed state");
    require(nativeSolves(output/"backend.jsonl")==before_solves,"Snapshot called native numerical solver");
    require(probe->save(output/"f0")->result!=first->RESULT_SUCCESS,"Snapshot overwrote immutable destination");
    probe->inputBusy(true);probe->pause();
    require(probe->save(output/"busy-finalize",true,"busy-id")->result!=first->RESULT_SUCCESS,"Finalize ignored in-flight input");
    require(!std::filesystem::exists(output/"busy-finalize"),"Rejected Finalize published bundle");
    require(!probe->load(output/"f0")&&original==state(probe->backend()),"Load ignored busy input");
    auto busy_snapshot=probe->save(output/"busy-snapshot");
    require(busy_snapshot->result==busy_snapshot->RESULT_SUCCESS,"Pure Snapshot improperly required drained input");
    boost::property_tree::ptree busy_boundary;boost::property_tree::read_json((output/"busy-snapshot"/"input_boundary.json").string(),busy_boundary);
    require(!busy_boundary.get<bool>("queues_drained"),"Snapshot concealed unfinished input queue");
    probe->inputBusy(false);
    // Both cryptographic refusal and archive parsing failure retain the live graph.
    std::filesystem::copy(output/"f0",output/"bad-hash",std::filesystem::copy_options::recursive);
    std::ofstream(output/"bad-hash"/"graph.data",std::ios::app)<<"changed";
    const auto identity=probe->mapperIdentity(), solver=probe->solverIdentity();
    require(!probe->load(output/"bad-hash")&&probe->mapperIdentity()==identity&&probe->solverIdentity()==solver&&original==state(probe->backend()),"Bad hash damaged live graph");
    // A valid manifest around invalid archive exercises staged archive disposal.
    {
      serialization::BundleWriter invalid(output/"bad-archive");
      for(const auto& file:std::filesystem::directory_iterator(output/"f0"))if(file.path().filename()!="snapshot.json")std::filesystem::copy_file(file.path(),invalid.path()/file.path().filename());
      serialization::writeText(invalid.path()/"graph.data","not a boost archive");
      for(const auto& item:std::vector<std::pair<std::string,std::string>>{{"yaml","map.yaml"},{"image","map.pgm"},{"posegraph","graph.posegraph"},{"data","graph.data"},{"graph_update","graph_update.json"},{"scan_geometry","scan_geometry.json"},{"input_boundary","input_boundary.json"},{"runtime","runtime.json"},{"laser_bindings","laser_bindings.json"}})invalid.add(item.first,item.second);
      invalid.publish(false,"");
    }
    require(!probe->load(output/"bad-archive")&&probe->mapperIdentity()==identity&&probe->solverIdentity()==solver&&original==state(probe->backend()),"Invalid archive damaged live graph");
    {
      auto nonstd=std::make_shared<Probe>(rclcpp::NodeOptions());nonstd->setup(true);
      const auto old=state(nonstd->backend());const auto * identity=nonstd->mapperIdentity();
      require(!nonstd->load(output/"f0")&&state(nonstd->backend())==old&&nonstd->mapperIdentity()==identity,"Non-std Karto exception escaped the staged load failure boundary");
      nonstd->cleanup();nonstd.reset();
    }
    {
      rclcpp::NodeOptions strict_options;strict_options.parameter_overrides({rclcpp::Parameter("ceres_solver_profile","strict")});
      auto incompatible=std::make_shared<Probe>(strict_options);incompatible->setup();
      const auto old=state(incompatible->backend());const auto * identity=incompatible->mapperIdentity();
      require(!incompatible->load(output/"f0")&&state(incompatible->backend())==old&&incompatible->mapperIdentity()==identity,"Profile-incompatible load replaced live state");
      incompatible->cleanup();incompatible.reset();
    }
    {
      const auto active_directory=output/"activation-path";std::filesystem::create_directory(active_directory);
      rclcpp::NodeOptions activation_options;activation_options.parameter_overrides({rclcpp::Parameter("ceres_diagnostics_path",(active_directory/"backend.jsonl").string())});
      auto inactive=std::make_shared<Probe>(activation_options);inactive->setup();
      const auto old=state(inactive->backend());const auto * identity=inactive->mapperIdentity();
      std::filesystem::rename(active_directory,output/"activation-path-retained");
      require(!inactive->load(output/"f0")&&state(inactive->backend())==old&&inactive->mapperIdentity()==identity,"Diagnostic activation failure replaced live state");
      inactive->cleanup();inactive.reset();
    }
    {
      auto unconstrained=std::make_shared<Probe>(rclcpp::NodeOptions());unconstrained->setup();unconstrained->seed("snapshot_unconstrained",1);unconstrained->pause();
      auto retained=unconstrained->save(output/"unconstrained-f0");require(retained->result==retained->RESULT_SUCCESS,"Single-node Snapshot failed");
      const auto old=state(unconstrained->backend());const auto previous=serialization::sha256File(output/"unconstrained-f0"/"snapshot.json");
      auto rejected=unconstrained->save(output/"unconstrained-f1",true,"unconstrained-once");
      require(rejected->result!=rejected->RESULT_SUCCESS&&!rejected->finalized&&!rejected->state_committed&&old==state(unconstrained->backend()),"Unavailable Finalize candidate was published as success");
      require(!std::filesystem::exists(output/"unconstrained-f1")&&previous==serialization::sha256File(output/"unconstrained-f0"/"snapshot.json"),"Finalize failure changed retained F0");
      unconstrained->cleanup();unconstrained.reset();
    }
    require(probe->load(output/"f0"),"Pure load failed");
    require(original==state(probe->backend())&&reference==probe->references()&&metadata==probe->metadata(),"Pure load changed backend/reference/version checkpoint");
    require(probe->holderEmpty(),"Load retained stale scan-ID cache");
    require(probe->load(output/"f0")&&original==state(probe->backend()),"Repeated same-name pure load failed");
    require(nativeSolves(output/"backend.jsonl")==before_solves,"Pure Load called native numerical solver");
    auto reloaded=probe->save(output/"f0-reloaded");require(reloaded->result==reloaded->RESULT_SUCCESS,"Reloaded snapshot failed");
    require(serialization::sha256File(output/"f0"/"map.pgm")==serialization::sha256File(output/"f0-reloaded"/"map.pgm")&&
      serialization::sha256File(output/"f0"/"map.yaml")==serialization::sha256File(output/"f0-reloaded"/"map.yaml"),"Pure load changed native raster/cache rendering");
    // Induce a real post-commit publication failure, without a production hook:
    // create the target after BundleWriter has reserved its private sibling.
    std::atomic<bool> stop{false}, collided{false};
    std::thread collision([&] {
      while(!stop.load()) {
        for(const auto & entry:std::filesystem::directory_iterator(output)) {
          if(entry.path().filename().string().find(".f1-io-failure.tmp-")==0) {
            std::filesystem::create_directory(output/"f1-io-failure");collided=true;return;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    auto failed=probe->save(output/"f1-io-failure",true,"one-finalize-budget");
    stop=true;collision.join();
    require(collided&&failed->result!=failed->RESULT_SUCCESS&&!failed->finalized&&failed->state_committed&&failed->commit_id>0,"I/O failure hid committed native state");
    require(!std::filesystem::exists(output/"f1-io-failure"/"snapshot.json"),"I/O failure published a partial bundle");
    std::ofstream(output/"io-failure-attempt.json") << "{\"request_id\":\"one-finalize-budget\",\"result\":" << int(failed->result)
      << ",\"state_committed\":true,\"finalized\":false,\"commit_id\":" << failed->commit_id
      << ",\"message\":" << serialization::jsonString(failed->message) << "}\n";
    const auto finalized=state(probe->backend());
    const int after_finalize=nativeSolves(output/"backend.jsonl");
    require(after_finalize==before_solves+1,"Finalize did not spend exactly one native solve");
    auto final=probe->save(output/"f1",true,"one-finalize-budget");
    require(final->result==final->RESULT_SUCCESS&&final->finalized&&final->state_committed&&state(probe->backend())==finalized,"Retry repeated an accepted native Finalize");
    auto reused=probe->save(output/"f1-reused",true,"one-finalize-budget");
    require(reused->result==reused->RESULT_SUCCESS&&reused->finalized&&finalized==state(probe->backend()),"Repeated Finalize spent another numerical budget");
    require(nativeSolves(output/"backend.jsonl")==after_finalize,"Repeated request_id invoked another native solve");
    const auto failureOptions=[&](const std::string & label) {
      rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("ceres_diagnostics_path",(output/(label+".jsonl")).string()),rclcpp::Parameter("graph_update_policy","loop_only")});return options;
    };
    const auto recordFailure=[&](const std::string & label,const auto & response,bool consistent,bool faulted,int solves,const std::string & request_id="") {
      std::ofstream(output/(label+"-attempt.json")) << "{\"request_id\":" << serialization::jsonString(request_id.empty()?label:request_id)
        << ",\"result\":" << int(response->result) << ",\"state_committed\":" << (response->state_committed?"true":"false")
        << ",\"finalized\":" << (response->finalized?"true":"false") << ",\"commit_id\":" << response->commit_id
        << ",\"joint_consistent\":" << (consistent?"true":"false") << ",\"faulted\":" << (faulted?"true":"false")
        << ",\"native_solves\":" << solves << ",\"message\":" << serialization::jsonString(response->message) << "}\n";
    };
    {
      const std::string label="commit-log-failure";auto broken=std::make_shared<Probe>(failureOptions(label));broken->setup();broken->seed(label);broken->pause();
      auto f0=broken->save(output/(label+"-f0"));require(f0->result==f0->RESULT_SUCCESS,"Commit-log fixture Snapshot failed");
      const auto retained=serialization::sha256File(output/(label+"-f0")/"snapshot.json");
      broken->failDiagnostic("result");auto failed=broken->save(output/(label+"-f1"),true,label);
      recordFailure(label,failed,broken->jointConsistent(),broken->faulted(),nativeSolves(output/(label+".jsonl")));
      require(broken->diagnosticInjections()==1&&failed->result==failed->RESULT_FAILED&&!failed->finalized&&failed->state_committed&&failed->commit_id>0,"Post-commit diagnostic failure hid committed state");
      require(broken->jointConsistent()&&broken->faulted(),"Post-commit diagnostic failure did not preserve consistent faulted state");
      require(nativeSolves(output/(label+".jsonl"))==1&&!std::filesystem::exists(output/(label+"-f1"))&&retained==serialization::sha256File(output/(label+"-f0")/"snapshot.json"),"Commit-log failure changed F0, published F1, or repeated solve");
      broken->cleanup();broken.reset();
    }
    {
      const std::string label="idempotent-log-failure";auto replay=std::make_shared<Probe>(failureOptions(label));replay->setup();replay->seed(label);replay->pause();
      auto f0=replay->save(output/(label+"-f0"));require(f0->result==f0->RESULT_SUCCESS,"Replay-log fixture Snapshot failed");
      const auto retained=serialization::sha256File(output/(label+"-f0")/"snapshot.json");
      auto committed=replay->save(output/(label+"-committed"),true,label);require(committed->result==committed->RESULT_SUCCESS&&committed->commit_id>0,"Replay-log fixture Finalize failed");
      const auto backend=state(replay->backend()),references=replay->references();const auto graph_revision=replay->graphRevision();
      replay->failDiagnostic("idempotent_result");auto failed=replay->save(output/(label+"-f1"),true,label);
      recordFailure(label,failed,replay->jointConsistent(),replay->faulted(),nativeSolves(output/(label+".jsonl")));
      require(replay->diagnosticInjections()==1&&failed->result==failed->RESULT_FAILED&&!failed->finalized&&failed->state_committed&&failed->commit_id==committed->commit_id,"Same-ID diagnostic failure lost its existing committed result");
      require(!replay->faulted()&&replay->jointConsistent()&&backend==state(replay->backend())&&references==replay->references()&&nativeSolves(output/(label+".jsonl"))==1,"Same-ID diagnostic failure changed numerical state or budget");
      require(!std::filesystem::exists(output/(label+"-f1"))&&retained==serialization::sha256File(output/(label+"-f0")/"snapshot.json"),"Same-ID diagnostic failure changed F0 or published F1");
      // Same graph/coverage is insufficient: the reference version can change
      // independently of factors and of backend warm-start values.
      replay->changeOnlyReference();const auto edited=replay->references();
      require(replay->graphRevision()==graph_revision&&backend==state(replay->backend()),"Reference-only control changed graph/backend");
      replay->failDiagnostic("result","finalize_request_id_stale");auto stale=replay->save(output/"reference-stale-f1",true,label);
      recordFailure("reference-stale",stale,replay->jointConsistent(),replay->faulted(),nativeSolves(output/(label+".jsonl")),label);
      require(replay->diagnosticInjections()==2&&stale->result==stale->RESULT_FAILED&&!stale->finalized&&!stale->state_committed&&stale->commit_id==0,"Reference-stale request inherited an unrelated old commit");
      require(!replay->faulted()&&backend==state(replay->backend())&&edited==replay->references()&&nativeSolves(output/(label+".jsonl"))==1&&!std::filesystem::exists(output/"reference-stale-f1")&&retained==serialization::sha256File(output/(label+"-f0")/"snapshot.json"),"Reference-stale rejection changed state, F0, or numerical budget");
      replay->cleanup();replay.reset();
    }
    {
      const std::string label="backend-stale";auto edited=std::make_shared<Probe>(failureOptions(label));edited->setup();edited->seed(label);edited->pause();
      auto f0=edited->save(output/(label+"-f0"));require(f0->result==f0->RESULT_SUCCESS,"Backend-stale fixture Snapshot failed");
      const auto retained=serialization::sha256File(output/(label+"-f0")/"snapshot.json");
      auto committed=edited->save(output/(label+"-committed"),true,label);require(committed->result==committed->RESULT_SUCCESS,"Backend-stale fixture Finalize failed");
      const auto references=edited->references();const auto graph_revision=edited->graphRevision();edited->changeOnlyBackend();const auto backend=state(edited->backend());
      require(edited->graphRevision()==graph_revision&&references==edited->references(),"Backend-only control changed graph/reference");
      edited->failDiagnostic("result","finalize_request_id_stale");auto stale=edited->save(output/(label+"-f1"),true,label);
      recordFailure(label,stale,edited->jointConsistent(),edited->faulted(),nativeSolves(output/(label+".jsonl")));
      require(edited->diagnosticInjections()==1&&stale->result==stale->RESULT_FAILED&&!stale->finalized&&!stale->state_committed&&stale->commit_id==0,"Backend-stale request inherited an unrelated old commit");
      require(!edited->faulted()&&backend==state(edited->backend())&&references==edited->references()&&nativeSolves(output/(label+".jsonl"))==1&&!std::filesystem::exists(output/(label+"-f1"))&&retained==serialization::sha256File(output/(label+"-f0")/"snapshot.json"),"Backend-stale rejection changed state, F0, or numerical budget");
      edited->cleanup();edited.reset();
    }
    // Actual DDS calls arrive from an independent Python process. This is not
    // a callback-only assertion and does not use a mock load service.
    const std::string wire_client=SNAPSHOT_WIRE_CLIENT_PATH, directory=output.string();
    char * child_argv[]={const_cast<char *>("python3"),const_cast<char *>(wire_client.c_str()),const_cast<char *>(directory.c_str()),nullptr};
    pid_t child=-1;require(posix_spawnp(&child,"python3",nullptr,nullptr,child_argv,environ)==0,"Unable to start independent wire client");
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(probe);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(60);int status=0;
    while(waitpid(child,&status,WNOHANG)==0) {
      executor.spin_some(std::chrono::milliseconds(5));
      if(std::chrono::steady_clock::now()>deadline) {kill(child,SIGKILL);waitpid(child,&status,0);throw std::runtime_error("Wire client exceeded owned process budget");}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    executor.remove_node(probe);
    require(WIFEXITED(status)&&WEXITSTATUS(status)==0,"Independent typed LoadSnapshot wire test failed");
    require(original==state(probe->backend())&&reference==probe->references(),"Wire pure load did not restore exact two states");
    require(nativeSolves(output/"backend.jsonl")==after_finalize,"Typed wire Load invoked a hidden numerical solve");
    probe->cleanup();probe.reset();rclcpp::shutdown();
    std::ofstream(output/"result.json") << "{\"passed\":true,\"scope\":\"real common service callback paths, synthetic geometry\",\"snapshot_no_compute\":true,\"busy_boundary_rejected\":true,\"hash_and_archive_failure_preserve_live\":true,\"profile_and_activation_failure_preserve_live\":true,\"pure_load_exact_backend_reference_versions\":true,\"repeated_same_name_load\":true,\"scan_holder_cleared\":true,\"finalize_same_request_one_budget\":true,\"post_commit_io_failure_explicit\":true,\"post_commit_diagnostic_failure_explicit\":true,\"idempotent_diagnostic_failure_explicit\":true,\"reference_stale_diagnostic_failure_rejected\":true,\"backend_stale_diagnostic_failure_rejected\":true,\"diagnostic_control_native_solves\":3,\"paused_first_measurement_rejected\":true,\"holder_noncontiguous_ids\":true,\"unconstrained_finalize_preserves_f0\":true,\"loaded_raster_byte_exact\":true,\"nonstd_karto_failure_preserves_live\":true,\"typed_load_cross_process\":true,\"native_solves_before\":" << before_solves << ",\"native_solves_after\":" << after_finalize << "}\n";
    return 0;
  } catch(const karto::Exception & e) {std::cerr<<e.GetErrorMessage()<<'\n';}
  catch(const std::exception & e){std::cerr<<e.what()<<'\n';}
  rclcpp::shutdown();return 1;
}
