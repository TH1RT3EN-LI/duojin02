#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <thread>
#include <type_traits>
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "slam_toolbox/slam_toolbox_sync.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "slam_toolbox/slam_toolbox_localization.hpp"
#include "slam_toolbox/experimental/slam_toolbox_lifelong.hpp"
#include "slam_toolbox/experimental/slam_toolbox_map_and_localization.hpp"
#include "slam_toolbox/serialization.hpp"
#include "slam_toolbox/laser_binding_contract.hpp"
#include "solvers/ceres_solver.hpp"
using namespace karto;
using namespace toolbox_types;
namespace fs=std::filesystem;
static void require(bool value,const char * why){if(!value)throw std::runtime_error(why);}
static double difference(const tf2::Transform & a,const tf2::Transform & b){
  double result=(a.getOrigin()-b.getOrigin()).length();
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)result=std::max(result,std::abs(a.getBasis()[i][j]-b.getBasis()[i][j]));return result;
}
static sensor_msgs::msg::LaserScan::SharedPtr source(double seconds=1.5){
  auto scan=std::make_shared<sensor_msgs::msg::LaserScan>();scan->header.stamp=duojin02_scan_geometry::stampMsg(int64_t(std::llround(seconds*1e9)));scan->header.frame_id="observation_laser";
  scan->angle_min=-1.5f;scan->angle_max=1.5f;scan->angle_increment=.02f;scan->range_min=.05f;scan->range_max=20.f;scan->time_increment=.05f/150;scan->scan_time=.05f;
  for(int i=0;i<151;++i){const double a=-1.5+.02*i;scan->ranges.push_back(3.5+.7*std::sin(3*a)+.4*std::cos(7*a));}return scan;
}
static void versionOneBundle(const fs::path & input,const fs::path & output,bool binding_role){
  fs::copy(input,output,fs::copy_options::recursive);boost::property_tree::ptree manifest;boost::property_tree::read_json((output/"snapshot.json").string(),manifest);manifest.put("producer","slam_toolbox.SaveSnapshot.v1");
  if(!binding_role){boost::property_tree::ptree files;for(const auto & item:manifest.get_child("files"))if(item.second.get<std::string>("role")!="laser_bindings")files.push_back(item);manifest.put_child("files",files);fs::remove(output/"laser_bindings.json");}
  std::map<std::string,std::string> records;for(const auto & item:manifest.get_child("files")){const auto & v=item.second;const auto role=v.get<std::string>("role");records[role]=role+"\t"+v.get<std::string>("path")+"\t"+v.get<std::string>("sha256")+"\t"+std::to_string(v.get<std::uintmax_t>("size_bytes"))+"\n";}
  std::string identity;for(const auto & item:records)identity+=item.second;identity+=manifest.get<std::string>("operation")+"\n"+manifest.get<std::string>("request_id")+"\n";manifest.put("snapshot_id",serialization::sha256Text(identity));boost::property_tree::write_json((output/"snapshot.json").string(),manifest);
}
template<class Base> class Access : public Base {
public:
  explicit Access(const rclcpp::NodeOptions & options):Base(options){}
  ~Access(){release();if constexpr(std::is_same_v<Base,slam_toolbox::SynchronousSlamToolbox>){this->stop_queue_=true;if(this->queue_thread_&&this->queue_thread_->joinable())this->queue_thread_->join();}}
  void setup(){
    this->setParams();this->setROSInterfaces();this->solver_=std::make_shared<solver_plugins::CeresSolver>();this->solver_->Configure(this->shared_from_this());
    this->smapper_->getMapper()->SetScanSolver(this->solver_.get());
    this->laser_assistant_=std::make_unique<laser_utils::LaserAssistant>(this->shared_from_this(),this->tf_.get(),this->base_frame_);
    this->pose_helper_=std::make_unique<pose_utils::GetPoseHelper>(this->tf_.get(),this->base_frame_,this->odom_frame_);
    this->scan_holder_=std::make_unique<laser_utils::ScanHolder>(this->lasers_);
    this->closure_assistant_=std::make_unique<loop_closure_assistant::LoopClosureAssistant>(this->shared_from_this(),this->smapper_->getMapper(),this->scan_holder_.get(),this->state_,this->processor_type_,this->smapper_mutex_);
    this->reprocessing_transform_.setIdentity();
    auto * mapper=this->smapper_->getMapper();mapper->setParamDoLoopClosing(false);mapper->setParamMinimumTravelDistance(0);mapper->setParamMinimumTravelHeading(0);mapper->setParamMinimumTimeInterval(0);
  }
  void put(double seconds,bool nonplanar=true){
    geometry_msgs::msg::TransformStamped sample;sample.header.frame_id=this->odom_frame_;sample.child_frame_id=this->base_frame_;sample.header.stamp=duojin02_scan_geometry::stampMsg(int64_t(std::llround(seconds*1e9)));
    sample.transform.translation.x=.4;sample.transform.translation.y=-.2;sample.transform.translation.z=nonplanar?.35:0;
    tf2::Quaternion q;q.setRPY(nonplanar?.12:0,nonplanar?-.08:0,.3);sample.transform.rotation=tf2::toMsg(q);require(this->tf_->setTransform(sample,"observation-probe",false),"TF insertion failed");
    sample.header.frame_id=this->base_frame_;sample.child_frame_id="observation_laser";sample.transform.translation.x=.305;sample.transform.translation.y=.03;sample.transform.translation.z=0;q.setRPY(0,0,.07);sample.transform.rotation=tf2::toMsg(q);require(this->tf_->setTransform(sample,"observation-probe",true),"Static laser insertion failed");
  }
  void putLaser(const std::string & frame,double offset=.6){
    geometry_msgs::msg::TransformStamped sample;sample.header.frame_id=this->base_frame_;sample.child_frame_id=frame;sample.transform.translation.x=offset;sample.transform.rotation.w=1;require(this->tf_->setTransform(sample,"observation-probe",true),"Second laser TF insertion failed");
  }
  ObservationPtr admission(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan){std::lock_guard<std::recursive_mutex> lock(this->physical_dispatch_mutex_);return this->prepareObservation(scan);}
  size_t devices(){std::lock_guard<std::mutex> lock(this->laser_registry_mutex_);return this->lasers_.size();}
  void slowOwnerStatus(){
    std::promise<void> entered,release;auto ready=release.get_future();std::thread owner([&]{boost::mutex::scoped_lock lock(this->smapper_mutex_);auto graph=this->smapper_->getMapper()->LockGraphState();entered.set_value();ready.wait();});entered.get_future().wait();
    try {const auto before=std::chrono::steady_clock::now();const auto value=status();require(value.template get<std::string>("schema")=="slam-input-status.v2"&&std::chrono::steady_clock::now()-before<std::chrono::seconds(2),"Status blocked on slow graph owner");release.set_value();owner.join();}catch(...){release.set_value();owner.join();throw;}
  }
  void feed(sensor_msgs::msg::LaserScan::ConstSharedPtr scan){this->enqueuePhysicalScan(std::move(scan));}
  void feedPrepared(sensor_msgs::msg::LaserScan::ConstSharedPtr scan,GeometryPtr geometry){std::lock_guard<std::recursive_mutex> lock(this->physical_dispatch_mutex_);Base::laserCallback(std::move(scan),std::move(geometry));}
  void waitEntered(){std::unique_lock<std::mutex> lock(gate_);require(changed_.wait_for(lock,std::chrono::seconds(5),[&]{return bool(captured_);}),"Worker did not reach packet gate");}
  ObservationPtr packet(){std::lock_guard<std::mutex> lock(gate_);return captured_;}
  void release(){std::lock_guard<std::mutex> lock(gate_);hold_=false;changed_.notify_all();}
  void arm(){std::lock_guard<std::mutex> lock(gate_);hold_=true;captured_.reset();done_=false;success_=false;}
  void waitDone(){const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(!done_&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(2));require(done_&&success_,"Real processing path failed");}
  void waitIdle(){if constexpr(std::is_same_v<Base,slam_toolbox::SynchronousSlamToolbox>){const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(5);while(!this->snapshotInputIdle()&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(2));require(this->snapshotInputIdle(),"Sync worker failed to close in-flight item");}}
  bool expired(const ObservationPtr & observation){Pose2 pose;tf2::Stamped<tf2::Transform> full;return !this->pose_helper_->getOdomSample(pose,full,rclcpp::Time(duojin02_scan_geometry::stampMsg(observation->reference_stamp_ns)));}
  Pose2 corrected(){return corrected_;}
  tf2::Transform output(){boost::mutex::scoped_lock lock(this->map_to_odom_mutex_);return this->map_to_odom_;}
  bool outputKnown(){boost::mutex::scoped_lock lock(this->map_to_odom_mutex_);return this->has_processed_observation_;}
  int64_t rasterStamp(){nav_msgs::msg::OccupancyGrid grid;boost::mutex::scoped_lock lock(this->smapper_mutex_);auto owner=this->smapper_->getMapper()->LockGraphState();require(this->renderMapLocked(grid),"Raster unavailable");return duojin02_scan_geometry::stampNs(grid.header.stamp);}
  tf2::Transform reprocessing(){return this->reprocessing_transform_;}
  void poisonReprocessing(){tf2::Quaternion q;q.setRPY(0,0,.4);this->reprocessing_transform_=tf2::Transform(q,tf2::Vector3(8,-3,0));}
  void pause(){this->state_.set(NEW_MEASUREMENTS,true);}
  void resumeAdmissions(){this->state_.set(NEW_MEASUREMENTS,false);}
  bool first(){return this->first_measurement_;}
  void admissionPauseControl(){pause();const auto before=source();feed(before);require(first()&&!outputKnown(),"Paused first observation changed output/admission state");this->state_.set(NEW_MEASUREMENTS,false);}
  std::shared_ptr<slam_toolbox::srv::SaveSnapshot::Response> save(const fs::path & directory,bool finalize=false,bool graph=true){auto request=std::make_shared<slam_toolbox::srv::SaveSnapshot::Request>();auto response=std::make_shared<slam_toolbox::srv::SaveSnapshot::Response>();request->filename=directory.string();request->include_graph=graph;request->mode=finalize?request->FINALIZE:request->SNAPSHOT;request->request_id=finalize?"observation-once":"";this->saveSnapshotCallback(nullptr,request,response);return response;}
  bool load(const fs::path & directory){auto request=std::make_shared<slam_toolbox::srv::LoadSnapshot::Request>();auto response=std::make_shared<slam_toolbox::srv::LoadSnapshot::Response>();request->filename=directory.string();request->match_type=std::is_same_v<Base,slam_toolbox::LocalizationSlamToolbox>?request->LOCALIZE_AT_POSE:request->START_AT_FIRST_NODE;this->loadSnapshotCallback(nullptr,request,response);return response->result==response->RESULT_SUCCESS&&response->state_committed;}
  void mappingMode(){this->processor_type_=PROCESS;}
  void seedLocalization(){auto scan=source();auto * laser=this->getLaser(scan);for(int i=0;i<3;++i){RangeReadingsVector ranges(scan->ranges.begin(),scan->ranges.end());auto * node=new LocalizedRangeScan(laser->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});node->SetRawSensorLimits(.05,20);node->SetTime(.2+i*.1);node->SetOdometricPose(Pose2(.01*i,0,0));require(this->smapper_->getMapper()->Process(node),"Localization fixture seed rejected");this->dataset_->Add(node);}this->process_near_pose_=std::make_unique<Pose2>(0,0,0);}
  void idleReset(){waitIdle();this->smapper_->Reset();require(this->smapper_->getMapper()->GetAllProcessedScans().empty()&&this->solver_->GetBackendSnapshot().nodes.empty(),"Idle SMapper reset left old graph/backend");}
  void clearControl(){if constexpr(std::is_same_v<Base,slam_toolbox::SynchronousSlamToolbox>){this->stop_queue_=true;if(this->queue_thread_&&this->queue_thread_->joinable())this->queue_thread_->join();auto p=packet();std::weak_ptr<const ScanObservation> weak=p;{boost::mutex::scoped_lock lock(this->q_mutex_);this->q_.push(p);++this->queued_scans_;}p.reset();{std::lock_guard<std::mutex> lock(gate_);captured_.reset();}auto request=std::make_shared<slam_toolbox::srv::ClearQueue::Request>();auto response=std::make_shared<slam_toolbox::srv::ClearQueue::Response>();this->clearQueueCallback(nullptr,request,response);require(weak.expired()&&response->status,"ClearQueue retained packet or failed");}}
  boost::property_tree::ptree status(){auto client=this->template create_client<std_srvs::srv::Trigger>("/slam_toolbox/processing_status");require(client->wait_for_service(std::chrono::seconds(2)),"Status service missing");auto future=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());require(rclcpp::spin_until_future_complete(this->shared_from_this(),future,std::chrono::seconds(5))==rclcpp::FutureReturnCode::SUCCESS,"Status RPC timed out");boost::property_tree::ptree tree;std::istringstream input(future.get()->message);boost::property_tree::read_json(input,tree);return tree;}
  void publisher(double period){this->publishTransformLoop(period);}
protected:
  LocalizedRangeScan * addScan(LaserRangeFinder * laser,const ObservationPtr & observation) override {
    {std::unique_lock<std::mutex> lock(gate_);captured_=observation;changed_.notify_all();changed_.wait(lock,[&]{return !hold_;});}
    auto * processed=Base::addScan(laser,observation);if(processed)corrected_=processed->GetCorrectedPose();success_=processed!=nullptr;done_=true;return processed;
  }
private:
  std::mutex gate_;std::condition_variable changed_;bool hold_{true};ObservationPtr captured_;std::atomic<bool> done_{false},success_{false};Pose2 corrected_;
};
template<class Node> struct ReleaseOnExit {std::shared_ptr<Node> node;~ReleaseOnExit(){node->release();}};
template<class Base> static void checkOutput(const std::shared_ptr<Access<Base>> & node,const ObservationPtr & packet,bool full){
  auto corrected=node->corrected();tf2::Quaternion q;q.setRPY(0,0,corrected.GetHeading());const tf2::Transform base_to_map=tf2::Transform(q,{corrected.GetX(),corrected.GetY(),0}).inverse();
  const auto expected=(packet->odom_from_base*base_to_map).inverse();require(difference(node->output(),expected)<1e-12,"Captured transform composition changed");require(node->outputKnown()&&node->rasterStamp()==packet->reference_stamp_ns,"Raster/output not bound to successful packet");
  if(full){q.setRPY(0,0,packet->pose.GetHeading());const tf2::Transform projected(q,{packet->pose.GetX(),packet->pose.GetY(),0});require(difference((projected*base_to_map).inverse(),expected)>.01,"Nonplanar control did not reject a Pose2-only surrogate");}
}
template<class Base> static void experimentalControl(){
  auto node=std::make_shared<Access<Base>>(rclcpp::NodeOptions());node->setup();node->mappingMode();node->put(1);node->put(2);node->admissionPauseControl();auto task=std::async(std::launch::async,[&]{node->feed(source());});ReleaseOnExit<Access<Base>> guard{node};node->waitEntered();auto packet=node->packet();node->put(1000);node->put(1001);require(node->expired(packet),"Experimental path TF did not expire");node->release();task.get();node->waitDone();checkOutput(node,packet,true);
}
int main(int argc,char ** argv){
  rclcpp::init(argc,argv);std::thread publisher;
  try{
    require(argc==2,"Output directory required");const fs::path out=argv[1];fs::create_directories(out);
    using Sync=slam_toolbox::SynchronousSlamToolbox;using Async=slam_toolbox::AsynchronousSlamToolbox;using Local=slam_toolbox::LocalizationSlamToolbox;
    experimentalControl<slam_toolbox::LifelongSlamToolbox>();experimentalControl<slam_toolbox::MapAndLocalizationSlamToolbox>();
    {
      auto node=std::make_shared<Access<Sync>>(rclcpp::NodeOptions());node->setup();node->put(1);node->put(2);node->admissionPauseControl();node->feed(source());node->waitEntered();auto packet=node->packet();node->put(1000);node->put(1001);require(node->expired(packet),"Old TF was not evicted");node->pause();
      node->slowOwnerStatus();const auto status=node->status();require(status.get<std::string>("schema")=="slam-input-status.v2"&&status.get<int>("capacity")==1000&&status.get<int>("physical_pending")==0&&status.get<bool>("processing")&&status.get<int>("queued")==0&&status.get<int>("outstanding")==1&&status.get<int64_t>("last_admitted_raw_ns")==packet->raw_stamp_ns&&status.get<int64_t>("last_admitted_ref_ns")==packet->reference_stamp_ns,"Status v2 omitted real in-flight/frontier state");boost::property_tree::write_json((out/"status-inflight.json").string(),status);
      auto rejected=node->save(out/"busy-finalize",true);require(rejected->result==rejected->RESULT_FAILED&&!rejected->state_committed,"Finalize ignored popped in-flight observation");
      node->release();node->waitDone();node->waitIdle();checkOutput(node,packet,true);packet.reset();auto saved=node->save(out/"sync-f0");require(saved->result==saved->RESULT_SUCCESS,"Sync packet snapshot failed");node->arm();node->resumeAdmissions();node->feed(source(1000));node->waitEntered();node->pause();require(!node->load(out/"sync-f0"),"Typed Load ignored popped in-flight packet");node->release();node->waitDone();node->waitIdle();const auto complete=node->status();require(complete.get<int64_t>("last_completed_raw_ns")==1000000000000&&complete.get<int>("outstanding")==0,"Completion frontier omitted consumed packet");boost::property_tree::write_json((out/"status-complete.json").string(),complete);node->idleReset();node->clearControl();node.reset();
    }
    {
      auto node=std::make_shared<Access<Local>>(rclcpp::NodeOptions());node->setup();node->put(1);node->put(2);node->seedLocalization();auto task=std::async(std::launch::async,[&]{node->feed(source());});ReleaseOnExit<Access<Local>> guard{node};node->waitEntered();auto packet=node->packet();node->put(1000);node->put(1001);require(node->expired(packet),"Localization TF eviction failed");node->release();task.get();node->waitDone();checkOutput(node,packet,true);node.reset();
    }
    {
      rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("scan_motion_compensation",true),rclcpp::Parameter("scan_reference_fraction",.5)});auto node=std::make_shared<Access<Async>>(options);node->setup();node->put(1,false);node->put(2,false);auto scan=source();duojin02_scan_geometry::Config config;config.enabled=true;config.reference_fraction=.5;duojin02_scan_geometry::History history;history.insert("base_footprint","observation_laser",0,{.305,.03,.07},true,120000000000);for(int i=0;i<5;++i)history.insert("odom","base_footprint",1450000000+i*50000000,{.1*i,0,.01*i},false,120000000000);
      auto geometry=std::make_shared<const duojin02_scan_geometry::TimedScan>(duojin02_scan_geometry::prepare(1500000000,"observation_laser",scan->angle_min,scan->angle_increment,scan->time_increment,scan->range_min,scan->range_max,scan->ranges,config,history,"odom","base_footprint"));auto task=std::async(std::launch::async,[&]{node->feedPrepared(scan,geometry);});ReleaseOnExit<Access<Async>> guard{node};node->waitEntered();auto packet=node->packet();history.insert("odom","base_footprint",1000000000000,{0,0,0},false,100000000);node->put(1000,false);node->put(1001,false);require(node->expired(packet),"D1 TF eviction failed");node->release();task.get();node->waitDone();checkOutput(node,packet,false);require(packet->geometry==geometry&&packet->geometry->beams.size()==scan->ranges.size(),"Prepared D1 packet lost its immutable geometry");node.reset();
    }
    {
      auto node=std::make_shared<Access<Async>>(rclcpp::NodeOptions());node->setup();node->put(1);node->put(2);node->release();node->feed(source());node->waitDone();
      auto second=source();second->header.frame_id="second_laser";node->putLaser(second->header.frame_id);const auto first=node->admission(source()),other=node->admission(second);
      require(first->laser!=other->laser&&first->laser->GetName()!=other->laser->GetName()&&node->devices()==2,"Different frames aliased the same native sensor");
      require(SensorManager::GetInstance()->GetSensorByName(first->laser->GetName())==first->laser&&SensorManager::GetInstance()->GetSensorByName(other->laser->GetName())==other->laser,"Registration replaced a queued device");
      for(int control=0;control<4;++control){auto changed=source();if(control==0)changed->ranges.pop_back();if(control==1)changed->angle_increment*=2;if(control==2)changed->range_max-=1;if(control==3)changed->range_min+=.01f;bool rejected=false;try{node->admission(changed);}catch(const std::runtime_error &){rejected=true;}require(rejected,"Cached frame accepted changed shape/range declarations");}
      const auto first_name=first->laser->GetName();auto saved=node->save(out/"two-laser-f0");require(saved->result==saved->RESULT_SUCCESS,"Two-device snapshot failed");auto raster=node->save(out/"two-laser-raster",false,false);require(raster->result==raster->RESULT_SUCCESS&&fs::exists(out/"two-laser-raster"/"laser_bindings.json"),"Raster-only v2 omitted device identities");require(node->load(out/"two-laser-f0"),"Two-device staged load failed");
      node->putLaser("observation_laser",9);bool calibration_rejected=false;try{node->admission(source());}catch(const std::runtime_error &){calibration_rejected=true;}require(calibration_rejected,"Loaded frame accepted different current calibration");node->put(1);node->put(2);
      const auto restored1=node->admission(source()),restored2=node->admission(second);require(restored1->laser!=restored2->laser&&restored1->laser->GetName()==first_name&&node->devices()==2,"Saved two-device frame identities did not restore");
      // Sidecar corruption is re-sealed honestly to test semantic validation,
      // rather than relying only on the file hash refusal path.
      serialization::BundleWriter broken(out/"conflicting-bindings");boost::property_tree::ptree manifest;boost::property_tree::read_json((out/"two-laser-f0"/"snapshot.json").string(),manifest);
      for(const auto & item:manifest.get_child("files")){const auto path=item.second.get<std::string>("path");fs::copy_file(out/"two-laser-f0"/path,broken.path()/path);}
      boost::property_tree::ptree bindings;boost::property_tree::read_json((broken.path()/"laser_bindings.json").string(),bindings);bindings.get_child("frames").front().second.get_child("calibration").front().second.put_value(123.);boost::property_tree::write_json((broken.path()/"laser_bindings.json").string(),bindings);
      for(const auto & item:manifest.get_child("files"))broken.add(item.second.get<std::string>("role"),item.second.get<std::string>("path"));broken.publish(false,"");
      auto * identity=restored1->laser;require(!node->load(out/"conflicting-bindings")&&node->admission(source())->laser==identity,"Conflicting binding damaged live device state");
      auto flat=out/"legacy-two";fs::copy_file(out/"two-laser-f0"/"graph.posegraph",flat.string()+".posegraph");fs::copy_file(out/"two-laser-f0"/"graph.data",flat.string()+".data");require(!node->load(flat),"Ambiguous legacy multi-device archive loaded without identities");node.reset();
      auto single=std::make_shared<Access<Async>>(rclcpp::NodeOptions());single->setup();single->put(1);single->put(2);single->release();single->feed(source());single->waitDone();auto one=single->save(out/"legacy-single-source");require(one->result==one->RESULT_SUCCESS,"Legacy fixture snapshot failed");flat=out/"legacy-single";fs::copy_file(out/"legacy-single-source"/"graph.posegraph",flat.string()+".posegraph");fs::copy_file(out/"legacy-single-source"/"graph.data",flat.string()+".data");require(single->load(flat),"Unique legacy sensor migration rejected");const auto bound=single->admission(source());require(bound&&single->devices()==1,"Legacy single sensor did not validate/bind first observed frame");
      versionOneBundle(out/"legacy-single-source",out/"v1-bound",true);require(single->load(out/"v1-bound"),"Manifest-bound v1 sidecar rejected");
      versionOneBundle(out/"legacy-single-source",out/"v1-unbound",false);require(single->load(out/"v1-unbound"),"Single-device v1 legacy migration rejected");auto before=single->admission(source())->laser;
      fs::copy(out/"v1-unbound",out/"v1-unmanifested-extra",fs::copy_options::recursive);fs::copy_file(out/"legacy-single-source"/"laser_bindings.json",out/"v1-unmanifested-extra"/"laser_bindings.json");require(serialization::sha256File(out/"v1-unbound"/"snapshot.json")==serialization::sha256File(out/"v1-unmanifested-extra"/"snapshot.json"),"Unmanifested-extra control changed verified identity");
      require(!single->load(out/"v1-unmanifested-extra")&&single->admission(source())->laser==before,"Unmanifested v1 sidecar influenced live Load");single.reset();
    }
    {
      auto node=std::make_shared<Access<Async>>(rclcpp::NodeOptions());node->setup();node->put(1);node->put(2);auto task=std::async(std::launch::async,[&]{node->feed(source());});ReleaseOnExit<Access<Async>> guard{node};node->waitEntered();auto packet=node->packet();node->put(1000);node->put(1001);require(node->expired(packet),"Async TF eviction failed");node->release();task.get();node->waitDone();checkOutput(node,packet,true);auto saved=node->save(out/"async-f0");require(saved->result==saved->RESULT_SUCCESS,"Async snapshot failed");
      auto observer=std::make_shared<rclcpp::Node>("observation_tf_reader");int count=0;int64_t latest=0;auto subscription=observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf",100,[&](const tf2_msgs::msg::TFMessage & message){for(const auto & tf:message.transforms)if(tf.header.frame_id=="map"&&tf.child_frame_id=="odom"){++count;latest=duojin02_scan_geometry::stampNs(tf.header.stamp);}});rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(observer);publisher=std::thread([node]{node->publisher(.01);});
      auto spin=[&](double seconds){const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);while(std::chrono::steady_clock::now()<until)executor.spin_once(std::chrono::milliseconds(10));};spin(.4);require(count>0,"Actual TF publisher did not activate");node->poisonReprocessing();require(node->load(out/"async-f0"),"Typed pure Load failed");require(!node->outputKnown()&&node->rasterStamp()==0&&difference(node->reprocessing(),tf2::Transform::getIdentity())==0,"Pure Load retained old epoch ROS output/reprocessing context");spin(.15);const int after_load=count;spin(.15);require(count==after_load,"Pure Load kept publishing old epoch TF");
      node->arm();auto resumed=std::async(std::launch::async,[&]{node->feed(source(1000));});ReleaseOnExit<Access<Async>> resumed_guard{node};node->waitEntered();auto first=node->packet();node->put(2000);node->put(2001);require(node->expired(first),"Post-load input TF did not expire");node->release();resumed.get();node->waitDone();checkOutput(node,first,true);auto corrected=node->corrected();tf2::Quaternion q;q.setRPY(0,0,corrected.GetHeading());const tf2::Transform expected=tf2::Transform(q,{corrected.GetX(),corrected.GetY(),0})*tf2::Transform(tf2::Quaternion(0,0,std::sin(first->pose.GetHeading()/2),std::cos(first->pose.GetHeading()/2)),{first->pose.GetX(),first->pose.GetY(),0}).inverse();require(difference(node->reprocessing(),expected)<1e-12,"First post-load reprocessing no longer follows original mode contract");spin(.15);require(count>after_load&&latest==first->reference_stamp_ns+500000000,"Resumed TF stamp did not use successful packet");rclcpp::shutdown();publisher.join();executor.remove_node(observer);node.reset();
    }
    std::ofstream(out/"result.json")<<"{\"passed\":true,\"scope\":\"real ROS common/sync/async/localization paths on synthetic geometry; no LiLoc/GT execution\",\"legacy_full_3d_preserved\":true,\"Pose2_only_surrogate_rejected\":true,\"delayed_tf_evicted\":true,\"experimental_entrypoints_packet\":true,\"v1_only_verified_bindings\":true,\"multi_laser_archive_and_conflicts\":true,\"legacy_single_migration_multi_refusal\":true,\"status_responsive_with_owner_held\":true,\"sync_worker_packet\":true,\"async_packet\":true,\"localization_packet\":true,\"D1_immutable_geometry\":true,\"busy_finalize_rejected\":true,\"paused_first_rejected\":true,\"idle_reset_and_clear_release\":true,\"typed_load_output_epoch_cleared\":true,\"no_old_epoch_tf_published\":true,\"post_load_reprocessing\":true,\"raster_and_tf_stamp_bound\":true}\n";return 0;
  }catch(const karto::Exception & error){std::cerr<<error.GetErrorMessage()<<'\n';}catch(const std::exception & error){std::cerr<<error.what()<<'\n';}rclcpp::shutdown();if(publisher.joinable())publisher.join();return 1;
}
