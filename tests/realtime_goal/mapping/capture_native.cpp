#define main archived_main
#include "native_probe.cpp"
#undef main
#include <chrono>
int main(int argc,char**argv){try{
 require(argc==4,"pool epoch output");tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism,2);
 J pool;std::ifstream in(argv[1]);in>>pool;std::ofstream out(argv[3]);
 for(std::string arm:{"T0D0","T0D1"}){
  std::string base=std::string(argv[2])+"/mapping/"+arm+"/maps/mapping_default";
  Dataset dataset;dataset.LoadFromFile(base+".data");for(auto*o:dataset.GetLasers())SensorManager::GetInstance()->RegisterSensor(dynamic_cast<Sensor*>(o));
  Mapper mapper;mapper.LoadFromFile(base+".posegraph");mapper.Initialize(8.);std::map<int,LocalizedRangeScan*>scans;
  for(auto&s:mapper.GetGraph()->GetVertices())for(auto&v:s.second)scans[v.first]=v.second->GetObject();
  bool positiveSeen=false,negativeSeen=false;
  for(const auto&a:pool["arms"][arm]["attempts"]){
   bool positive=a.contains("fine")&&a["fine"]["decision"]=="accepted";
   if(positive?positiveSeen:negativeSeen)continue;if(positive)positiveSeen=true;else negativeSeen=true;
   for(auto it=a["reference_base_poses"].begin();it!=a["reference_base_poses"].end();++it)scans.at(std::stoi(it.key()))->SetCorrectedPoseAndUpdate(pose(it.value()));
   auto*q=scans.at(a["query_id"].get<int>());q->SetCorrectedPoseAndUpdate(pose(a["query_base_before"]));LocalizedRangeScanVector refs;
   for(auto&i:a["coarse"]["chain_ids"]){auto*s=scans.at(i.get<int>());require(s!=q,"query outside references");refs.push_back(s);}
   LocalizedRangeScan query(*q,LocalizedRangeScan::MeasurementCloneTag{});query.SetSensorPose(pose(a["coarse"]["search_center_sensor_pose"]));
   for(std::string matcherKind:{"loop","sequential"}){
   auto*m=matcherKind=="loop"?mapper.GetGraph()->GetLoopScanMatcher():mapper.GetSequentialScanMatcher();m->SetCollectMatchDiagnostics(false);m->SetSelectAttainedMode(false);m->SetReturnSelectedPoseScore(true);
   ScanMatcher::MatchDiagnostics captured;std::string capturedBytes;double referenceScore=0;Pose2 referencePose;Matrix3 referenceCov;
   // Counterbalanced pairs on the same input/binary. No GT, parameter selection
   // or matcher-output differences can explain the measured copy-only overhead.
   for(int repetition=0;repetition<12;++repetition)for(int order=0;order<2;++order){
    bool capture=((repetition+order)%2)==0;m->SetCaptureCorrelationGrid(capture);Pose2 mean;Matrix3 cov;
    auto start=std::chrono::steady_clock::now();double score=m->MatchLidarEvidence(&query,refs,mean,cov,true);
    double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();const auto&d=m->GetLastMatchDiagnostics();
    require(d.support.known()&&d.support.views.size()==refs.size()+1,"lightweight source support retained");
    require(bool(d.correlation_grid_payload)==capture,"only explicit capture owns full grid bytes");
    require(d.stages.size()==2,"both coarse and fine stage diagnostics retained");
    for(const auto&stage:d.stages)require(!stage.lattice_context.empty(),"stage has exact lattice identity");
    if(repetition==0&&order==0){referenceScore=score;referencePose=mean;referenceCov=cov;}
    require(score==referenceScore&&mean==referencePose,"capture toggle preserves score and pose");
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)require(cov(i,j)==referenceCov(i,j),"capture toggle preserves covariance");
    if(captured.correlation_grid_payload)require(captured.correlation_grid_payload->bytes==capturedBytes,"previous captured grid remains immutable after next match");
    if(capture){captured=d;capturedBytes=d.correlation_grid_payload->bytes;}
    out<<J{{"matcher",matcherKind},{"arm",arm},{"attempt_index",a["attempt_index"]},{"historical_label",positive?"positive":"negative"},{"repeat",repetition},{"order",order},{"capture",capture},{"grid_bytes",d.correlation_grid_payload?d.correlation_grid_payload->bytes.size():0},{"elapsed_seconds",elapsed},{"stage_count",d.stages.size()},{"score",score},{"pose",jp(mean)},{"source_known",d.support.known()},{"passed",true}}.dump()<<'\n';
   }
   }
   if(positiveSeen&&negativeSeen)break;
  }
 }
 return 0;
}catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
