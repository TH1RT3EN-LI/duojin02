#define main archived_main
#include "native_probe.cpp"
#undef main
#include <chrono>
int main(int argc,char**argv){try{
 tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism,2);
 require(argc==4,"pool epoch output");J pool;std::ifstream in(argv[1]);in>>pool;std::ofstream out(argv[3]);
 for(std::string arm:{"T0D0","T0D1"}){
 std::string base=std::string(argv[2])+"/mapping/"+arm+"/maps/mapping_default";Dataset dataset;dataset.LoadFromFile(base+".data");for(auto*o:dataset.GetLasers())SensorManager::GetInstance()->RegisterSensor(dynamic_cast<Sensor*>(o));Mapper mapper;mapper.LoadFromFile(base+".posegraph");mapper.Initialize(8.);std::map<int,LocalizedRangeScan*>scans;for(auto&s:mapper.GetGraph()->GetVertices())for(auto&v:s.second)scans[v.first]=v.second->GetObject();
 for(const auto&a:pool["arms"][arm]["attempts"]){
 for(auto it=a["reference_base_poses"].begin();it!=a["reference_base_poses"].end();++it)scans.at(std::stoi(it.key()))->SetCorrectedPoseAndUpdate(pose(it.value()));auto*q=scans.at(a["query_id"].get<int>());q->SetCorrectedPoseAndUpdate(pose(a["query_base_before"]));LocalizedRangeScanVector refs;for(auto&i:a["coarse"]["chain_ids"]){auto*s=scans.at(i.get<int>());require(s!=q,"frozen query excluded from references");refs.push_back(s);}J policies=J::array();
 for(int policy:{0,1,2}){
 const bool semantic=policy>0;
 LocalizedRangeScan query(*q,LocalizedRangeScan::MeasurementCloneTag{});query.SetSensorPose(pose(a["coarse"]["search_center_sensor_pose"]));auto*coarse=mapper.GetGraph()->GetLoopScanMatcher();auto*fine=mapper.GetSequentialScanMatcher();for(auto*m:{coarse,fine}){m->SetCollectMatchDiagnostics(false);m->SetSelectAttainedMode(policy==2);m->SetReturnSelectedPoseScore(semantic);m->SetCaptureCorrelationGrid(false);}
 const auto start=std::chrono::steady_clock::now();Pose2 mean;Matrix3 cov;double score=coarse->MatchLidarEvidence(&query,refs,mean,cov,false);require(coarse->GetLastMatchDiagnostics().support.known()&&coarse->GetLastMatchDiagnostics().support.views.size()==refs.size()+1&&!coarse->GetLastMatchDiagnostics().correlation_grid_payload,"coarse owns source support without optional grid copy");double actual=coarse->EvaluatePoseOnCurrentGrid(&query,mean);require(!semantic||fabs(score-actual)<1e-12,"all-candidate coarse actual-score semantics");bool coarseGate=score>.45&&cov(0,0)<9&&cov(1,1)<9;J row={{"policy",policy==2?"supported_mode":semantic?"returned_pose_score":"legacy_peak"},{"coarse_score",score},{"coarse_peak",coarse->GetLastMatchDiagnostics().peak_score},{"coarse_actual_score",actual},{"coarse_pose",jp(mean)},{"coarse_gate",coarseGate},{"fine_executed",false},{"coarse_diagnostics",diagnostic(coarse)}};
 if(coarseGate){LocalizedRangeScan tmp(query,LocalizedRangeScan::MeasurementCloneTag{});tmp.SetSensorPose(mean);Pose2 refined;Matrix3 shape;double fs=fine->MatchLidarEvidence(&tmp,refs,refined,shape,true);require(fine->GetLastMatchDiagnostics().support.known()&&fine->GetLastMatchDiagnostics().support.views.size()==refs.size()+1&&!fine->GetLastMatchDiagnostics().correlation_grid_payload,"fine owns source support without optional grid copy");double fa=fine->EvaluatePoseOnCurrentGrid(&tmp,refined);require(!semantic||fabs(fs-fa)<1e-12,"all-candidate fine actual-score semantics");row["fine_executed"]=true;row["fine_score"]=fs;row["fine_peak"]=fine->GetLastMatchDiagnostics().peak_score;row["fine_actual_score"]=fa;row["fine_pose"]=jp(refined);row["fine_gate"]=fs>=.55;row["fine_diagnostics"]=diagnostic(fine);}
 row["elapsed_seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();policies.push_back(row);
 }
 out<<J{{"arm",arm},{"attempt_index",a["attempt_index"]},{"query_id",a["query_id"]},{"raw_stamp_ns",a["query_raw_stamp_ns"]},{"chain_ids",a["coarse"]["chain_ids"]},{"historical_coarse_gate",a["coarse"]["decision"]},{"historical_fine_gate",a.contains("fine")?a["fine"]["decision"]:J(nullptr)},{"policies",policies},{"score_pose_consistency_passed",true},{"physical_accuracy_evaluated",false}}.dump()<<'\n';out.flush();
 }
 }return 0;
}catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
