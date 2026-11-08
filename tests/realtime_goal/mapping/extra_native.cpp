#define main archived_main
#include "native_probe.cpp"
#undef main
#include <chrono>
int main(int argc,char**argv){try{
 tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism,2);
 require(argc==2,"output path");std::ofstream out(argv[1]);
 auto*l=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("realtime-extra"));l->SetMinimumRange(.05);l->SetMaximumRange(4);l->SetRangeThreshold(2);l->SetMinimumAngle(0);l->SetMaximumAngle(0);l->SetAngularResolution(.01);l->SetOffsetPose(Pose2());SensorManager::GetInstance()->RegisterSensor(l);
 {
 LocalizedRangeScan q(l->GetName(),RangeReadingsVector{1},LocalizedRangeScan::LegacyRawTag{});q.SetRawSensorLimits(.05,4);q.SetCorrectedPose(Pose2());q.SetOdometricPose(Pose2());Mapper mapper;std::unique_ptr<ScanMatcher>m(ScanMatcher::Create(&mapper,.4,.01,.03,2));auto*g=m->GetCorrelationGrid();g->GetCoordinateConverter()->SetOffset(Vector2<double>(-2.2,-2.2));
 auto cell=g->WorldToGrid(Vector2<double>(1.02,0));g->GetDataPointer()[g->GridIndex(cell)]=GridStates_Occupied;
 Pose2 mean;Matrix3 cov;double score=m->CorrelateScan(&q,Pose2(0,0,1e-9),Vector2<double>(.1,0),Vector2<double>(.01,.01),1e-9,.01,false,mean,cov,false);
 require(score==1&&!m->GetLastMatchDiagnostics().mode_diagnostics_collected&&m->GetLastMatchDiagnostics().score_evaluated,"default evaluates returned pose while mode enumeration remains opt-in");out<<J{{"kind","default_compatibility"},{"score",score},{"diagnostics",diagnostic(m.get())},{"passed",true}}.dump()<<'\n';
 // Same native objective across the branch cut; no linear heading average.
 for(double center:{KT_PI-.005,-KT_PI+.005}){
 g->Clear();auto c=g->WorldToGrid(Vector2<double>(-1,0));g->GetDataPointer()[g->GridIndex(c)]=GridStates_Occupied;m->SetCollectMatchDiagnostics(true);m->SetSelectAttainedMode(true);m->SetReturnSelectedPoseScore(true);
 score=m->CorrelateScan(&q,Pose2(0,0,center),Vector2<double>(.01,.01),Vector2<double>(.01,.01),.02,.005,false,mean,cov,false);double actual=m->EvaluatePoseOnCurrentGrid(&q,mean);require(score==actual&&score>0&&std::abs(mean.GetHeading())>3.1,"branch-cut output actual score and normalized attained angle");out<<J{{"kind","heading_wrap"},{"search_center",center},{"actual_score",actual},{"diagnostics",diagnostic(m.get())},{"passed",true}}.dump()<<'\n';}
 for(bool diagnostics:{false,true}){
 m->SetCollectMatchDiagnostics(diagnostics);m->SetSelectAttainedMode(false);m->SetReturnSelectedPoseScore(false);const auto start=std::chrono::steady_clock::now();for(int i=0;i<100;++i)m->CorrelateScan(&q,Pose2(0,0,1e-9),Vector2<double>(.1,.1),Vector2<double>(.01,.01),.01,.01,false,mean,cov,false);double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();out<<J{{"kind","diagnostic_cost_synthetic_100_calls"},{"collect",diagnostics},{"seconds",elapsed},{"scope","one-beam synthetic microbenchmark; not real replay latency"},{"passed",true}}.dump()<<'\n';}
 }SensorManager::GetInstance()->UnregisterSensor(l);delete l;return 0;
}catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
