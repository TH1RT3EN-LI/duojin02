#define main archived_main
#include "native_probe.cpp"
#undef main
int main(int argc,char**argv){try{
 require(argc==2,"output");tbb::global_control limit(tbb::global_control::max_allowed_parallelism,2);
 auto*qLaser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("stage-query"));auto*rLaser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("stage-reference"));
 for(auto*l:{qLaser,rLaser}){l->SetMinimumRange(.05);l->SetMaximumRange(4);l->SetRangeThreshold(2);l->SetMinimumAngle(0);l->SetMaximumAngle(l==qLaser?0:.2);l->SetAngularResolution(.1);l->SetOffsetPose(Pose2());SensorManager::GetInstance()->RegisterSensor(l);}
 {
 LocalizedRangeScan q(qLaser->GetName(),RangeReadingsVector{1},LocalizedRangeScan::LegacyRawTag{}),a(rLaser->GetName(),RangeReadingsVector{1,1,1},LocalizedRangeScan::LegacyRawTag{}),b(rLaser->GetName(),RangeReadingsVector{1,1,1},LocalizedRangeScan::LegacyRawTag{});
 q.SetCorrectedPose(Pose2());a.SetCorrectedPose(Pose2(-.08,0,0));b.SetCorrectedPose(Pose2(.08,0,0));q.SetTime(3);a.SetTime(1);b.SetTime(2);
 for(auto*s:{&q,&a,&b}){s->SetRawSensorLimits(.05,4);s->SetOdometricPose(s->GetCorrectedPose());}
 Mapper mapper;std::unique_ptr<ScanMatcher>m(ScanMatcher::Create(&mapper,.4,.01,.01,2));m->SetSelectAttainedMode(true);m->SetCollectMatchDiagnostics(true);
 Pose2 mean;Matrix3 cov;double score=m->MatchLidarEvidence(&q,LocalizedRangeScanVector{&a,&b},mean,cov,true);auto d=m->GetLastMatchDiagnostics();
 require(d.stages.size()==2,"coarse and fine retained");require(d.stages[0].modes.size()>1&&d.stages[0].unresolved,"constructed coarse support remains multimodal");
 require(d.unresolved,"fine refinement cannot erase recorded coarse ambiguity");require(d.stages[0].search_center==Pose2(),"coarse search center immutable");
 require(d.stages[1].search_center==d.stages[0].selected_pose,"fine center tied to actual coarse representative");
 require(d.support.known()&&!d.correlation_grid_payload,"source support is owned without full grid copy");
 require(d.support.context.find(";mode-center-variances=")!=std::string::npos,"mode tie-break scales bind unpenalized source support");
 require(score==m->EvaluatePoseOnCurrentGrid(&q,mean)&&score>0,"final score is actual returned pose");
 std::ofstream out(argv[1]);out<<J{{"kind","coarse_fine_ambiguity"},{"score",score},{"diagnostics",diagnostic(m.get())},{"passed",true}}.dump()<<'\n';
 }
 for(auto*l:{qLaser,rLaser}){SensorManager::GetInstance()->UnregisterSensor(l);delete l;}return 0;
}catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
