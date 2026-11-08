#define main archived_main
#include "native_probe.cpp"
#undef main
#include <tbb/global_control.h>

int main(int argc, char ** argv) {
 try {
  require(argc==2,"output path"); std::ofstream out(argv[1]);
  tbb::global_control concurrency(tbb::global_control::max_allowed_parallelism,2);
  auto * laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name("root-mode-regression"));
  laser->SetMinimumRange(.05);laser->SetMaximumRange(4);laser->SetRangeThreshold(2);
  laser->SetMinimumAngle(0);laser->SetMaximumAngle(0);laser->SetAngularResolution(.01);laser->SetOffsetPose(Pose2());
  SensorManager::GetInstance()->RegisterSensor(laser);
  {
   LocalizedRangeScan q(laser->GetName(),RangeReadingsVector{1},LocalizedRangeScan::LegacyRawTag{});
   q.SetRawSensorLimits(.05,4);q.SetCorrectedPose(Pose2());q.SetOdometricPose(Pose2());
   Mapper mapper;std::unique_ptr<ScanMatcher> m(ScanMatcher::Create(&mapper,.4,.01,.03,2));
   m->SetCollectMatchDiagnostics(true);m->SetSelectAttainedMode(true);auto*g=m->GetCorrelationGrid();
   g->GetCoordinateConverter()->SetOffset(Vector2<double>(-2.2,-2.2));
   auto mark=[&](double x,double y){auto c=g->WorldToGrid(Vector2<double>(x,y));g->GetDataPointer()[g->GridIndex(c)]=GridStates_Occupied;};
   // Exact symmetric mode tie remains explicitly unresolved. Move only the
   // declared search center to break the tie without changing either peak.
   for(double center:{-.02,0.0,.02}){
    g->Clear();mark(.95,0);mark(1.05,0);Pose2 mean;Matrix3 cov;
    double score=m->CorrelateScan(&q,Pose2(center,0,1e-9),Vector2<double>(.1,0),Vector2<double>(.01,.01),1e-9,.01,false,mean,cov,false);
    require(score==1&&fabs(fabs(mean.GetX())-.05)<1e-10,"returns supported equal-peak representative");
    require(center==0?m->GetLastMatchDiagnostics().selection_tied:(center*mean.GetX()>0),"search-center policy rather than lattice-order preference");
    out<<J{{"kind","search_center_tie_policy"},{"center",center},{"score",score},{"diagnostics",diagnostic(m.get())},{"passed",true}}.dump()<<'\n';
   }
   // A single connected ring-shaped plateau has a hole at its centroid. The
   // projection must return an actual maximum, despite having only one mode.
   g->Clear();for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)if(abs(x)==2||abs(y)==2)mark(1+.01*x,.01*y);
   Pose2 mean;Matrix3 cov;double score=m->CorrelateScan(&q,Pose2(0,0,1e-9),Vector2<double>(.04,.04),Vector2<double>(.01,.01),1e-9,.01,false,mean,cov,false);
   require(score==1&&m->GetLastMatchDiagnostics().modes.size()==1&&m->GetLastMatchDiagnostics().projected_to_attained,"non-convex single plateau mean projected onto actual support");
   out<<J{{"kind","nonconvex_plateau"},{"score",score},{"diagnostics",diagnostic(m.get())},{"passed",true}}.dump()<<'\n';
   // Refinement passes the same object as const search-center and output. Its
   // declared objective, covariance and diagnostics must equal non-aliased calls.
   for(bool select:{false,true})for(bool fine:{false,true}){
    g->Clear();mark(1.03,0);m->SetSelectAttainedMode(select);
    Pose2 input(0,0,1e-9),separate,aliased=input;Matrix3 ca,cb;ca.SetToIdentity();cb.SetToIdentity();
    double sa=m->CorrelateScan(&q,input,Vector2<double>(.1,.02),Vector2<double>(.01,.01),.01,.01,true,separate,ca,fine);
    auto da=m->GetLastMatchDiagnostics();
    double sb=m->CorrelateScan(&q,aliased,Vector2<double>(.1,.02),Vector2<double>(.01,.01),.01,.01,true,aliased,cb,fine);
    require(fabs(sa-sb)<1e-14&&separate==aliased,"aliased search input preserves selected score and pose");
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)require(fabs(ca(i,j)-cb(i,j))<1e-12,"alias covariance equality");
    require(m->GetLastMatchDiagnostics().stages.at(0).search_center==input,"stage retains original input center");
    require(sa>0&&sa<1,"nonzero displacement pays declared prior penalty");
    out<<J{{"kind","aliased_center"},{"mode_selection",select},{"fine",fine},{"score",sa},{"diagnostics",diagnostic(m.get())},{"passed",true}}.dump()<<'\n';
   }
  }
  SensorManager::GetInstance()->UnregisterSensor(laser);delete laser;return 0;
 }catch(const karto::Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}
 catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
