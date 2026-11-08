#include KARTO_MAPPER_IMPLEMENTATION
#define main existing_model_probe_main
#include "model_probe.cpp"
#undef main
int main(int argc,char**argv){try{if(argc!=2)throw std::runtime_error("usage: alternate_rollback OUTPUT_DIRECTORY");output_directory=argv[1];std::filesystem::create_directories(output_directory);
 auto*l=laser("all_entry_rollback",0);Mapper mapper;CaptureSolver solver;mapper.SetScanSolver(&solver);mapper.setParamDoLoopClosing(false);mapper.setParamMinimumTravelDistance(0);mapper.setParamMinimumTravelHeading(0);mapper.setParamMinimumTimeInterval(0);
 mapper.setParamCoarseSearchAngleOffset(KT_PI);mapper.setParamCoarseAngleResolution(KT_PI/16);mapper.setParamFineSearchAngleOffset(KT_PI/32);
 std::mt19937 rng(152);std::vector<std::unique_ptr<LocalizedRangeScan>> accepted;
 for(int i=0;i<2;++i){accepted.push_back(geometry(l,false,0,Vec::Zero(),rng,1000000000LL+i*100000000LL));check(mapper.Process(accepted.back().get()),"initial supported scan");}
 auto* manager=mapper.GetMapperSensorManager();const auto running=manager->GetRunningScans(l->GetName());auto* last=manager->GetLastScan(l->GetName());const size_t nodes=solver.nodes.size(),edges=mapper.GetGraph()->GetEdges().size();
 for(int entry=0;entry<4;++entry){auto q=geometry(l,false,0,Vec(0,0,KT_PI),rng,2000000000LL+entry*100000000LL);bool result;
  if(entry==0)result=mapper.Process(q.get());else if(entry==1)result=mapper.ProcessAgainstNodesNearBy(q.get());else if(entry==2)result=mapper.ProcessLocalization(q.get());else result=mapper.ProcessAgainstNode(q.get(),0);
  std::cout<<"{\"test\":\"alternate_entry_rollback\",\"entry\":"<<entry<<",\"accepted\":"<<(result?"true":"false")<<",\"query_yaw\":"<<q->GetCorrectedPose().GetHeading()<<"}\n";
  check(!result,"native antipodal matcher result must reject chart");check(solver.nodes.size()==nodes&&mapper.GetGraph()->GetEdges().size()==edges,"no graph half commit");
  check(manager->GetLastScan(l->GetName())==last&&manager->GetRunningScans(l->GetName())==running,"all entry points restore prior running/last roster");
  check(logChart(Pose2(),q->GetCorrectedPose()).value.norm()<1e-12&&logChart(Pose2(),q->GetOdometricPose()).value.norm()<1e-12,"all entry points restore input corrected/odometric pose");check(q->GetUniqueId()==-1&&q->GetStateId()==-1,"all entry points restore query identifiers");}
 auto next=geometry(l,false,0,Vec::Zero(),rng,3000000000LL);check(mapper.Process(next.get()),"valid query after rejected alternatives");check(next->GetUniqueId()==2&&next->GetStateId()==2,"all entry rejections restore allocator");
 std::cout<<"{\"test\":\"all_entry_atomic_rejection\",\"passed\":true,\"entries\":4,\"native_antipodal_scan\":true}\n";return 0;
 }catch(const Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
