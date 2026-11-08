#include <karto_sdk/Mapper.h>
#include <Eigen/Eigenvalues>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <random>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <filesystem>
using namespace karto;
using namespace karto::measurement;
std::string output_directory;
void check(bool b,const char*w){if(!b)throw std::runtime_error(w);}
void near(double a,double b,double tol,const char*w){check(std::abs(a-b)<tol,w);}
Pose2 base(const Pose2&s,const Pose2&o){double a=s.GetHeading()-o.GetHeading();return Pose2(s.GetX()-cos(a)*o.GetX()+sin(a)*o.GetY(),s.GetY()-sin(a)*o.GetX()-cos(a)*o.GetY(),a);}
Pose2 left(const Pose2&g,const Pose2&p){Eigen::Vector2d t=rotation(g.GetHeading())*Eigen::Vector2d(p.GetX(),p.GetY());return Pose2(t(0)+g.GetX(),t(1)+g.GetY(),angle(p.GetHeading()+g.GetHeading()));}
void algebra(){
 Mat p;p<<.03,.004,.001,.004,.09,-.002,.001,-.002,.02;
 double worst=0;
 for(double arm:{0.,.305,1.2})for(double a:{-2.8,0.,2.1}){
  Pose2 s(2,-3,a),o(arm,-.2*arm,.13);Mat analytic=sensorToBaseJacobian(s,o),numeric;
  for(int k=0;k<3;++k){Vec v(s.GetX(),s.GetY(),s.GetHeading()),e=Vec::Zero();e(k)=1e-6;
   auto hi=base(Pose2(v(0)+e(0),v(1)+e(1),v(2)+e(2)),o);auto lo=base(Pose2(v(0)-e(0),v(1)-e(1),v(2)-e(2)),o);
   numeric.col(k)=Vec(hi.GetX()-lo.GetX(),hi.GetY()-lo.GetY(),hi.GetHeading()-lo.GetHeading())/(2e-6);}
  worst=std::max(worst,(analytic-numeric).norm());check((analytic-numeric).norm()<1e-8,"full sensor base Jacobian finite difference");
 }
 Pose2 anchor(1,2,3.12);Pose2Vector means{Pose2(1.03,2.01,-3.12),Pose2(.97,2.02,3.10)};std::vector<Matrix3> ps{kartoMatrix(p),kartoMatrix(2*p)};
 auto f=intersect(anchor,means,ps);auto revmeans=means;auto revps=ps;std::reverse(revmeans.begin(),revmeans.end());std::reverse(revps.begin(),revps.end());auto rev=intersect(anchor,revmeans,revps);
 check((logChart(f.mean,rev.mean).value).norm()<1e-10,"CI permutation invariant");
 means.push_back(means[0]);ps.push_back(ps[0]);auto copied=intersect(anchor,means,ps);
 check(logChart(f.mean,copied.mean).value.norm()<1e-10,"CI duplicate mean idempotent");check((f.chart_covariance-copied.chart_covariance).norm()<1e-10,"CI duplicate information idempotent");
 Pose2 g(2,-1,.7);Pose2Vector transformed;Mat r=Mat::Identity();r.topLeftCorner<2,2>()=rotation(.7);std::vector<Matrix3> rotated;
 for(size_t i=0;i<means.size();++i){transformed.push_back(left(g,means[i]));rotated.push_back(kartoMatrix(r*eigen(ps[i])*r.transpose()));}
 auto equiv=intersect(left(g,anchor),transformed,rotated);check(logChart(left(g,f.mean),equiv.mean).value.norm()<1e-8,"CI global coordinate equivariance");
 // Equal information in the common chart and different means: neutral tie, no first winner.
 Pose2 zero;Pose2Vector equalmeans{Pose2(-.1,0,0),Pose2(.1,0,0)};std::vector<Matrix3> eq;
 for(const auto&m:equalmeans){auto j=logChart(zero,m).jacobian;eq.push_back(kartoMatrix(j.inverse()*p*j.inverse().transpose()));}
 auto tie=intersect(zero,equalmeans,eq);near(tie.mean.GetX(),0,1e-10,"CI equal covariance neutral tie");check((tie.chart_covariance-p).norm()<1e-10,"CI equal covariance no P/n");
 // Exact SE(2) right-reference change and full covariance transport.
 Pose2 reference(.8,-.3,.4);Pose2Vector changed;std::vector<Matrix3> changedCov;
 auto right=[&](const Pose2& pose){const Eigen::Vector2d t=rotation(pose.GetHeading())*Eigen::Vector2d(reference.GetX(),reference.GetY());return Pose2(pose.GetX()+t(0),pose.GetY()+t(1),angle(pose.GetHeading()+reference.GetHeading()));};
 for(size_t i=0;i<means.size();++i){changed.push_back(right(means[i]));Mat j=Mat::Identity();j.topRightCorner<2,1>()=rotation(means[i].GetHeading())*Eigen::Vector2d(-reference.GetY(),reference.GetX());changedCov.push_back(kartoMatrix(j*eigen(ps[i])*j.transpose()));}
 auto reref=intersect(right(anchor),changed,changedCov);check(logChart(right(f.mean),reref.mean).value.norm()<1e-8,"CI right reference mean equivariance");
 Mat adj=Mat::Identity();adj.topLeftCorner<2,2>()=rotation(-reference.GetHeading());adj.topRightCorner<2,1>()=rotation(-reference.GetHeading())*Eigen::Vector2d(-reference.GetY(),reference.GetX());
 check((reref.chart_covariance-adj*f.chart_covariance*adj.transpose()).norm()<1e-8,"CI right reference covariance congruence");
 bool rejected=false;try{intersect(zero,{Pose2(0,0,KT_PI)},{kartoMatrix(p)});}catch(const std::exception&){rejected=true;}check(rejected,"chart cut rejects");
 std::cout<<"{\"test\":\"algebra\",\"passed\":true,\"full_jacobian_max_error\":"<<worst<<",\"ci_duplicate_idempotent\":true,\"permutation_invariant\":true,\"neutral_equal_information_tie\":true,\"chart_cut_rejected\":true}\n";
}
void consistentGaussianFusion(){
 std::mt19937 rng(94017);std::normal_distribution<double> normal(0,1);Mat p=Mat::Zero();p.diagonal()<<.01,.02,.0004;Mat l=p.llt().matrixL();
 for(double rho:{0.,.8,1.}){int ciCoverage=0,independentCoverage=0;Vec bias=Vec::Zero();const int trials=4000;
  for(int k=0;k<trials;++k){auto draw=[&](){return Vec(l*Vec(normal(rng),normal(rng),normal(rng)));};
   const Vec common=draw();const Vec ea=std::sqrt(rho)*common+std::sqrt(1-rho)*draw(),eb=std::sqrt(rho)*common+std::sqrt(1-rho)*draw();
   Pose2Vector means{expChart(Pose2(),ea),expChart(Pose2(),eb)};std::vector<Matrix3> inputs;
   for(const auto&m:means){const Mat j=logChart(Pose2(),m).jacobian;inputs.push_back(kartoMatrix(j.inverse()*p*j.inverse().transpose()));}
   const auto fusion=intersect(Pose2(),means,inputs);const Vec error=logChart(Pose2(),fusion.mean).value;bias+=error;
   const double ci=(error.transpose()*fusion.chart_covariance.inverse()*error)(0),ind=(error.transpose()*(.5*p).inverse()*error)(0);
   ciCoverage+=ci<=7.814727903;independentCoverage+=ind<=7.814727903;
  }
  const double coverage=double(ciCoverage)/trials;check(coverage>.93,"CI conservative coverage with calibrated Gaussian inputs");
  std::cout<<"{\"test\":\"calibrated_gaussian_fusion\",\"shared_correlation\":"<<rho<<",\"trials\":"<<trials<<",\"ci_coverage95\":"<<coverage<<",\"independent_model_coverage95\":"<<double(independentCoverage)/trials<<",\"bias_norm\":"<<(bias/trials).norm()<<",\"passed\":true}\n";
 }
}
LaserRangeFinder* laser(const std::string&name,double arm){auto*l=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name(name));l->SetMinimumRange(.05);l->SetMaximumRange(20);l->SetRangeThreshold(8);l->SetMinimumAngle(-1.5);l->SetMaximumAngle(1.5);l->SetAngularResolution(.02);l->SetOffsetPose(Pose2(arm,.1*arm,.07));SensorManager::GetInstance()->RegisterSensor(l);return l;}
std::unique_ptr<LocalizedRangeScan> geometry(LaserRangeFinder*l,bool corridor,double noise,Vec disturbance,std::mt19937& rng,int64_t stamp){
 std::normal_distribution<double> normal(0,noise);RangeReadingsVector ranges;PointVectorDouble endpoints,origins;std::vector<int64_t>stamps;std::vector<kt_int32u> indices;std::vector<uint8_t>states;
 auto o=l->GetOffsetPose();
 for(int i=0;i<151;++i){double a=-1.5+.02*i,r;
  if(corridor){r=1.8/std::max(1e-12,std::abs(sin(a)));}
  else r=3.5+.7*sin(3*a)+.4*cos(7*a)+.2*sin(13*a);
  r+=normal(rng);const bool noReturn=r>=20;r=std::min(r,20.);ranges.push_back(r);Eigen::Vector2d point(o.GetX()+r*cos(a+o.GetHeading()),o.GetY()+r*sin(a+o.GetHeading()));
  Eigen::Vector2d from(o.GetX(),o.GetY());point=rotation(disturbance(2))*point+disturbance.head<2>();from=rotation(disturbance(2))*from+disturbance.head<2>();
  endpoints.emplace_back(point(0),point(1));origins.emplace_back(from(0),from(1));stamps.push_back(stamp+i*1000000);indices.push_back(i);states.push_back(noReturn?2:1);}
 auto s=std::make_unique<LocalizedRangeScan>(l->GetName(),ranges,LocalizedRangeScan::LegacyRawTag{});s->SetRawSensorLimits(.05,20);s->SetScanTimeProvenance(stamp,stamp,"synthetic-native.v1","seeded range and shared rigid perturbations");s->SetDeskewGeometry(endpoints,origins,stamps,indices,states);s->SetCorrectedPose(Pose2());s->SetOdometricPose(Pose2());return s;
}
void lifecycle(){
 auto*l=laser("owner_lifecycle",.305);std::mt19937 rng(731);auto a=geometry(l,false,0,Vec::Zero(),rng,1000000000);auto b=geometry(l,false,0,Vec::Zero(),rng,2000000000);
 a->SetStateId(0);b->SetStateId(1);a->SetUniqueId(0);b->SetUniqueId(1);
 Mapper mapper;MapperGraph graph(&mapper,8);auto*va=graph.AddVertex(a.get());auto*vb=graph.AddVertex(b.get());
 Evidence e;e.context="same-query-reference-prior-stage";e.views.push_back(std::make_shared<ViewPayload>(b->GetMeasurementIdentityPayload()));e.raw_sources={"sensor:stamp"};
 auto*edge=new Edge<LocalizedRangeScan>(va,vb);Matrix3 p;p.SetToIdentity();auto*info=new LinkInfo(Pose2(),Pose2(),p);info->SetEvidence(e);edge->SetLabel(info);graph.Graph<LocalizedRangeScan>::AddEdge(edge);
 check(graph.FindMeasurementOwner(e)==edge,"live owner found");Evidence clone=e;clone.views[0]=std::make_shared<ViewPayload>(*e.views[0]);check(graph.FindMeasurementOwner(clone)==edge,"full payload equality not pointer identity");
 Evidence different=e;different.context+="other-prior";check(!graph.FindMeasurementOwner(different),"different prior not duplicate");different=e;different.views[0]=std::make_shared<ViewPayload>("different geometry");check(!graph.FindMeasurementOwner(different),"different geometry not duplicate");
 std::stringstream bytes;{boost::archive::binary_oarchive ar(bytes);ar<<e;}Evidence restored;{boost::archive::binary_iarchive ar(bytes);ar>>restored;}check(restored==e,"archive exact evidence roundtrip");
 graph.Graph<LocalizedRangeScan>::RemoveEdge(0);check(!graph.FindMeasurementOwner(e),"removed owner releases identity");graph.Graph<LocalizedRangeScan>::AddEdge(edge);check(graph.FindMeasurementOwner(e)==edge,"same identity can reacquire owner");
 graph.RemoveVertex(l->GetName(),0);check(!graph.FindMeasurementOwner(e),"removed owner node releases identity");graph.Graph<LocalizedRangeScan>::AddVertex(l->GetName(),va);check(graph.FindMeasurementOwner(e)==edge,"same identity after node readd");
 Evidence malformed=e;malformed.raw_sources.clear();check(!malformed.known(),"incomplete roster is unknown");
 bool caught=false;try{info->SetEvidence(malformed);}catch(const std::exception&){caught=true;}check(caught,"setter rejects incomplete evidence");
 malformed=e;malformed.views[0].reset();check(!malformed.known(),"null evidence view rejected");
 malformed=e;malformed.raw_sources[0]="other physical raw source";check(!(malformed==e),"raw source participates in equality");
 LinkInfo legacy;check(legacy.GetEvidenceStatus()=="unknown_legacy","legacy missing evidence stays unknown");
 auto*l2=laser("other_sensor_same_stamp",.305);auto other=geometry(l2,false,0,Vec::Zero(),rng,2000000000);check(other->GetMeasurementIdentityPayload()!=b->GetMeasurementIdentityPayload(),"different sensor same timestamp differs");
 // Graph owns vertices/edges, scans remain outside it.
 graph.Clear();SensorManager::GetInstance()->UnregisterSensor(l);SensorManager::GetInstance()->UnregisterSensor(l2);delete l;delete l2;
 std::cout<<"{\"test\":\"ownership\",\"passed\":true,\"archive_roundtrip\":true,\"live_membership_rebuilt\":true,\"removed_edge_and_node_reacquire\":true,\"legacy_unknown\":true}\n";
}
struct CaptureSolver : ScanSolver {
 IdPoseVector corrections;std::vector<std::string> events;std::set<int> nodes;
 void Compute() override {} void Configure(rclcpp::Node::SharedPtr) override {}
 const IdPoseVector& GetCorrections() const override{return corrections;}
 void RecordMatchDiagnostic(const std::string& e)override{events.push_back(e);}
 void AddNode(Vertex<LocalizedRangeScan>*v)override{nodes.insert(v->GetObject()->GetUniqueId());}
 void RemoveNode(kt_int32s id)override{nodes.erase(id);}
};
void actualOwnership(){
 auto*l=laser("actual_owner",.305);std::mt19937 rng(923);CaptureSolver solver;Mapper mapper;mapper.SetScanSolver(&solver);
 mapper.setParamDoLoopClosing(false);mapper.setParamMinimumTravelDistance(0);mapper.setParamMinimumTravelHeading(0);mapper.setParamMinimumTimeInterval(0);
 std::vector<std::unique_ptr<LocalizedRangeScan>> scans;
 for(int i=0;i<8;++i){scans.push_back(geometry(l,false,0,Vec::Zero(),rng,1000000000LL+i*100000000LL));
  scans.back()->SetOdometricPose(Pose2(.02*i,0,0));check(mapper.Process(scans.back().get()),"actual Mapper Process accepted");}
 auto * graph=mapper.GetGraph();check(graph->GetEdges().size()==7,"one owner per reused sequential/running-chain result");int duplicates=0;
 for(const auto&e:solver.events)if(e.find("measurement_already_owned")!=std::string::npos)++duplicates;
 check(duplicates>0,"actual redundant anchor projections rejected");
 for(auto* edge:graph->GetEdges()){auto*info=dynamic_cast<LinkInfo*>(edge->GetLabel());check(info&&info->GetEvidence().known(),"all real factors carry exact evidence");check(graph->FindMeasurementOwner(info->GetEvidence())==edge,"one live owner");}
 mapper.SaveToFile(output_directory+"/ownership.posegraph");Mapper restored;restored.LoadFromFile(output_directory+"/ownership.posegraph");
 check(restored.GetGraph()->GetEdges().size()==7,"actual mapper archive edge count");
 for(auto* edge:restored.GetGraph()->GetEdges()){auto* info=dynamic_cast<LinkInfo*>(edge->GetLabel());check(restored.GetGraph()->FindMeasurementOwner(info->GetEvidence())==edge,"owner derived after actual mapper archive");}
 // Force an unsupported antipodal chart before commit and verify whole query undo.
 auto rejected=geometry(l,false,0,Vec::Zero(),rng,3000000000LL);auto* manager=mapper.GetMapperSensorManager();manager->AddScan(rejected.get());graph->AddVertex(rejected.get());
 Matrix3 covariance;covariance.SetToIdentity();const size_t oldEdges=graph->GetEdges().size();const size_t oldNodes=solver.nodes.size()-1;
 check(!graph->AddEdges(rejected.get(),covariance,.9,Pose2(0,0,rejected->GetSensorPose().GetHeading()+KT_PI)),"antipodal query rejected");
 check(graph->GetEdges().size()==oldEdges&&solver.nodes.size()==oldNodes,"query rejection commits no edge/node");
 auto next=geometry(l,false,0,Vec::Zero(),rng,4000000000LL);check(mapper.Process(next.get()),"next query accepted after rejection");check(next->GetStateId()==8&&next->GetUniqueId()==8,"failed transaction restores ID allocation");
 mapper.Reset();check(mapper.GetGraph()==nullptr,"Reset releases graph ownership");
 std::cout<<"{\"test\":\"actual_mapper_ownership\",\"passed\":true,\"nodes_before_rejection\":8,\"owner_edges\":7,\"redundant_anchor_projections\":"<<duplicates<<",\"archive_registry_rebuilt\":true,\"chart_rejection_atomic\":true,\"reset_releases_owners\":true}\n";
 SensorManager::GetInstance()->UnregisterSensor(l);delete l;
}
void matcherTrials(){
 std::mt19937 rng(10061);std::normal_distribution<double> n(0,1);
 for(bool corridor:{false,true})for(double arm:{0.,1.2})for(bool shared:{false,true}){
 auto*l=laser(std::string("native_")+(corridor?"corridor":"rich")+std::to_string(arm)+(shared?"shared":"independent"),arm);
 Mapper mapper;std::unique_ptr<ScanMatcher> matcher(ScanMatcher::Create(&mapper,.4,.01,.03,8));
 Vec sum=Vec::Zero();Mat outer=Mat::Zero(),reported=Mat::Zero();int coverage=0;double nees=0;constexpr int trials=40;
 for(int k=0;k<trials;++k){Vec common=shared?Vec(.035*n(rng),.035*n(rng),.012*n(rng)):Vec::Zero();
  auto ref=geometry(l,corridor,.015,Vec::Zero(),rng,1000000000);auto query=geometry(l,corridor,.015,common,rng,2000000000);
  Pose2 frame(1,-.7,k%2?.9:-.4);ref->SetCorrectedPose(frame);
  auto clipped=[&](){return std::max(-2.,std::min(2.,n(rng)));};
  query->SetCorrectedPose(left(frame,Pose2(.06*clipped(),.04*clipped(),.01*clipped())));
  Pose2 mean;Matrix3 covariance;LocalizedRangeScanVector refs{ref.get()};double score=matcher->MatchLidarEvidence(query.get(),refs,mean,covariance,true);
  check(score>0 && std::isfinite(score),"actual native matcher response");
  Pose2 estimate=base(mean,l->GetOffsetPose());Vec err(estimate.GetX()-frame.GetX(),estimate.GetY()-frame.GetY(),angle(estimate.GetHeading()-frame.GetHeading()));
  Mat p=eigen(sensorToBaseCovariance(mean,l->GetOffsetPose(),covariance));Mat rt=Mat::Identity();rt.topLeftCorner<2,2>()=rotation(-frame.GetHeading());err=rt*err;p=rt*p*rt.transpose();check(p.allFinite(),"native transformed covariance finite");
  double q=(err.transpose()*p.inverse()*err)(0);if(q<=7.814727903)++coverage;nees+=q;sum+=err;outer+=err*err.transpose();reported+=p;
 }
 Vec bias=sum/trials;Mat empirical=outer/trials-bias*bias.transpose();Mat average=reported/trials;
 std::cout<<"{\"test\":\"native_matcher_coverage\",\"scene\":\""<<(corridor?"corridor":"rich")<<"\",\"lever_arm_m\":"<<arm<<",\"shared_rigid_disturbance\":"<<(shared?"true":"false")<<",\"trials\":"<<trials<<",\"bias\":["<<bias(0)<<','<<bias(1)<<','<<bias(2)<<"],\"empirical_variance\":["<<empirical(0,0)<<','<<empirical(1,1)<<','<<empirical(2,2)<<"],\"reported_variance\":["<<average(0,0)<<','<<average(1,1)<<','<<average(2,2)<<"],\"coverage95\":"<<double(coverage)/trials<<",\"mean_nees\":"<<nees/trials<<",\"covariance_calibrated\":false}\n";
 SensorManager::GetInstance()->UnregisterSensor(l);delete l;
 }
}
int main(int argc,char**argv){try{if(argc!=2)throw std::runtime_error("usage: model_probe OUTPUT_DIRECTORY");output_directory=argv[1];std::filesystem::create_directories(output_directory);std::cout<<std::setprecision(17);algebra();consistentGaussianFusion();lifecycle();actualOwnership();matcherTrials();return 0;}catch(const Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
