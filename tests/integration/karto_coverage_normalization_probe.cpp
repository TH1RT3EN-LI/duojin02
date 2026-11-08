// Preserve Karto's calibrated coverage-weighted source-roster score scale.
#include <karto_sdk/Mapper.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace karto;
void require(bool value,const char*why){if(!value)throw std::runtime_error(why);}
void near(double a,double b,const char*why,double tolerance=1e-12){require(std::abs(a-b)<tolerance,why);}
LaserRangeFinder* sensor(const char* name,size_t beams){
 auto*laser=LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinder_Custom,Name(name));
 laser->SetMinimumRange(.05);laser->SetMaximumRange(10);laser->SetRangeThreshold(8);
 laser->SetMinimumAngle(-1.5);laser->SetMaximumAngle(-1.5+.02*(beams-1));laser->SetAngularResolution(.02);
 laser->SetOffsetPose(Pose2(.305,0,.02));SensorManager::GetInstance()->RegisterSensor(laser);return laser;
}
std::unique_ptr<LocalizedRangeScan> scan(LaserRangeFinder* laser,size_t roster,size_t hits){
 RangeReadingsVector raw(roster,0);PointVectorDouble points(roster),origins(roster);
 std::vector<int64_t> times(roster);std::vector<kt_int32u> indices(roster);std::vector<uint8_t> states(roster,0);
 for(size_t i=0;i<roster;++i){indices[i]=i;times[i]=1000000000LL+i*1000000LL;
  if(i<hits){raw[i]=4;states[i]=1;const double angle=-1.5+.02*i;origins[i]={.305,0};points[i]={.305+4*cos(angle),4*sin(angle)};}}
 auto result=std::make_unique<LocalizedRangeScan>(laser->GetName(),raw,LocalizedRangeScan::LegacyRawTag{});
 result->SetRawSensorLimits(.05,10);result->SetScanTimeProvenance(1000000000LL,1000000000LL,"coverage-policy-fixture","fixed analytic geometry");
 result->SetDeskewGeometry(points,origins,times,indices,states);result->SetCorrectedPose(Pose2(0,0,0));return result;
}
int main(){try{
 auto*fullSensor=sensor("source_roster_151",151);auto*wideSensor=sensor("source_roster_302",302);
 auto full=scan(fullSensor,151,151);auto sparse=scan(wideSensor,302,151);auto invalid=scan(fullSensor,151,0);
 require(full->GetPointReadings(true).size()==151&&sparse->GetPointReadings(true).size()==151,"same selected finite hit geometry");
 for(size_t i=0;i<151;++i){near(full->GetPointReadings()[i].GetX(),sparse->GetPointReadings()[i].GetX(),"same physical hit x");near(full->GetPointReadings()[i].GetY(),sparse->GetPointReadings()[i].GetY(),"same physical hit y");}
 Mapper mapper;std::unique_ptr<ScanMatcher> matcher(ScanMatcher::Create(&mapper,.3,.01,.03,8));
 LocalizedRangeScanVector reference{full.get()};Pose2 meanFull,meanSparse,meanEmpty;Matrix3 covariance;
 const double fullScore=matcher->MatchScan(full.get(),reference,meanFull,covariance,false,false);
 const double sparseScore=matcher->MatchScan(sparse.get(),reference,meanSparse,covariance,false,false);
 require(std::isfinite(fullScore)&&fullScore>.5,"native reference receives a supported score");
 near(sparseScore,.5*fullScore,"same hits plus invalid roster must halve coverage confidence");
 near(meanFull.GetX(),meanSparse.GetX(),"coverage normalization must not change maximizing candidate x");
 near(meanFull.GetY(),meanSparse.GetY(),"coverage normalization must not change maximizing candidate y");
 near(meanFull.GetHeading(),meanSparse.GetHeading(),"coverage normalization must not change maximizing candidate yaw");
 LocalizedRangeScan empty(fullSensor->GetName(),{},LocalizedRangeScan::LegacyRawTag{});empty.SetRawSensorLimits(.05,10);empty.SetCorrectedPose(Pose2());
 near(matcher->MatchScan(&empty,reference,meanEmpty,covariance,false,false),0,"zero source roster returns safe zero");
 near(matcher->MatchScan(invalid.get(),reference,meanEmpty,covariance,false,false),0,"zero selected finite hits returns safe zero");
 std::cout<<std::setprecision(17)<<"{\"passed\":true,\"normalization\":\"source-beam-roster.coverage.v1\",\"full_source_count\":151,\"sparse_source_count\":302,\"selected_finite_hits_each\":151,\"full_score\":"<<fullScore<<",\"sparse_score\":"<<sparseScore<<",\"ratio\":"<<sparseScore/fullScore<<",\"zero_source_score\":0,\"zero_hit_score\":0}\n";
 SensorManager::GetInstance()->UnregisterSensor(fullSensor);SensorManager::GetInstance()->UnregisterSensor(wideSensor);delete fullSensor;delete wideSensor;return 0;
}catch(const Exception&e){std::cerr<<e.GetErrorMessage()<<'\n';return 1;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
