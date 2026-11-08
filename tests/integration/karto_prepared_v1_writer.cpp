// Fixture generator: compile against the pre-repair installed archive-v1 backend.
#include <fstream>
#include <karto_sdk/Karto.h>
int main(int argc,char **argv){if(argc!=2)return 2;
 auto*laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("v1_fixture"));
 laser->SetMinimumRange(.1);laser->SetMaximumRange(10);laser->SetMinimumAngle(0);laser->SetMaximumAngle(1);laser->SetAngularResolution(1);
 karto::SensorManager::GetInstance()->RegisterSensor(laser);
 karto::LocalizedRangeScan scan(laser->GetName(),{2,3});
 scan.SetDeskewGeometry({{2,0},{0,3}},{{0,0},{0,0}},{1000,2000},{0,1});
 scan.SetScanTimeProvenance(1000,1500,"v1-fixture","frozen pre-repair backend");
 std::ofstream stream(argv[1],std::ios::binary);boost::archive::binary_oarchive archive(stream);archive<<scan;
 return 0;}
