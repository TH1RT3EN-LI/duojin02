// Compile writer against the frozen Karto v0 header/library and reader against the current version.
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <karto_sdk/Karto.h>
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
#ifdef DUOJIN02_READ_LEGACY
  karto::LocalizedRangeScan scan;
  std::ifstream input(argv[1],std::ios::binary);
  boost::archive::binary_iarchive archive(input);archive >> scan;
  if (scan.HasDeskewGeometry() || scan.GetNumberOfRangeReadings()!=2 ||
    scan.GetCorrectedPose().GetX()!=1.25 || !scan.GetTimeMappingId().empty())
  {throw std::runtime_error("legacy geometry semantics were not preserved");}
  std::cout<<"Karto v0 binary scan reads as current with historical D0 geometry\n";
#else
  karto::LocalizedRangeScan scan(karto::Name("legacy"),{2,3});
  scan.SetCorrectedPose(karto::Pose2(1.25,2.5,0.3));
  std::ofstream output(argv[1],std::ios::binary);
  boost::archive::binary_oarchive archive(output);archive << scan;
#endif
}
