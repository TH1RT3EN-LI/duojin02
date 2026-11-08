// Audit every persisted real scan through instrumented native lookup and clone.
#include <karto_sdk/Mapper.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  try {
    karto::Mapper mapper;
    mapper.LoadFromFile(argv[1]);
    require(mapper.GetGraph() != nullptr, "Missing recorded graph");
    std::unique_ptr<karto::Grid<uint8_t>> grid(karto::Grid<uint8_t>::CreateGrid(2001,2001,.01));
    karto::GridIndexLookup<uint8_t> lookup(grid.get());
    size_t scans=0, beams=0, invalid=0, no_return=0, hits=0, matchable=0, short_finite=0;
    double max_clone_error=0.;
    for (const auto &sensor : mapper.GetGraph()->GetVertices()) {
      for (const auto &vertex : sensor.second) {
        auto *scan=vertex.second->GetObject();
        karto::LocalizedRangeScan clone(*scan,karto::LocalizedRangeScan::MeasurementCloneTag{});
        require(clone.HasDeskewGeometry()==scan->HasDeskewGeometry(),"Clone geometry kind");
        require(clone.GetBeamStampNs()==scan->GetBeamStampNs(),"Clone beam time");
        require(clone.GetOriginalBeamIndices()==scan->GetOriginalBeamIndices(),"Clone source indices");
        require(clone.GetRawScanStampNs()==scan->GetRawScanStampNs(),"Clone source header");
        const auto &points=scan->GetPointReadings(false);
        const auto &copied=clone.GetPointReadings(false);
        require(points.size()==scan->GetNumberOfRangeReadings(),"Point roster size");
        require(copied.size()==points.size(),"Clone point roster size");
        lookup.ComputeOffsets(&clone,0,.01,.01);
        for (size_t i=0; i<points.size(); ++i) {
          ++beams;
          const auto state=scan->GetBeamState(i);
          require(state==clone.GetBeamState(i),"Clone beam state");
          const double raw=scan->GetRangeReadings()[i];
          if (std::isfinite(raw) && raw<.05) ++short_finite;
          if (state==karto::LocalizedRangeScan::BeamState::Invalid) ++invalid;
          else if (state==karto::LocalizedRangeScan::BeamState::NoReturn) ++no_return;
          else ++hits;
          if (scan->IsBeamMatchable(i)) ++matchable;
          else for (unsigned a=0;a<3;++a)
            require(lookup.GetLookupArray(a)->GetArrayPointer()[i]==karto::INVALID_SCAN,
              "Invalid/nonmatchable recorded beam reached lookup");
          if (scan->IsBeamRayTraceable(i)) {
            const auto origin=scan->GetRayOrigin(i);
            require(std::isfinite(origin.GetX()) && std::isfinite(origin.GetY()),"Nonfinite recorded ray origin");
            require(std::isfinite(points[i].GetX()) && std::isfinite(points[i].GetY()),"Nonfinite recorded endpoint");
            max_clone_error=std::max(max_clone_error,(points[i]-copied[i]).Length());
            max_clone_error=std::max(max_clone_error,(origin-clone.GetRayOrigin(i)).Length());
          }
        }
        ++scans;
      }
    }
    require(scans>0 && short_finite>0,"Not the complete recorded map fixture");
    require(max_clone_error<1e-12,"Clone changed physical recorded measurement");
    std::cout << "{\"scans\":"<<scans<<",\"beams\":"<<beams<<",\"invalid\":"<<invalid
      <<",\"no_return\":"<<no_return<<",\"hit\":"<<hits<<",\"matchable\":"<<matchable
      <<",\"recorded_finite_short\":"<<short_finite<<",\"max_clone_error_m\":"<<max_clone_error
      <<",\"native_lookup_angles_per_scan\":3,\"passed\":true}\n";
  } catch (const karto::Exception &e) {
    std::cerr<<e.GetErrorMessage()<<'\n'; return 1;
  } catch (const std::exception &e) {
    std::cerr<<e.what()<<'\n'; return 1;
  }
}
