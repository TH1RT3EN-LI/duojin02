#include "duojin02_navigation/gpu_geometry.hpp"
#include "duojin02_navigation/tracking.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

using namespace duojin02_navigation;
using Clock = std::chrono::steady_clock;

namespace {
double ms(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}
std::string quoted(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') out << '\\';
    if (c == '\n') out << "\\n"; else out << c;
  }
  out << '"';
  return out.str();
}
struct Scene {
  Polygon polygon{{-.214, -.203}, {.213, -.203}, {.213, .203}, {-.214, .203}};
  std::vector<Point> points;
  std::vector<Pose> poses;
};
struct Scenario { std::string name; size_t poses, obstacles; bool early_collision; };
Scene makeScene(const Scenario &scenario, uint64_t seed) {
  Scene scene;
  std::mt19937_64 random(seed);
  std::uniform_real_distribution<double> xy(-.1, .1), far(2., 8.);
  scene.points.reserve(scenario.obstacles);
  scene.poses.reserve(scenario.poses);
  for (size_t i = 0; i < scenario.obstacles; ++i)
    scene.points.push_back({far(random), far(random), .007071067812});
  if (scenario.name == "centered_clearance") {
    for (size_t i = 0; i < scene.points.size(); ++i)
      scene.points[i] = {-.8 + 1.6 * i / scene.points.size() + (seed % 17) * 1e-6,
                         i % 2 ? .25 : -.25, .007071067812};
  }
  if (scenario.early_collision)
    scene.points.front() = {0, 0, .007071067812};
  for (size_t i = 0; i < scenario.poses; ++i)
    scene.poses.push_back({xy(random), xy(random), xy(random)});
  return scene;
}
struct Measurement {
  double build_ms{0}, setup_ms{0}, query_ms{0}, end_to_end_ms{0};
  bool used_cuda{false};
  GeometryResult result;
  GeometryStatistics stats;
};
GeometryResult query(GeometryCompute &backend, const Scene &scene, bool combined) {
  return combined ? backend.evaluateWithClearance(scene.poses, .010, .080)
                  : backend.evaluate(scene.poses, .010);
}
GeometryResult referenceQuery(const Scene &scene, bool combined) {
  GeometryResult result;
  result.collisions.reserve(scene.poses.size());
  if (combined) result.clearances.reserve(scene.poses.size());
  for (const auto &pose : scene.poses) {
    result.collisions.push_back(collision(pose, scene.polygon, scene.points, .010));
    if (combined) result.clearances.push_back(footprintClearance(pose, scene.polygon, scene.points, .080));
  }
  return result;
}
bool sameResult(const GeometryResult &cpu, const GeometryResult &gpu) {
  if (cpu.collisions != gpu.collisions || cpu.clearances.size() != gpu.clearances.size())
    return false;
  for (size_t i = 0; i < cpu.clearances.size(); ++i)
    if (!std::isfinite(cpu.clearances[i]) || !std::isfinite(gpu.clearances[i]) ||
        std::abs(cpu.clearances[i] - gpu.clearances[i]) > 1e-8) return false;
  return true;
}
void distribution(std::ostream &out, std::vector<double> values) {
  const auto raw = values;
  std::sort(values.begin(), values.end());
  auto p = [&](double quantile) {
    return values[std::min(values.size() - 1, static_cast<size_t>(std::ceil(values.size() * quantile)) - 1)];
  };
  out << "{\"samples\":" << values.size() << ",\"mean_ms\":"
      << std::accumulate(values.begin(), values.end(), 0.) / values.size()
      << ",\"p50_ms\":" << p(.50) << ",\"p95_ms\":" << p(.95)
      << ",\"p99_ms\":" << p(.99) << ",\"max_ms\":" << values.back()
      << ",\"raw_ms\":[";
  for (size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << raw[i];
  out << "]}";
}
void measurements(std::ostream &out, const std::vector<Measurement> &samples) {
  out << '{';
  for (const auto &field : {std::string("scene_build"), std::string("scene_setup"),
                            std::string("query"), std::string("end_to_end")}) {
    std::vector<double> values;
    for (const auto &sample : samples)
      values.push_back(field == "scene_build" ? sample.build_ms : field == "scene_setup" ? sample.setup_ms :
                       field == "query" ? sample.query_ms : sample.end_to_end_ms);
    out << (field == "scene_build" ? "" : ",") << quoted(field) << ':';
    distribution(out, values);
  }
  out << ",\"cuda_samples\":" << std::count_if(samples.begin(), samples.end(),
          [](const Measurement &sample) { return sample.used_cuda; });
  out << ",\"scene_uploads\":";
  uint64_t uploads = 0, bytes = 0, rechecks = 0, clearance_rechecks = 0, indexed = 0, candidate_work = 0, cpu_dispatches = 0;
  for (const auto &sample : samples) {
    uploads += sample.stats.scene_uploads;
    bytes += sample.stats.scene_upload_bytes;
    rechecks += sample.stats.boundary_rechecks;
    clearance_rechecks += sample.stats.clearance_rechecks;
    indexed += sample.stats.indexed_batches;
    candidate_work += sample.stats.last_candidate_work;
    cpu_dispatches += sample.stats.candidate_cpu_dispatches;
  }
  out << uploads << ",\"scene_upload_bytes\":" << bytes << ",\"boundary_rechecks\":" << rechecks
      << ",\"clearance_rechecks\":" << clearance_rechecks << ",\"indexed_batches\":" << indexed
      << ",\"candidate_work\":" << candidate_work << ",\"candidate_cpu_dispatches\":" << cpu_dispatches << '}';
}
}

int main(int argc, char **argv) {
  try {
    bool required = false;
    size_t sample_count = 40;
    std::string output;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--require-cuda") required = true;
      else if (arg == "--samples" && i + 1 < argc) sample_count = std::stoul(argv[++i]);
      else if (arg == "--json-out" && i + 1 < argc) output = argv[++i];
      else throw std::runtime_error("Unknown/incomplete benchmark argument: " + arg);
    }
    if (sample_count < 2) throw std::runtime_error("--samples must be at least 2");
    std::ostringstream report;
    report << std::setprecision(9) << "{\"schema_version\":1,\"seed\":20261015,"
           << "\"scope\":\"synchronous geometry API; excludes ROS input/command path\","
           << "\"timing_notes\":\"process_first_call starts after program initialization and excludes OS process launch; cold_backend creates a fresh GeometryCompute in a warmed CUDA context; result availability excludes backend destruction\","
           << "\"reference_scope\":\"unchanged full-scan collision/footprintClearance predicates, scene validation excluded\","
           << "\"backend_order\":\"cyclic rotation of reference/indexed/CUDA; paired index retains identical input seed\","
           << "\"clearance_tolerance\":1e-8,"
           << "\"percentiles\":\"empirical nearest rank; p99 requires larger runs for inference\",";
    const Scenario startup_scene{"startup", 256, 8000, false};
    const auto process_start = Clock::now();
    auto startup_input = makeScene(startup_scene, 20261015);
    const double startup_build_ms = ms(process_start);
    const auto constructor_start = Clock::now();
    GeometryCompute startup({true, 0});
    const double context_ms = ms(constructor_start);
    const auto startup_setup = Clock::now();
    startup.setScene(startup_input.polygon, startup_input.points);
    const double startup_setup_ms = ms(startup_setup);
    const auto startup_query = Clock::now();
    const auto startup_result = query(startup, startup_input, true);
    const double startup_query_ms = ms(startup_query), startup_total_ms = ms(process_start);
    if (required && !startup_result.used_cuda) throw std::runtime_error("Real CUDA execution required: " + startup.deviceName());
    report << "\"device\":" << quoted(startup.deviceName()) << ",\"cuda_required\":" << (required ? "true" : "false")
           << ",\"process_first_call\":{\"scene_build_ms\":" << startup_build_ms
           << ",\"backend_constructor_ms\":" << context_ms << ",\"scene_setup_ms\":" << startup_setup_ms
           << ",\"query_ms\":" << startup_query_ms << ",\"end_to_end_ms\":" << startup_total_ms
           << ",\"used_cuda\":" << (startup_result.used_cuda ? "true" : "false") << "},\"cases\":[";
    bool first = true;
    size_t mismatches = 0, paired_samples = 0;
    double maximum_clearance_error = 0;
    for (const auto &scenario : std::vector<Scenario>{{"early_collision",256,8000,true},
          {"sparse_no_collision",256,8000,false},{"large_no_collision",1600,8000,false},
          {"centered_clearance",256,8000,false},{"empty",256,0,false}}) {
      for (const bool combined : {false, true}) {
        for (const std::string mode : {"warm_scene", "scene_reuse", "scene_refresh", "cold_backend"}) {
          Scene resident = makeScene(scenario, 20261015);
          GeometryCompute cpu({false, 0}), gpu({true, 0});
          cpu.setScene(resident.polygon, resident.points);
          gpu.setScene(resident.polygon, resident.points);
          const auto reference = query(cpu, resident, combined), warm = query(gpu, resident, combined);
          if (!sameResult(reference, warm)) throw std::runtime_error("Warmup CPU/CUDA mismatch");
          std::vector<Measurement> reference_samples, cpu_samples, gpu_samples;
          for (size_t iteration = 0; iteration < sample_count; ++iteration) {
            Measurement pair[3];
            for (int slot = 0; slot < 3; ++slot) {
              // Alternate execution order; both backends receive the same seed and input.
              const int backend_index = (slot + iteration) % 3;
              auto &sample = pair[backend_index];
              const auto begin = Clock::now();
              std::unique_ptr<GeometryCompute> fresh;
              GeometryCompute *backend = backend_index == 2 ? &gpu : backend_index == 1 ? &cpu : nullptr;
              Scene refreshed;
              const Scene *input = &resident;
              if (mode == "scene_reuse" && backend) {
                const auto setup = Clock::now();
                backend->setScene(input->polygon, input->points);
                sample.setup_ms = ms(setup);
              } else if (mode != "warm_scene" && mode != "scene_reuse") {
                const auto build = Clock::now();
                refreshed = makeScene(scenario, 20261016 + iteration);
                sample.build_ms = ms(build);
                input = &refreshed;
                const auto setup = Clock::now();
                if (mode == "cold_backend" && backend_index) {
                  fresh = std::make_unique<GeometryCompute>(GeometryOptions{backend_index == 2, 0});
                  backend = fresh.get();
                }
                if (backend) backend->setScene(input->polygon, input->points);
                sample.setup_ms = ms(setup);
              }
              const auto evaluate = Clock::now();
              const auto before = backend ? backend->statistics() : GeometryStatistics{};
              sample.result = backend ? query(*backend, *input, combined) : referenceQuery(*input, combined);
              if (backend) {
                const auto after = backend->statistics();
                sample.stats.scene_uploads = after.scene_uploads - before.scene_uploads;
                sample.stats.scene_upload_bytes = after.scene_upload_bytes - before.scene_upload_bytes;
                sample.stats.boundary_rechecks = after.boundary_rechecks - before.boundary_rechecks;
                sample.stats.clearance_rechecks = after.clearance_rechecks - before.clearance_rechecks;
                sample.stats.indexed_batches = after.indexed_batches - before.indexed_batches;
                sample.stats.last_candidate_work = after.last_candidate_work;
                sample.stats.candidate_cpu_dispatches = after.candidate_cpu_dispatches - before.candidate_cpu_dispatches;
              }
              sample.query_ms = ms(evaluate);
              sample.end_to_end_ms = ms(begin);
              sample.used_cuda = sample.result.used_cuda;
            }
            if (!sameResult(pair[0].result, pair[1].result) || !sameResult(pair[0].result, pair[2].result)) ++mismatches;
            for (size_t i = 0; i < pair[0].result.clearances.size(); ++i)
              maximum_clearance_error = std::max({maximum_clearance_error,
                  std::abs(pair[0].result.clearances[i] - pair[1].result.clearances[i]),
                  std::abs(pair[0].result.clearances[i] - pair[2].result.clearances[i])});
            if (required && scenario.obstacles && !pair[2].used_cuda) throw std::runtime_error("Required CUDA sample fell back to CPU");
            ++paired_samples;
            reference_samples.push_back(std::move(pair[0]));
            cpu_samples.push_back(std::move(pair[1]));
            gpu_samples.push_back(std::move(pair[2]));
          }
          report << (first ? "" : ",") << "{\"scenario\":" << quoted(scenario.name)
                 << ",\"query\":" << quoted(combined ? "collision_and_clearance" : "collision")
                 << ",\"mode\":" << quoted(mode) << ",\"poses\":" << scenario.poses
                 << ",\"obstacles\":" << scenario.obstacles << ",\"reference_cpu\":";
          measurements(report, reference_samples);
          report << ",\"indexed_cpu\":";
          measurements(report, cpu_samples);
          report << ",\"gpu\":";
          measurements(report, gpu_samples);
          report << '}';
          first = false;
          std::cerr << scenario.name << ' ' << (combined ? "combined" : "collision") << ' ' << mode
                    << " paired=" << sample_count << '\n';
        }
      }
    }
    report << "],\"paired_samples\":" << paired_samples << ",\"mismatches\":" << mismatches
           << ",\"maximum_abs_clearance_error\":" << maximum_clearance_error
           << ",\"status\":" << quoted(mismatches ? "failed" : "passed") << "}\n";
    if (!output.empty()) {
      std::ofstream file(output);
      if (!file) throw std::runtime_error("Cannot open output " + output);
      file << report.str();
    } else std::cout << report.str();
    return mismatches ? 1 : 0;
  } catch (const std::exception &error) {
    std::cerr << "BENCHMARK_FAILED " << error.what() << '\n';
    return 1;
  }
}
