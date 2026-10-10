// Exercise the real solver through public APIs. Compile this translation unit
// with the patched solvers/ceres_solver.cpp, not an older installed plugin.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "solvers/ceres_solver.hpp"

namespace
{
using Pose = Eigen::Vector3d;
using Graph = std::unordered_map<int, Pose>;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::string quoted(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out << '\\' << c;
    } else if (c == '\n') {
      out << "\\n";
    } else if (c == '\r') {
      out << "\\r";
    } else {
      out << c;
    }
  }
  out << '"';
  return out.str();
}

Pose initial_pose(int ordinal)
{
  return Pose(3.0 + 0.7 * ordinal, -2.0 + 0.01 * ordinal, 0.1 + 0.001 * ordinal);
}

Pose difference(const Pose & a, const Pose & b)
{
  const double c = std::cos(a.z());
  const double s = std::sin(a.z());
  return Pose(c * (b.x() - a.x()) + s * (b.y() - a.y()),
    -s * (b.x() - a.x()) + c * (b.y() - a.y()),
    NormalizeAngle(b.z() - a.z()));
}

struct Constraint
{
  int source;
  int target;
  Pose measurement;
  bool active;
};

struct Fixture
{
  rclcpp::Node::SharedPtr node;
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> scans;
  std::map<int, std::unique_ptr<karto::Vertex<karto::LocalizedRangeScan>>> vertices;
  std::vector<std::unique_ptr<karto::Edge<karto::LocalizedRangeScan>>> edges;
  std::vector<Constraint> constraints;
  std::size_t rehashes = 0;
  // Destroy the solver before the Karto objects used to construct its graph.
  std::unique_ptr<solver_plugins::CeresSolver> solver;

  explicit Fixture(const std::string & name)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("ceres_num_threads", 2)});
    node = std::make_shared<rclcpp::Node>("ceres_anchor_" + name, options);
    node->declare_parameter<bool>("debug_logging", true);
    solver = std::make_unique<solver_plugins::CeresSolver>();
    solver->Configure(node);
  }

  void add(int id, int ordinal)
  {
    require(solver->getGraph()->count(id) == 0, "duplicate live fixture node ID");
    auto scan = std::make_unique<karto::LocalizedRangeScan>(
      karto::Name("anchor_probe_laser"), karto::RangeReadingsVector{});
    scan->SetUniqueId(id);
    const Pose pose = initial_pose(ordinal);
    scan->SetCorrectedPose(karto::Pose2(pose.x(), pose.y(), pose.z()));
    auto vertex = std::make_unique<karto::Vertex<karto::LocalizedRangeScan>>(scan.get());
    const auto previous = solver->getGraph()->bucket_count();
    solver->AddNode(vertex.get());
    const auto current = solver->getGraph()->bucket_count();
    if (previous != current) {
      ++rehashes;
      std::cout << "PROBE_REHASH " << id << ' ' << previous << ' ' << current << '\n';
    }
    // Deliberately never reserve or rehash the solver's unordered_map.
    vertices.insert_or_assign(id, std::move(vertex));
    scans.push_back(std::move(scan));
  }

  void connect(int source, int target)
  {
    const Pose measurement(1.0, 0.12, 0.003);
    karto::Matrix3 covariance;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance(row, column) = 0.0;
      }
    }
    covariance(0, 0) = covariance(1, 1) = 0.04;
    covariance(2, 2) = 0.01;
    auto edge = std::make_unique<karto::Edge<karto::LocalizedRangeScan>>(
      vertices.at(source).get(), vertices.at(target).get());
    edge->SetLabel(new karto::LinkInfo(karto::Pose2(0, 0, 0),
      karto::Pose2(measurement.x(), measurement.y(), measurement.z()), covariance));
    solver->AddConstraint(edge.get());
    constraints.push_back({source, target, measurement, true});
    edges.push_back(std::move(edge));
  }

  void remove(int id)
  {
    solver->RemoveNode(id);
    for (auto & constraint : constraints) {
      if (constraint.source == id || constraint.target == id) {constraint.active = false;}
    }
  }

  void disconnect(int source, int target)
  {
    solver->RemoveConstraint(source, target);
    for (auto & constraint : constraints) {
      if ((constraint.source == source && constraint.target == target) ||
        (constraint.source == target && constraint.target == source))
      {
        constraint.active = false;
      }
    }
  }

  void perturb(int id)
  {
    const Pose current = solver->getGraph()->at(id);
    // ModifyNode adds its yaw argument to the previous yaw.
    solver->ModifyNode(id, Pose(current.x() + 0.8, current.y() + 0.4, 0.2));
  }

  double cost(const Graph & graph, int override_id, const Pose & override_pose) const
  {
    double result = 0.0;
    for (const auto & constraint : constraints) {
      if (!constraint.active || !graph.count(constraint.source) || !graph.count(constraint.target)) {
        continue;
      }
      const Pose a = constraint.source == override_id ? override_pose : graph.at(constraint.source);
      const Pose b = constraint.target == override_id ? override_pose : graph.at(constraint.target);
      Pose residual = difference(a, b) - constraint.measurement;
      residual.z() = NormalizeAngle(residual.z());
      result += 0.5 * (residual.head<2>().squaredNorm() / 0.04 +
        residual.z() * residual.z() / 0.01);
    }
    return result;
  }
};

class CaptureStdout
{
public:
  CaptureStdout() : previous_(std::cout.rdbuf(stream_.rdbuf())) {}
  ~CaptureStdout() {std::cout.rdbuf(previous_);}
  std::string str() const {return stream_.str();}

private:
  std::ostringstream stream_;
  std::streambuf * previous_;
};

std::vector<std::string> results;
std::string active_case;

void check_compute(
  Fixture & fixture, const std::string & name, std::optional<int> anchor,
  int expected_parameter_blocks, std::optional<int> expected_reduced = std::nullopt,
  std::optional<int> must_move = std::nullopt)
{
  active_case = name;
  const Graph before = *fixture.solver->getGraph();
  Pose gradient = Pose::Zero();
  if (anchor) {
    const Pose pose = before.at(*anchor);
    for (int component = 0; component < 3; ++component) {
      Pose positive = pose;
      Pose negative = pose;
      positive(component) += 1e-6;
      negative(component) -= 1e-6;
      gradient(component) = (fixture.cost(before, *anchor, positive) -
        fixture.cost(before, *anchor, negative)) / 2e-6;
      require(std::isfinite(gradient(component)) && std::abs(gradient(component)) > 1e-4,
        name + ": anchor must have a nonzero objective gradient in all three coordinates");
    }
  }

  std::string report;
  {
    CaptureStdout capture;
    fixture.solver->Compute();
    report = capture.str();
  }
  std::cout << "PROBE_SUMMARY_BEGIN " << name << '\n' << report <<
    "PROBE_SUMMARY_END " << name << '\n';
  std::smatch match;
  require(std::regex_search(report, match,
    std::regex("Parameter blocks[[:space:]]+([0-9]+)[[:space:]]+([0-9]+)")),
    name + ": real Ceres FullReport is missing parameter-block counts");
  const int original = std::stoi(match[1]);
  const int reduced = std::stoi(match[2]);
  require(original == expected_parameter_blocks, name + ": unexpected original parameter blocks");
  // Ceres preprocessing also eliminates parameter blocks without residuals.
  // Orphan cases therefore need an explicit reduced count; reconnecting those
  // nodes then checks that an old anchor was actually released.
  const int expected_reduced_blocks = expected_reduced.value_or(original - (anchor ? 3 : 0));
  require(reduced == expected_reduced_blocks,
    name + ": unexpected reduced parameter blocks or anchor constant state");
  require(std::regex_search(report, match,
    std::regex("Residual blocks[[:space:]]+([0-9]+)[[:space:]]+([0-9]+)")),
    name + ": real Ceres FullReport is missing residual-block counts");
  const int residual_blocks = std::stoi(match[1]);
  const int reduced_residual_blocks = std::stoi(match[2]);
  int expected_residual_blocks = 0;
  for (const auto & constraint : fixture.constraints) {
    if (constraint.active && before.count(constraint.source) && before.count(constraint.target)) {
      ++expected_residual_blocks;
    }
  }
  require(residual_blocks == expected_residual_blocks &&
    reduced_residual_blocks == expected_residual_blocks,
    name + ": constraints were not added or removed from the real Ceres problem");

  const Graph & graph = *fixture.solver->getGraph();
  const auto & corrections = fixture.solver->GetCorrections();
  require(corrections.size() == graph.size(), name + ": stale or missing corrections");
  std::set<int> corrected_ids;
  for (const auto & correction : corrections) {
    require(graph.count(correction.first) == 1, name + ": correction for a removed node");
    require(corrected_ids.insert(correction.first).second, name + ": duplicate correction ID");
    const Pose pose(correction.second.GetX(), correction.second.GetY(),
      correction.second.GetHeading());
    require(pose.allFinite(), name + ": nonfinite correction");
    require((pose - graph.at(correction.first)).cwiseAbs().maxCoeff() < 1e-12,
      name + ": corrections differ from the solver's public graph");
  }
  double motion = 0.0;
  for (const auto & entry : graph) {
    require(entry.second.allFinite(), name + ": nonfinite graph pose");
    motion = std::max(motion, (entry.second - before.at(entry.first)).norm());
  }
  double anchor_change = 0.0;
  if (anchor) {
    anchor_change = (graph.at(*anchor) - before.at(*anchor)).cwiseAbs().maxCoeff();
    require(anchor_change <= 1e-12, name + ": selected anchor moved under nonzero gradient");
    require(motion > 1e-4, name + ": biased constraints did not move any variable node");
    require(fixture.cost(graph, *anchor, graph.at(*anchor)) <
      fixture.cost(before, *anchor, before.at(*anchor)), name + ": objective did not decrease");
  }
  double required_node_motion = 0.0;
  if (must_move) {
    required_node_motion = (graph.at(*must_move) - before.at(*must_move)).norm();
    require(required_node_motion > 1e-4, name + ": the released former anchor remained fixed");
  }

  std::ostringstream json;
  json << std::setprecision(17) << "{\"case\":" << quoted(name) <<
    ",\"pass\":true,\"node_count\":" << graph.size() <<
    ",\"correction_count\":" << corrections.size() <<
    ",\"original_parameter_blocks\":" << original <<
    ",\"reduced_parameter_blocks\":" << reduced <<
    ",\"residual_blocks\":" << residual_blocks << ",\"anchor_id\":";
  if (anchor) {json << *anchor;} else {json << "null";}
  json << ",\"anchor_max_change\":" << anchor_change <<
    ",\"anchor_gradient\":[" << gradient.x() << ',' << gradient.y() << ',' << gradient.z() <<
    "],\"max_graph_motion\":" << motion << ",\"required_node_motion\":" <<
    required_node_motion << ",\"rehash_count\":" << fixture.rehashes << '}';
  results.push_back(json.str());
  std::cout << "PROBE_RESULT " << results.back() << std::endl;
}

void run_cases()
{
  {
    Fixture fixture("multiple_rehash");
    for (int id = 0; id < 160; ++id) {fixture.add(id, id);}
    require(fixture.rehashes >= 3, "160 nodes must exercise multiple unordered_map rehashes");
    for (int id = 0; id < 159; ++id) {fixture.connect(id, id + 1);}
    check_compute(fixture, "multiple_rehash", 0, 480);
  }
  {
    Fixture fixture("remove_before_compute");
    for (int id = 0; id < 3; ++id) {fixture.add(id, id);}
    fixture.connect(0, 1);
    fixture.connect(1, 2);
    fixture.remove(0);
    check_compute(fixture, "remove_before_compute", 1, 6);
  }
  {
    Fixture fixture("remove_after_compute");
    for (int id = 0; id < 3; ++id) {fixture.add(id, id);}
    fixture.connect(0, 1);
    fixture.connect(1, 2);
    check_compute(fixture, "before_anchor_removal", 0, 9);
    fixture.remove(0);
    fixture.perturb(1);
    check_compute(fixture, "remove_after_compute", 1, 6);
  }
  {
    Fixture fixture("nonzero_first");
    fixture.add(7, 0);
    fixture.add(9, 2);
    fixture.add(0, 1);
    fixture.connect(7, 0);
    fixture.connect(0, 9);
    check_compute(fixture, "retain_first_inserted_nonzero", 7, 9);
    fixture.remove(7);
    fixture.perturb(0);
    check_compute(fixture, "replacement_can_be_zero", 0, 6);
  }
  {
    Fixture fixture("incident_residual_lifecycle");
    for (int id = 0; id < 4; ++id) {fixture.add(id, id);}
    fixture.connect(0, 1);
    fixture.connect(1, 2);
    fixture.connect(2, 3);
    fixture.connect(1, 3);
    check_compute(fixture, "before_incident_residual_removal", 0, 12);
    fixture.remove(0);
    // A stale residual ID would cause a fatal Ceres call here.
    fixture.disconnect(0, 1);
    // A nonincident edge must remain removable through the same public API.
    fixture.disconnect(1, 3);
    fixture.perturb(1);
    check_compute(fixture, "incident_missing_normal_edge_removed", 1, 9);
    fixture.add(0, 0);
    fixture.connect(0, 1);
    fixture.connect(0, 3);
    fixture.perturb(1);
    check_compute(fixture, "readd_same_node_id_and_edge_endpoints", 1, 12);
    fixture.disconnect(0, 1);
    fixture.perturb(1);
    check_compute(fixture, "remove_readded_edge_changes_real_problem", 1, 12);
  }
  {
    Fixture fixture("lowest_neighbor_becomes_orphan");
    for (int id = 0; id < 4; ++id) {fixture.add(id, id);}
    fixture.connect(0, 1);
    fixture.connect(0, 2);
    fixture.connect(2, 3);
    check_compute(fixture, "before_lowest_neighbor_becomes_orphan", 0, 12);
    fixture.remove(0);
    fixture.perturb(2);
    check_compute(fixture, "skip_neighbor_with_blocks_but_no_live_residual", 2, 9, 3);
  }
  {
    Fixture fixture("surviving_anchor_loses_last_edge");
    for (int id = 0; id < 3; ++id) {fixture.add(id, id);}
    fixture.connect(0, 1);
    fixture.connect(1, 2);
    check_compute(fixture, "before_surviving_anchor_loses_last_edge", 0, 9);
    fixture.disconnect(0, 1);
    fixture.perturb(1);
    check_compute(fixture, "surviving_orphan_anchor_replaced", 1, 9, 3);
    fixture.perturb(0);
    fixture.connect(0, 1);
    check_compute(fixture, "reconnected_former_anchor_is_variable", 1, 9, 6, 0);
  }
  {
    Fixture fixture("unconstrained_first");
    fixture.add(0, 20);
    fixture.add(11, 1);
    fixture.add(10, 0);
    fixture.connect(10, 11);
    check_compute(fixture, "unconstrained_first_selects_smallest_eligible", 10, 6);
  }
  {
    Fixture fixture("constraints_arrive_later");
    fixture.add(0, 20);
    fixture.add(11, 1);
    fixture.add(10, 0);
    check_compute(fixture, "no_eligible_parameter_blocks", std::nullopt, 0);
    fixture.connect(10, 11);
    check_compute(fixture, "later_constraints_reselect_anchor", 10, 6);
  }
  {
    Fixture fixture("clear_and_rebuild");
    for (int id = 0; id < 3; ++id) {fixture.add(id, id);}
    fixture.connect(0, 1);
    fixture.connect(1, 2);
    check_compute(fixture, "before_remove_all", 0, 9);
    for (int id = 0; id < 3; ++id) {fixture.remove(id);}
    require(fixture.solver->getGraph()->empty(), "RemoveNode must remove all nodes");
    fixture.add(0, 0);
    fixture.add(1, 1);
    fixture.connect(0, 1);
    check_compute(fixture, "remove_all_then_add_same_ids", 0, 6);
    fixture.solver->Reset();
    require(fixture.solver->getGraph()->empty(), "Reset must empty the public graph");
    require(fixture.solver->GetCorrections().empty(), "Reset must clear corrections");
    fixture.constraints.clear();
    fixture.add(5, 1);
    fixture.add(3, 0);
    fixture.connect(3, 5);
    check_compute(fixture, "reset_rebuild_clears_previous_anchor", 5, 6);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string failure;
  try {
    run_cases();
  } catch (const karto::Exception & error) {
    failure = error.GetErrorMessage();
  } catch (const std::exception & error) {
    failure = error.what();
  }
  rclcpp::shutdown();
  std::ostringstream json;
  json << "{\"test\":\"public_api_ceres_anchor_lifecycle\",\"passed\":" <<
    (failure.empty() ? "true" : "false") << ",\"completed_checks\":" << results.size() <<
    ",\"results\":[";
  for (std::size_t index = 0; index < results.size(); ++index) {
    if (index) {json << ',';}
    json << results[index];
  }
  json << "],\"failure_case\":" << quoted(failure.empty() ? "" : active_case) <<
    ",\"failure\":" << quoted(failure) << '}';
  std::cout << "PROBE_FINAL " << json.str() << std::endl;
  if (argc > 1) {
    std::ofstream output(argv[1]);
    if (!output) {
      std::cerr << "Cannot write probe JSON: " << argv[1] << '\n';
      return 2;
    }
    output << json.str() << '\n';
  }
  return failure.empty() ? 0 : 1;
}
