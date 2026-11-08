/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include "nav2_amcl/amcl_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "message_filters/subscriber.h"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include <iomanip>
#include <locale>
#include <sstream>
#include "nav2_amcl/portable_utils.hpp"
#include "nav2_amcl/occupancy_identity.hpp"
#include "nav2_util/validate_messages.hpp"

using namespace std::placeholders;
using rcl_interfaces::msg::ParameterType;
using namespace std::chrono_literals;

namespace nav2_amcl
{
using nav2_util::geometry_utils::orientationAroundZAxis;

AmclNode::AmclNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("amcl", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");
  rcl_interfaces::msg::ParameterDescriptor backend_descriptor;
  backend_descriptor.read_only = true;
  backend_descriptor.description = "AMCL observation execution backend: cpu, auto, or strict cuda";
  declare_parameter("observation_backend", "auto", backend_descriptor);
  declare_parameter("observation_cuda_min_particles", 0, backend_descriptor);
  declare_parameter("observation_log_every", 0, backend_descriptor);
  backend_descriptor.description = "Probability likelihood power; finite and in (0, 1]";
  declare_parameter("likelihood_temperature_beta", 1.0, backend_descriptor);
  backend_descriptor.description = "ESS/N resampling gate; zero preserves interval-only legacy policy";
  declare_parameter("resample_ess_ratio", 0.0, backend_descriptor);
  backend_descriptor.description = "Optional KLD confidence probability; -1 preserves legacy pf_z quantile";
  declare_parameter("kld_confidence_probability", -1.0, backend_descriptor);
  backend_descriptor.description = "Source-stamped PF diagnostics interval; zero disables periodic logging";
  declare_parameter("filter_diagnostics_log_every", 0, backend_descriptor);
  backend_descriptor.description = "Experimental: publish immutable pre-resample statistics; PF inference unchanged";
  declare_parameter("pre_resample_pose_output", false, backend_descriptor);
  backend_descriptor.description = "Opt-in finite-search engineering initializer on global localization request; not a posterior";
  declare_parameter("global_acquisition_enabled", false, backend_descriptor);
  backend_descriptor.description = "Refine a user initial pose within a bounded map-frame search before publishing localization";
  declare_parameter("local_initialization_enabled", false, backend_descriptor);
  backend_descriptor.description = "Half width in metres of the user initial pose search square; all headings are searched";
  declare_parameter("local_initialization_xy_half_width", 2.0, backend_descriptor);
  backend_descriptor.description = "Run owned immutable-anchor searches outside scan callbacks";
  declare_parameter("global_acquisition_async", true, backend_descriptor);
  backend_descriptor.description = "Geometric agreement gate in (0,1]; not a calibrated probability";
  declare_parameter("global_acquisition_min_score", 0.65, backend_descriptor);
  backend_descriptor.description = "Minimum score separation from the certified finite-domain competitor bound; not physical confidence";
  declare_parameter("global_acquisition_min_margin", 0.05, backend_descriptor);
  backend_descriptor.description = "Positive bound-evaluation budget; unresolved search keeps initialization pending";
  declare_parameter("global_acquisition_max_evaluations", 3000000, backend_descriptor);
  backend_descriptor.description = "Experimental fine KDE modes and same-support scores; diagnostic only";
  declare_parameter("fine_mode_diagnostics", false, backend_descriptor);
  declare_parameter("fine_mode_position_bandwidth", 0.01, backend_descriptor);
  declare_parameter("fine_mode_yaw_bandwidth", 0.017453292519943295, backend_descriptor);
  backend_descriptor.description = "Maximum pending scans waiting for complete beam TF support";
  declare_parameter("scan_pending_queue_size", 200, backend_descriptor);
  backend_descriptor.description = "Maximum steady-clock wait for beam TF support, seconds";
  declare_parameter("scan_pending_timeout_sec", 2.0, backend_descriptor);
  scan_geometry_config_ = duojin02_scan_geometry::declareConfig(*this);


  add_parameter(
    "alpha1", rclcpp::ParameterValue(0.2),
    "This is the alpha1 parameter", "These are additional constraints for alpha1");

  add_parameter(
    "alpha2", rclcpp::ParameterValue(0.2),
    "This is the alpha2 parameter", "These are additional constraints for alpha2");

  add_parameter(
    "alpha3", rclcpp::ParameterValue(0.2),
    "This is the alpha3 parameter", "These are additional constraints for alpha3");

  add_parameter(
    "alpha4", rclcpp::ParameterValue(0.2),
    "This is the alpha4 parameter", "These are additional constraints for alpha4");

  add_parameter(
    "alpha5", rclcpp::ParameterValue(0.2),
    "This is the alpha5 parameter", "These are additional constraints for alpha5");

  add_parameter(
    "base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")),
    "Which frame to use for the robot base");

  add_parameter("beam_skip_distance", rclcpp::ParameterValue(0.5));
  add_parameter("beam_skip_error_threshold", rclcpp::ParameterValue(0.9));
  add_parameter("beam_skip_threshold", rclcpp::ParameterValue(0.3));
  add_parameter("do_beamskip", rclcpp::ParameterValue(false));

  add_parameter(
    "global_frame_id", rclcpp::ParameterValue(std::string("map")),
    "The name of the coordinate frame published by the localization system");

  add_parameter(
    "lambda_short", rclcpp::ParameterValue(0.1),
    "Exponential decay parameter for z_short part of model");

  add_parameter(
    "laser_likelihood_max_dist", rclcpp::ParameterValue(2.0),
    "Maximum distance to do obstacle inflation on map, for use in likelihood_field model");

  add_parameter(
    "laser_max_range", rclcpp::ParameterValue(100.0),
    "Maximum scan range to be considered",
    "-1.0 will cause the laser's reported maximum range to be used");

  add_parameter(
    "laser_min_range", rclcpp::ParameterValue(-1.0),
    "Minimum scan range to be considered",
    "-1.0 will cause the laser's reported minimum range to be used");

  add_parameter(
    "laser_model_type", rclcpp::ParameterValue(std::string("likelihood_field")),
    "Which model to use, either beam, likelihood_field, or likelihood_field_prob",
    "Same as likelihood_field but incorporates the beamskip feature, if enabled");

  add_parameter(
    "set_initial_pose", rclcpp::ParameterValue(false),
    "Causes AMCL to set initial pose from the initial_pose* parameters instead of "
    "waiting for the initial_pose message");

  add_parameter(
    "initial_pose.x", rclcpp::ParameterValue(0.0),
    "X coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.y", rclcpp::ParameterValue(0.0),
    "Y coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.z", rclcpp::ParameterValue(0.0),
    "Z coordinate of the initial robot pose in the map frame");

  add_parameter(
    "initial_pose.yaw", rclcpp::ParameterValue(0.0),
    "Yaw of the initial robot pose in the map frame");

  add_parameter(
    "max_beams", rclcpp::ParameterValue(60),
    "How many evenly-spaced beams in each scan to be used when updating the filter");

  add_parameter(
    "max_particles", rclcpp::ParameterValue(2000),
    "Maximum allowed number of particles");

  add_parameter(
    "min_particles", rclcpp::ParameterValue(500),
    "Minimum allowed number of particles");

  add_parameter(
    "odom_frame_id", rclcpp::ParameterValue(std::string("odom")),
    "Which frame to use for odometry");

  add_parameter("pf_err", rclcpp::ParameterValue(0.05));
  add_parameter("pf_z", rclcpp::ParameterValue(0.99));

  add_parameter(
    "recovery_alpha_fast", rclcpp::ParameterValue(0.0),
    "Exponential decay rate for the fast average weight filter, used in deciding when to recover "
    "by adding random poses",
    "A good value might be 0.1");

  add_parameter(
    "recovery_alpha_slow", rclcpp::ParameterValue(0.0),
    "Exponential decay rate for the slow average weight filter, used in deciding when to recover "
    "by adding random poses",
    "A good value might be 0.001");

  add_parameter(
    "resample_interval", rclcpp::ParameterValue(1),
    "Number of filter updates required before resampling");

  add_parameter("robot_model_type", rclcpp::ParameterValue("nav2_amcl::DifferentialMotionModel"));

  add_parameter(
    "save_pose_rate", rclcpp::ParameterValue(0.5),
    "Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter "
    "server, in the variables ~initial_pose_* and ~initial_cov_*. This saved pose will be used "
    "on subsequent runs to initialize the filter",
    "-1.0 to disable");

  add_parameter("sigma_hit", rclcpp::ParameterValue(0.2));

  add_parameter(
    "tf_broadcast", rclcpp::ParameterValue(true),
    "Set this to false to prevent amcl from publishing the transform between the global frame and "
    "the odometry frame");

  add_parameter(
    "transform_tolerance", rclcpp::ParameterValue(1.0),
    "Time with which to post-date the transform that is published, to indicate that this transform "
    "is valid into the future");

  add_parameter(
    "update_min_a", rclcpp::ParameterValue(0.2),
    "Rotational movement required before performing a filter update");

  add_parameter(
    "update_min_d", rclcpp::ParameterValue(0.25),
    "Translational movement required before performing a filter update");

  add_parameter("z_hit", rclcpp::ParameterValue(0.5));
  add_parameter("z_max", rclcpp::ParameterValue(0.05));
  add_parameter("z_rand", rclcpp::ParameterValue(0.5));
  add_parameter("z_short", rclcpp::ParameterValue(0.05));

  add_parameter(
    "always_reset_initial_pose", rclcpp::ParameterValue(false),
    "Requires that AMCL is provided an initial pose either via topic or initial_pose* parameter "
    "(with parameter set_initial_pose: true) when reset. Otherwise, by default AMCL will use the"
    "last known pose to initialize");

  add_parameter(
    "scan_topic", rclcpp::ParameterValue("scan"),
    "Topic to subscribe to in order to receive the laser scan for localization");

  add_parameter(
    "map_topic", rclcpp::ParameterValue("map"),
    "Topic to subscribe to in order to receive the map to localize on");

  add_parameter(
    "first_map_only", rclcpp::ParameterValue(false),
    "Set this to true, when you want to load a new map published from the map_server");
}

AmclNode::~AmclNode()
{
}

nav2_util::CallbackReturn
AmclNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  initParameters();
  RCLCPP_INFO(get_logger(), "AMCL scan_model_contract=%s",
    duojin02_scan_geometry::contractJson(scan_geometry_config_).c_str());
  initTransforms();
  if (!duojin02_scan_geometry::legacyContract(scan_geometry_config_))
  {
    scan_geometry_ = std::make_unique<duojin02_scan_geometry::RosScanGeometry>(
      *this, scan_geometry_config_.tf_history_sec);
    scan_retry_timer_ = create_wall_timer(5ms, [this]() {drainPendingScans();}, callback_group_);
  }
  initParticleFilter();
  initLaserScan();
  initMessageFilters();
  initPubSub();
  initServices();
  initOdometry();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(callback_group_, get_node_base_interface());
  executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Lifecycle publishers must be explicitly activated
  pose_pub_->on_activate();
  particle_cloud_pub_->on_activate();
  estimate_contract_pub_->on_activate();

  first_pose_sent_ = false;

  // Keep track of whether we're in the active state. We won't
  // process incoming callbacks until we are
  active_ = true;

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation = orientationAroundZAxis(initial_pose_yaw_);

    initialPoseReceived(msg);
  } else if (init_pose_received_on_inactive) {
    handleInitialPose(last_published_pose_);
  }

  auto node = shared_from_this();
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AmclNode::dynamicParametersCallback,
      this, std::placeholders::_1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "Deactivating");

  active_ = false;
  pending_scans_.clear();

  // Lifecycle publishers must be explicitly deactivated
  pose_pub_->on_deactivate();
  particle_cloud_pub_->on_deactivate();
  estimate_contract_pub_->on_deactivate();

  // reset dynamic parameter handler
  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  executor_thread_.reset();
  scan_retry_timer_.reset();
  scan_geometry_.reset();
  pending_scans_.clear();

  // Get rid of the inputs first (services and message filter input), so we
  // don't continue to process incoming messages
  global_loc_srv_.reset();
  initial_guess_srv_.reset();
  nomotion_update_srv_.reset();
  executor_thread_.reset();  //  to make sure initial_pose_sub_ completely exit
  initial_pose_sub_.reset();
  laser_scan_connection_.disconnect();
  tf_listener_.reset();  //  listener may access lase_scan_filter_, so it should be reset earlier
  laser_scan_filter_.reset();
  laser_scan_sub_.reset();

  // Map
  map_sub_.reset();  //  map_sub_ may access map_, so it should be reset earlier
  if (map_ != NULL) {
    map_free(map_);
    map_ = nullptr;
  }
  first_map_received_ = false;
  free_space_indices.resize(0);

  // Transforms
  tf_broadcaster_.reset();
  tf_buffer_.reset();

  // PubSub
  pose_pub_.reset();
  particle_cloud_pub_.reset();
  estimate_contract_pub_.reset();

  // Odometry
  motion_model_.reset();

  // Particle Filter
  pf_free(pf_);
  pf_ = nullptr;
  global_acquisition_.reset();
  global_acquisition_pending_ = false;
  global_tracking_confirmation_pending_ = false;
  acquisition_consumed_scan_ = false;
  acquisition_consumed_stamp_ns_ = std::numeric_limits<int64_t>::min();
  global_acquisition_status_ = "lifecycle_cleanup";
  estimate_contract_.resetObservation();
  fine_modes_valid_ = false;

  // Laser Scan
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
  force_update_ = true;

  if (set_initial_pose_) {
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.x",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.x)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.y",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.y)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.z",
        rclcpp::ParameterValue(last_published_pose_.pose.pose.position.z)));
    set_parameter(
      rclcpp::Parameter(
        "initial_pose.yaw",
        rclcpp::ParameterValue(tf2::getYaw(last_published_pose_.pose.pose.orientation))));
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AmclNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool
AmclNode::checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
{
  rclcpp::Duration elapsed_time = now() - last_time;
  if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
    return true;
  }
  return false;
}

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> AmclNode::free_space_indices;
#endif

bool
AmclNode::getOdomPose(
  geometry_msgs::msg::PoseStamped & odom_pose,
  double & x, double & y, double & yaw,
  const rclcpp::Time & sensor_timestamp, const std::string & frame_id)
{
  // Get the robot's pose
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = nav2_util::strip_leading_slash(frame_id);
  ident.header.stamp = sensor_timestamp;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

  try {
    tf_buffer_->transform(ident, odom_pose, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    ++scan_error_count_;
    if (scan_error_count_ % 20 == 0) {
      RCLCPP_ERROR(
        get_logger(), "(%d) consecutive laser scan transforms failed: (%s)", scan_error_count_,
        e.what());
    }
    return false;
  }

  scan_error_count_ = 0;  // reset since we got a good transform
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

pf_vector_t
AmclNode::uniformPoseGenerator(void * arg)
{
  map_t * map = reinterpret_cast<map_t *>(arg);

#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
  max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
  min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
  max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

  pf_vector_t p;

  RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
  for (;; ) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i, j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
      break;
    }
  }
#endif
  return p;
}

void
AmclNode::globalLocalizationCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  if (!map_ || !pf_ || free_space_indices.empty()) {
    RCLCPP_WARN(get_logger(), "Global initialization rejected: no usable free-space map");
    return;
  }
  global_acquisition_.reset();
  global_acquisition_pending_ = get_parameter("global_acquisition_enabled").as_bool();
  global_tracking_confirmation_pending_ = false;
  acquisition_consumed_scan_ = false;
  acquisition_consumed_stamp_ns_ = std::numeric_limits<int64_t>::min();
  if (global_acquisition_pending_) {
    AcquisitionConfig config;
    config.final_sigma = std::max(2.0 * sigma_hit_, map_->scale);
    config.min_score = get_parameter("global_acquisition_min_score").as_double();
    config.min_margin = get_parameter("global_acquisition_min_margin").as_double();
    config.max_evaluations = get_parameter("global_acquisition_max_evaluations").as_int();
    global_acquisition_ = std::make_unique<AcquisitionExecutor>(config,
      get_parameter("global_acquisition_async").as_bool());
    global_acquisition_status_ = "awaiting_first_scan";
    initial_pose_is_known_ = false;
    latest_tf_valid_ = false;
    first_pose_sent_ = false;
    sent_first_transform_ = false;
    estimate_contract_.resetObservation();
    fine_modes_valid_ = false;
    pf_init_ = false;
    RCLCPP_INFO(get_logger(), "AMCL acquisition status=awaiting_first_scan "
      "score_semantics=finite_domain_geometric_not_physical_confidence "
      "final_sigma_m=%.17g confirmations=3 async=%d "
      "min_score=%.17g min_margin=%.17g max_bound_evaluations=%llu",
      config.final_sigma, get_parameter("global_acquisition_async").as_bool(),
      config.min_score, config.min_margin,
      static_cast<unsigned long long>(config.max_evaluations));
    publishEstimateContract();
    return;
  }
  global_acquisition_status_ = "compatibility_uniform";
  RCLCPP_INFO(get_logger(), "Initializing with uniform distribution");

  pf_init_model(
    pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
    reinterpret_cast<void *>(map_));
  estimate_contract_.resetObservation();
  fine_modes_valid_ = false;
  RCLCPP_INFO(get_logger(), "Global initialisation done!");
  initial_pose_is_known_ = true;
  pf_init_ = false;
}

void
AmclNode::initialPoseReceivedSrv(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> req,
  std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response>/*res*/)
{
  initialPoseReceived(std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(req->pose));
}

// force nomotion updates (amcl updating without requiring motion)
void
AmclNode::nomotionUpdateCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
{
  RCLCPP_INFO(get_logger(), "Requesting no-motion update");
  force_update_ = true;
}

void
AmclNode::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  RCLCPP_INFO(get_logger(), "initialPoseReceived");

  if (!nav2_util::validateMsg(*msg)) {
    RCLCPP_ERROR(get_logger(), "Received initialpose message is malformed. Rejecting.");
    return;
  }
  if (nav2_util::strip_leading_slash(msg->header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
      nav2_util::strip_leading_slash(msg->header.frame_id).c_str(),
      global_frame_id_.c_str());
    return;
  }
  // Overriding last published pose to initial pose
  last_published_pose_ = *msg;

  if (!active_) {
    init_pose_received_on_inactive = true;
    RCLCPP_WARN(
      get_logger(), "Received initial pose request, "
      "but AMCL is not yet in the active state");
    return;
  }
  handleInitialPose(*msg);
}

void
AmclNode::handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  geometry_msgs::msg::TransformStamped tx_odom;
  try {
    rclcpp::Time rclcpp_time = now();
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    // Check if the transform is available
    tx_odom = tf_buffer_->lookupTransform(
      base_frame_id_, tf2_ros::fromMsg(msg.header.stamp),
      base_frame_id_, tf2_time, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if (sent_first_transform_) {
      RCLCPP_WARN(get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    }
    tf2::impl::Converter<false, true>::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  tf2::Transform tx_odom_tf2;
  tf2::impl::Converter<true, false>::convert(tx_odom.transform, tx_odom_tf2);

  tf2::Transform pose_old;
  tf2::impl::Converter<true, false>::convert(msg.pose.pose, pose_old);

  tf2::Transform pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame

  RCLCPP_INFO(
    get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
    now().nanoseconds() * 1e-9,
    pose_new.getOrigin().x(),
    pose_new.getOrigin().y(),
    tf2::getYaw(pose_new.getRotation()));

  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  constexpr int pose_axes[] = {0, 1, 5};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * pose_axes[i] + pose_axes[j]];
    }
  }

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  global_acquisition_.reset();
  global_acquisition_pending_ = false;
  global_tracking_confirmation_pending_ = false;
  global_acquisition_status_ = "external_initial_pose";
  acquisition_consumed_stamp_ns_ = std::numeric_limits<int64_t>::min();
  pf_init_ = false;
  init_pose_received_on_inactive = false;
  initial_pose_is_known_ = true;
  estimate_contract_.resetObservation();
  fine_modes_valid_ = false;
  if (get_parameter("local_initialization_enabled").as_bool()) {
    AcquisitionConfig config;
    config.local_domain = true;
    config.local_center = {pf_init_pose_mean.v[0], pf_init_pose_mean.v[1], pf_init_pose_mean.v[2]};
    config.local_xy_half_width = get_parameter("local_initialization_xy_half_width").as_double();
    config.final_sigma = std::max(2.0 * sigma_hit_, map_->scale);
    config.min_score = get_parameter("global_acquisition_min_score").as_double();
    config.min_margin = get_parameter("global_acquisition_min_margin").as_double();
    config.max_evaluations = get_parameter("global_acquisition_max_evaluations").as_int();
    global_acquisition_ = std::make_unique<AcquisitionExecutor>(config,
      get_parameter("global_acquisition_async").as_bool());
    global_acquisition_pending_ = true;
    global_acquisition_status_ = "local_awaiting_first_scan";
    initial_pose_is_known_ = false;
    latest_tf_valid_ = false;
    first_pose_sent_ = false;
    sent_first_transform_ = false;
    RCLCPP_INFO(get_logger(), "Local initial pose refinement started: map x=%.3f y=%.3f yaw=%.3f "
      "xy_half_width=%.3f m, all headings; waiting for distinct scans, no map pose published",
      config.local_center.x, config.local_center.y, config.local_center.yaw, config.local_xy_half_width);
    publishEstimateContract();
  }
}

void
AmclNode::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!active_ || !first_map_received_) {return;}
  if (!scan_geometry_) {
    processLaserScan(laser_scan, nullptr, now().nanoseconds(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    return;
  }
  const auto capacity = static_cast<size_t>(get_parameter("scan_pending_queue_size").as_int());
  if (pending_scans_.size() >= capacity) {
    RCLCPP_WARN(get_logger(), "AMCL scan rejected source_stamp_ns=%ld reason=pending_queue_full",
      rclcpp::Time(laser_scan->header.stamp).nanoseconds());
    return;
  }
  pending_scans_.push_back({laser_scan, std::chrono::steady_clock::now(), now().nanoseconds()});
  drainPendingScans();
}

void AmclNode::drainPendingScans()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!active_ || !scan_geometry_) {return;}
  while (!pending_scans_.empty()) {
    const auto & pending = pending_scans_.front();
    duojin02_scan_geometry::TimedScan geometry;
    try {
      if (scan_geometry_config_.enabled) {
        geometry = scan_geometry_->prepare(
          *pending.scan, scan_geometry_config_, odom_frame_id_, base_frame_id_);
      } else {
        geometry = scan_geometry_->prepareReference(
          *pending.scan, scan_geometry_config_, odom_frame_id_, base_frame_id_);
      }
    } catch (const duojin02_scan_geometry::Error & error) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - pending.received).count();
      using duojin02_scan_geometry::Failure;
      if ((error.reason == Failure::MissingFuture || error.reason == Failure::MissingFrame) &&
        elapsed < get_parameter("scan_pending_timeout_sec").as_double()) {return;}
      RCLCPP_WARN(get_logger(), "AMCL scan rejected source_stamp_ns=%ld reason=%s wait_sec=%.6f",
        rclcpp::Time(pending.scan->header.stamp).nanoseconds(), error.what(), elapsed);
      pending_scans_.pop_front();
      continue;
    }
    auto scan = pending.scan;
    const auto receipt_ros = pending.received_ros_ns;
    const auto receipt_steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
      pending.received.time_since_epoch()).count();
    pending_scans_.pop_front();
    processLaserScan(scan, &geometry, receipt_ros, receipt_steady);
  }
}

void AmclNode::processLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan,
  const duojin02_scan_geometry::TimedScan * geometry,
  int64_t received_ros_ns, int64_t received_steady_ns)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  const auto raw_stamp_ns = rclcpp::Time(laser_scan->header.stamp).nanoseconds();
  try {
    const auto reference_stamp_ns = duojin02_scan_geometry::referenceStampNs(
      *laser_scan, scan_geometry_config_);
    if (reference_stamp_ns != raw_stamp_ns) {
      auto referenced = std::make_shared<sensor_msgs::msg::LaserScan>(*laser_scan);
      referenced->header.stamp = duojin02_scan_geometry::stampMsg(reference_stamp_ns);
      laser_scan = referenced;
    }
  } catch (const duojin02_scan_geometry::Error & error) {
    RCLCPP_WARN(get_logger(), "AMCL scan rejected source_stamp_ns=%ld reason=%s",
      raw_stamp_ns, error.what());
    return;
  }

  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}
  if (!first_map_received_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(get_logger(), "Waiting for map....");
      last_time_printed_msg_ = now();
    }
    return;
  }

  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  last_laser_received_ts_ = now();
  int laser_index = -1;
  geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    if (!addNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if (geometry) {
    const auto & reference = geometry->odom_from_base_reference;
    pose.v[0] = reference.x;
    pose.v[1] = reference.y;
    pose.v[2] = reference.yaw;
    latest_odom_pose_.header.stamp = laser_scan->header.stamp;
    latest_odom_pose_.header.frame_id = odom_frame_id_;
    latest_odom_pose_.pose.position.x = reference.x;
    latest_odom_pose_.pose.position.y = reference.y;
    latest_odom_pose_.pose.orientation = orientationAroundZAxis(reference.yaw);
  } else if (!getOdomPose(
      latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
      laser_scan->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  if (global_acquisition_pending_ && global_acquisition_) {
    // Record before laser preparation: a scan with valid odometry can still
    // fail later observation preparation, after ordinary PF motion would run.
    global_acquisition_->recordOdometry({{pose.v[0], pose.v[1], pose.v[2]},
      rclcpp::Time(laser_scan->header.stamp).nanoseconds()});
  }

  if (global_acquisition_ && !global_acquisition_pending_ &&
    rclcpp::Time(laser_scan->header.stamp).nanoseconds() <= acquisition_consumed_stamp_ns_)
  {
    RCLCPP_WARN(get_logger(), "AMCL_ACQUISITION raw_stamp_ns=%ld reference_stamp_ns=%ld "
      "state=consumed_source_rejected pose_published=0 scan_pf_likelihood_applied=0",
      raw_stamp_ns, rclcpp::Time(laser_scan->header.stamp).nanoseconds());
    return;
  }
  if (global_tracking_confirmation_pending_) {force_update_ = true;}
  pf_vector_t delta = pf_vector_zero();
  bool force_publication = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }

    force_publication = true;
    resample_count_ = 0;
  } else {
    // Set the laser update flags
    if (shouldUpdateFilter(pose, delta)) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
    if (lasers_update_[laser_index] && !global_acquisition_pending_) {
      motion_model_->odometryUpdate(pf_, pose, delta);
    }
    force_update_ = false;
  }

  bool resampled = false;
  bool filter_updated = false;
  std::shared_ptr<const PosteriorSnapshot> output_snapshot;

  // If the robot has moved, update the filter
  if (global_acquisition_pending_) {lasers_update_[laser_index] = true;}
  if (lasers_update_[laser_index]) {
    acquisition_consumed_scan_ = false;
    const auto observation_count_before = pf_->diagnostics.update_count;
    const bool observation_processed = updateFilter(laser_index, laser_scan, pose,
      scan_geometry_config_.enabled ? geometry : nullptr, raw_stamp_ns);
    if (global_acquisition_pending_ || acquisition_consumed_scan_) {
      // Initialization owns this scan. It is never applied again as a PF
      // likelihood, and no arbitrary/previous PF pose or TF is published.
      pf_odom_pose_ = pose;
      publishEstimateContract();
      return;
    }
    if (global_tracking_confirmation_pending_) {
      // A geometric seed is not a normal PF observation. In particular an
      // old PF_SENSOR_OK left across pf_init must not authorize publication.
      acquisition_consumed_stamp_ns_ = std::max(acquisition_consumed_stamp_ns_,
        rclcpp::Time(laser_scan->header.stamp).nanoseconds());
      if (!observation_processed || pf_->diagnostics.status != PF_SENSOR_OK ||
        pf_->diagnostics.update_count <= observation_count_before)
      {
        pf_odom_pose_ = pose;  // motion already consumed, even on sensor failure
        force_update_ = true;
        lasers_update_[laser_index] = true;
        global_acquisition_status_ = "awaiting_first_valid_pf_update";
        RCLCPP_WARN(get_logger(), "AMCL_ACQUISITION_TRACKING raw_stamp_ns=%ld "
          "reference_stamp_ns=%ld state=retry_first_valid_pf_update "
          "sensor_return=%d fresh_pf_update=%d pf_status=%s pose_published=0 motion_reference_consumed=1",
          raw_stamp_ns, rclcpp::Time(laser_scan->header.stamp).nanoseconds(), observation_processed,
          pf_->diagnostics.update_count > observation_count_before,
          pf_sensor_status_name(pf_->diagnostics.status));
        publishEstimateContract();
        return;
      }
      global_tracking_confirmation_pending_ = false;
      global_acquisition_status_ = "tracking_after_valid_pf_update";
      initial_pose_is_known_ = true;
      RCLCPP_INFO(get_logger(), "AMCL_ACQUISITION_TRACKING raw_stamp_ns=%ld "
        "reference_stamp_ns=%ld state=first_valid_pf_update publication_authorized=1",
        raw_stamp_ns, rclcpp::Time(laser_scan->header.stamp).nanoseconds());
    }
    filter_updated = true;
    if (!observation_processed) {
      // Motion already advanced the particles. Consume its odometry reference
      // even when input preparation fails, so the next scan cannot apply it twice.
      pf_odom_pose_ = pose;
      lasers_update_[laser_index] = false;
      RCLCPP_WARN(get_logger(), "AMCL observation rejected raw_stamp_ns=%ld "
        "reason=input_preparation_failed motion_prior_preserved=1", raw_stamp_ns);
    }
    if (observation_processed && pf_->diagnostics.status == PF_SENSOR_OK) {
      const auto available_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      estimate_contract_.recordAccepted(laser_scan_frame_id + ":" + std::to_string(raw_stamp_ns),
        rclcpp::Time(laser_scan->header.stamp).nanoseconds(), received_ros_ns,
        received_steady_ns, available_ns);
    }
    const double observation_ess = pf_->diagnostics.ess;

    if (get_parameter("pre_resample_pose_output").as_bool()) {
      output_snapshot = buildPosteriorSnapshot(pf_);
    }

    // Resample the particles
    if (observation_processed && pf_->diagnostics.status == PF_SENSOR_OK &&
      !(++resample_count_ % resample_interval_) &&
      pf_should_resample(pf_, get_parameter("resample_ess_ratio").as_double()) == 1)
    {
      pf_update_resample(pf_, reinterpret_cast<void *>(map_));
      resampled = true;
    } else {
      pf_recompute_stats(pf_);
    }

    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);
    const auto log_every = get_parameter("filter_diagnostics_log_every").as_int();
    if (observation_processed && log_every > 0 && pf_->diagnostics.update_count % log_every == 0) {
      const auto & d = pf_->diagnostics;
      double largest_cluster_mass = 0.0;
      for (int cluster = 0; cluster < set->cluster_count; ++cluster) {
        largest_cluster_mass = std::max(largest_cluster_mass, set->clusters[cluster].weight);
      }
      const char * diversity = resampled && d.unique_parents == 1 ? "single_parent" :
        (observation_ess <= 1.0 + 1e-9 ? "posterior_point_mass" : "nondegenerate_weights");
      RCLCPP_INFO(get_logger(),
        "AMCL filter raw_stamp_ns=%ld source_stamp_ns=%ld update=%llu status=%s logZ=%.17g "
        "log_min=%.17g log_max=%.17g observation_ess=%.17g ess=%.17g samples=%d resampled=%d "
        "resample_input=%d resample_output=%d bins=%d second_bin=%d "
        "kld_raw=%.17g kld_limit=%d stop=%s parents=%d duplicates=%.17g "
        "lineages=%d recovery_samples=%d clusters=%d beta=%.17g covariance=selected_cluster "
        "prior_ess=%.17g prior_max_weight=%.17g posterior_max_weight=%.17g "
        "posterior_entropy=%.17g observation_input=%d largest_cluster_mass=%.17g diversity=%s",
        raw_stamp_ns, rclcpp::Time(laser_scan->header.stamp).nanoseconds(), d.update_count,
        pf_sensor_status_name(d.status), d.log_evidence, d.log_weight_min, d.log_weight_max,
        observation_ess, d.ess, set->sample_count, static_cast<int>(resampled),
        d.resample_input_count, d.resample_output_count, d.occupied_bins, d.second_bin_sample,
        d.kld_unclipped_limit, d.kld_limit, pf_resample_stop_name(d.stop_reason),
        d.unique_parents, d.duplicate_fraction, d.distinct_lineages, d.random_injections,
        set->cluster_count, get_parameter("likelihood_temperature_beta").as_double(),
        d.prior_ess, d.prior_max_weight, d.posterior_max_weight, d.posterior_entropy,
        d.observation_input_count, largest_cluster_mass, diversity);
      if (const auto * observation = lasers_[laser_index]->observationStats()) {
        RCLCPP_INFO(get_logger(),
          "AMCL residual raw_stamp_ns=%ld source_stamp_ns=%ld update=%llu "
          "raw_beams=%llu selected_beams=%llu valid_beams=%llu effective_beams=%llu "
          "log_factor_min=%.17g log_factor_max=%.17g distance_p50_m=%.17g "
          "distance_p90_m=%.17g distance_p99_m=%.17g random_floor_fraction=%.17g "
          "representative_particle=%d scope=highest_posterior_candidate mapping_id=%s deskew=%d "
          "effective_source_indices=%s excluded_source_reasons=%s "
          "selection_reason_schema=0:included,1:sensor_invalid,2:no_return,3:algorithm_min,4:algorithm_max,5:beam_skip "
          "clock_evidence=uncalibrated beam_order_evidence=original_index_assumed motion_model=%s "
          "beam_aggregation=%s beam_noise_assumption=%s",
          raw_stamp_ns, rclcpp::Time(laser_scan->header.stamp).nanoseconds(), d.update_count,
          static_cast<unsigned long long>(observation->raw_beams),
          static_cast<unsigned long long>(observation->beams),
          static_cast<unsigned long long>(observation->valid_beams),
          static_cast<unsigned long long>(observation->effective_beams),
          observation->log_factor_min, observation->log_factor_max, observation->distance_p50,
          observation->distance_p90, observation->distance_p99, observation->random_floor_fraction,
          observation->diagnostic_particle, scan_geometry_config_.mapping_id.c_str(),
          static_cast<int>(scan_geometry_config_.enabled),
          observation->effective_source_indices.c_str(), observation->excluded_source_reasons.c_str(),
          duojin02_scan_geometry::motionModelJson(scan_geometry_config_).c_str(),
          sensor_model_type_ == "likelihood_field_prob" ? "product_potential" : "additive_cubic_score",
          sensor_model_type_ == "likelihood_field_prob" ?
          "conditional_factorization_given_fixed_trajectory" : "no_joint_independence_claim");
      }
    }

    if (!force_update_) {
      publishParticleCloud(set);
    }
  }
  if (filter_updated || resampled || force_publication || !first_pose_sent_) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps;
    int max_weight_hyp = -1;
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp, output_snapshot.get())) {
      estimate_contract_.state_reference_stamp_ns =
        rclcpp::Time(laser_scan->header.stamp).nanoseconds();
      publishAmclPose(laser_scan, hyps, max_weight_hyp);
      calculateMaptoOdomTransform(laser_scan, hyps, max_weight_hyp, geometry);

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    if (tf_broadcast_ == true) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(laser_scan->header.stamp) +
        transform_tolerance_;
      sendMapToOdomTransform(transform_expiration);
    }
  }
  publishEstimateContract();
}

bool AmclNode::addNewScanner(
  int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::string & laser_scan_frame_id,
  geometry_msgs::msg::PoseStamped & laser_pose)
{
  lasers_.emplace_back(createLaserObject());
  lasers_update_.push_back(true);
  laser_index = frame_to_laser_.size();

  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = laser_scan_frame_id;
  ident.header.stamp = rclcpp::Time();
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try {
    tf_buffer_->transform(ident, laser_pose, base_frame_id_, transform_tolerance_);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      get_logger(), "Couldn't transform from %s to %s, "
      "even though the message notifier is in use: (%s)",
      laser_scan->header.frame_id.c_str(),
      base_frame_id_.c_str(), e.what());
    return false;
  }

  pf_vector_t laser_pose_v;
  laser_pose_v.v[0] = laser_pose.pose.position.x;
  laser_pose_v.v[1] = laser_pose.pose.position.y;
  // laser mounting angle gets computed later -> set to 0 here!
  laser_pose_v.v[2] = 0;
  lasers_[laser_index]->SetLaserPose(laser_pose_v);
  frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  return true;
}

bool AmclNode::shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta)
{
  delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
  delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
  delta.v[2] = angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]);

  // See if we should update the filter
  bool update = fabs(delta.v[0]) > d_thresh_ ||
    fabs(delta.v[1]) > d_thresh_ ||
    fabs(delta.v[2]) > a_thresh_;
  update = update || force_update_;
  return update;
}

void AmclNode::applyAcquisitionMotion(const AcquisitionDispatch & dispatch)
{
  pf_vector_t reference{{dispatch.anchor.odom.x, dispatch.anchor.odom.y,
      dispatch.anchor.odom.yaw}};
  int64_t reference_stamp = dispatch.anchor.reference_stamp_ns;
  size_t updates = 0;
  for (size_t i = 0; i < dispatch.motion_history.size(); ++i) {
    const auto & sample = dispatch.motion_history[i];
    pf_vector_t current{{sample.odom.x, sample.odom.y, sample.odom.yaw}};
    pf_vector_t delta{{current.v[0] - reference.v[0], current.v[1] - reference.v[1],
        angleutils::angle_diff(current.v[2], reference.v[2])}};
    const bool ordinary_gate = std::abs(delta.v[0]) > d_thresh_ ||
      std::abs(delta.v[1]) > d_thresh_ || std::abs(delta.v[2]) > a_thresh_;
    const bool final_tail = i + 1 == dispatch.motion_history.size();
    const bool nonzero = delta.v[0] != 0 || delta.v[1] != 0 || delta.v[2] != 0;
    if (nonzero && (ordinary_gate || final_tail)) {
      motion_model_->odometryUpdate(pf_, current, delta);
      RCLCPP_INFO(get_logger(), "AMCL_ACQUISITION_MOTION anchor_reference_stamp_ns=%ld "
        "from_reference_stamp_ns=%ld reference_stamp_ns=%ld "
        "odom_x=%.17g odom_y=%.17g odom_yaw=%.17g "
        "delta_x=%.17g delta_y=%.17g delta_yaw=%.17g ordinary_gate=%d final_tail=%d "
        "semantics=existing_scan_gate_prediction_plus_final_tail",
        dispatch.anchor.reference_stamp_ns, reference_stamp, sample.reference_stamp_ns,
        current.v[0], current.v[1], current.v[2], delta.v[0], delta.v[1], delta.v[2],
        ordinary_gate, final_tail);
      reference = current;
      reference_stamp = sample.reference_stamp_ns;
      ++updates;
    }
  }
  RCLCPP_INFO(get_logger(), "AMCL_ACQUISITION_MOTION_SUMMARY anchor_reference_stamp_ns=%ld "
    "history_samples=%zu applied_updates=%zu semantics=existing_scan_gate_prediction_plus_final_tail",
    dispatch.anchor.reference_stamp_ns, dispatch.motion_history.size(), updates);
}

bool AmclNode::updateFilter(
  const int & laser_index,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const pf_vector_t & pose,
  const duojin02_scan_geometry::TimedScan * geometry, int64_t raw_stamp_ns)
{
  nav2_amcl::LaserData ldata;
  ldata.laser = lasers_[laser_index].get();
  ldata.range_count = laser_scan->ranges.size();
  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.
  geometry_msgs::msg::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = laser_scan->header.stamp;
  min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
  min_q.quaternion = orientationAroundZAxis(laser_scan->angle_min);

  inc_q.header = min_q.header;
  inc_q.quaternion = orientationAroundZAxis(laser_scan->angle_min + laser_scan->angle_increment);
  try {
    if (!geometry) {
    tf_buffer_->transform(min_q, min_q, base_frame_id_);
    tf_buffer_->transform(inc_q, inc_q, base_frame_id_);
    }
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(
      get_logger(), "Unable to transform min/max laser angles into base frame: %s",
      e.what());
    return false;
  }
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  RCLCPP_DEBUG(
    get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
    angle_increment);

  // Apply range min/max thresholds, if the user supplied them
  if (laser_max_range_ > 0.0) {
    ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
  } else {
    ldata.range_max = laser_scan->range_max;
  }
  double range_min;
  if (laser_min_range_ > 0.0) {
    range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
  } else {
    range_min = laser_scan->range_min;
  }

  // The LaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];
  if (geometry) {
    if (geometry->beams.size() != static_cast<size_t>(ldata.range_count)) {
      throw std::runtime_error("prepared scan geometry changed original beam indexing");
    }
    ldata.endpoints = new double[ldata.range_count][2];
    ldata.measurement_states.reserve(ldata.range_count);
    ldata.selection_reasons.reserve(ldata.range_count);
  }
  for (int i = 0; i < ldata.range_count; i++) {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (laser_scan->ranges[i] <= range_min) {
      ldata.ranges[i][0] = ldata.range_max;
    } else {
      ldata.ranges[i][0] = laser_scan->ranges[i];
    }
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
    if (geometry) {
      const auto & beam = geometry->beams[i];
      if (beam.original_index != static_cast<size_t>(i)) {
        throw std::runtime_error("prepared scan changed original beam identity");
      }
      ldata.measurement_states.push_back(static_cast<uint8_t>(beam.state));
      using State = duojin02_scan_geometry::BeamState;
      using Selection = nav2_amcl::LaserData::SelectionReason;
      Selection selection;
      if (beam.state == State::Invalid) {selection = Selection::SensorInvalid;}
      else if (beam.state == State::NoReturn) {selection = Selection::NoReturn;}
      else if (beam.raw_range <= range_min) {selection = Selection::BelowAlgorithmMinimum;}
      else if (beam.raw_range >= ldata.range_max) {selection = Selection::AtOrAboveAlgorithmMaximum;}
      else {selection = Selection::Included;}
      ldata.selection_reasons.push_back(selection);
      // Invalid measurements have no coordinates. The legacy contiguous ABI
      // receives a finite neutral placeholder, guarded by authoritative state.
      const auto endpoint = beam.valid() ? beam.endpointPoint() : duojin02_scan_geometry::Point{};
      ldata.endpoints[i][0] = endpoint.x;
      ldata.endpoints[i][1] = endpoint.y;
    }
  }
  if (global_acquisition_pending_ && global_acquisition_) {
    acquisition_consumed_scan_ = true;
    acquisition_consumed_stamp_ns_ = std::max(acquisition_consumed_stamp_ns_,
      rclcpp::Time(laser_scan->header.stamp).nanoseconds());
    ldata.validatePrepared();
    std::vector<AcquisitionPoint> points;
    std::ostringstream selected_sources;
    const int step = std::max(1, static_cast<int>(std::ceil(
      ldata.range_count / static_cast<double>(max_beams_))));
    const auto & extrinsic = lasers_[laser_index]->laserPose();
    for (int i = 0; i < ldata.range_count; i += step) {
      if (!ldata.acceptsHit(i)) {continue;}
      if (!points.empty()) {selected_sources << ',';}
      selected_sources << i;
      if (ldata.endpoints) {points.push_back({ldata.endpoints[i][0], ldata.endpoints[i][1]});}
      else {
        // Raw bearings already include laser yaw via the transform above.
        points.push_back({extrinsic.v[0] + ldata.ranges[i][0] * std::cos(ldata.ranges[i][1]),
          extrinsic.v[1] + ldata.ranges[i][0] * std::sin(ldata.ranges[i][1])});
      }
    }
    const auto start = std::chrono::steady_clock::now();
    AcquisitionInput input;
    input.points = points;
    input.odom = {pose.v[0], pose.v[1], pose.v[2]};
    input.raw_stamp_ns = raw_stamp_ns;
    input.reference_stamp_ns = rclcpp::Time(laser_scan->header.stamp).nanoseconds();
    input.frame = laser_scan->header.frame_id;
    input.selected_source_indices = selected_sources.str();
    const auto dispatch = global_acquisition_->advance(*map_, std::move(input));
    const auto & result = dispatch.search;
    global_acquisition_status_ = result.status;
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    std::ostringstream modes;
    modes << std::setprecision(17);
    for (size_t i = 0; i < result.modes.size(); ++i) {
      if (i) {modes << ';';}
      const auto & mode = result.modes[i];
      modes << mode.pose.x << ',' << mode.pose.y << ',' << mode.pose.yaw << ',' << mode.score;
    }
    RCLCPP_INFO(get_logger(), "AMCL_ACQUISITION raw_stamp_ns=%ld reference_stamp_ns=%ld state=%s "
      "observed_scans=%d confirmations=%d searched=%d exhaustive_lattice=%d "
      "evaluations=%llu beams=%zu best_score=%.17g runner_up_score=%.17g margin=%.17g "
      "elapsed_ms=%.17g modes=%s source_frame=%s selected_source_indices=%s "
      "completed_search=%d completed_certificate=%d search_in_flight=%d anchor_raw_stamp_ns=%ld "
      "anchor_reference_stamp_ns=%ld anchor_source_frame=%s anchor_selected_source_indices=%s "
      "anchor_beams=%zu anchor_age_source_sec=%.17g anchor_search_ms=%.17g "
      "search_complete=%d certificate_valid=%d competitor_upper_bound=%.17g "
      "certified_gap=%.17g bound_evaluations=%llu frontier_size=%zu "
      "lattice_xy_step=%.17g lattice_yaw_step=%.17g "
      "search_domain=canonical_integer_cell_lattice modes_reference=anchor "
      "score_semantics=finite_domain_geometric_not_physical_confidence "
      "pose_published=0 scan_pf_likelihood_applied=0",
      raw_stamp_ns, rclcpp::Time(laser_scan->header.stamp).nanoseconds(), result.status.c_str(),
      result.observed_scans, result.confirmations, result.searched, result.exhaustive_lattice,
      static_cast<unsigned long long>(result.evaluations), points.size(), result.best_score,
      result.runner_up_score, result.margin, elapsed_ms, modes.str().c_str(),
      laser_scan->header.frame_id.c_str(), selected_sources.str().c_str(),
      dispatch.completed_search, dispatch.completed_certificate, dispatch.in_flight, dispatch.anchor.raw_stamp_ns,
      dispatch.anchor.reference_stamp_ns, dispatch.anchor.frame.empty() ? "none" : dispatch.anchor.frame.c_str(),
      dispatch.anchor.selected_source_indices.empty() ? "none" : dispatch.anchor.selected_source_indices.c_str(),
      dispatch.anchor.points.size(),
      dispatch.completed_search ? (rclcpp::Time(laser_scan->header.stamp).nanoseconds() -
      dispatch.anchor.reference_stamp_ns) * 1e-9 : 0.0, dispatch.search_ms,
      result.search_complete, result.certificate_valid, result.competitor_upper_bound,
      result.certified_gap, static_cast<unsigned long long>(result.evaluations),
      static_cast<size_t>(result.frontier_size), result.lattice_xy_step, result.lattice_yaw_step);
    if (dispatch.completed_certificate && result.acquired) {
      const auto & chosen = result.modes.front().pose;
      pf_vector_t mean{{chosen.x, chosen.y, chosen.yaw}};
      pf_matrix_t covariance = pf_matrix_zero();
      // Tracking search envelope, not a calibrated covariance inferred from
      // geometric scores. The first subsequent scan gets the normal PF update.
      const double spread = std::max(2.0 * sigma_hit_, map_->scale);
      covariance.m[0][0] = covariance.m[1][1] = spread * spread;
      covariance.m[2][2] = 0.017453292519943295 * 0.017453292519943295;
      pf_init(pf_, mean, covariance);
      // Compose the original discrete motion kernels. A single aggregated
      // delta is not equivalent: existing alpha*d^2 noise depends on the
      // partition. This preserves that software model, not its calibration.
      applyAcquisitionMotion(dispatch);
      pf_init_ = true;
      resample_count_ = 0;
      initial_pose_is_known_ = false;
      global_acquisition_pending_ = false;
      global_tracking_confirmation_pending_ = true;
      global_acquisition_status_ = "awaiting_first_valid_pf_update";
      force_update_ = true;
      // Keep the next laser flag true even with zero odometry displacement.
      for (auto && update : lasers_update_) {update = true;}
      estimate_contract_.resetObservation();
      fine_modes_valid_ = false;
    } else {lasers_update_[laser_index] = false;}
    pf_odom_pose_ = pose;
    return true;
  }
  const bool sensor_updated = lasers_[laser_index]->sensorUpdate(
    pf_, reinterpret_cast<nav2_amcl::LaserData *>(&ldata));
  if (get_parameter("fine_mode_diagnostics").as_bool()) {
    fine_modes_valid_ = false;
    fine_mean_score_ = fine_mode_score_ = NAN;
    fine_scored_mean_pose_ = fine_scored_mode_pose_ = {{NAN, NAN, NAN}};
    if (pf_->diagnostics.status == PF_SENSOR_OK) {
      const auto coarse = buildPosteriorSnapshot(pf_);
      fine_modes_ = resolveFineModes(pf_->sets[pf_->current_set],
        get_parameter("fine_mode_position_bandwidth").as_double(),
        get_parameter("fine_mode_yaw_bandwidth").as_double());
      if (auto * model = dynamic_cast<LikelihoodFieldModelProb *>(lasers_[laser_index].get())) {
        const auto best = std::max_element(coarse->clusters.begin(), coarse->clusters.end(),
          [](const auto & a, const auto & b) {return a.weight < b.weight;});
        if (best != coarse->clusters.end()) {
          fine_scored_mean_pose_ = best->mean;
          fine_mean_score_ = model->scorePoseOnLastSupport(ldata, fine_scored_mean_pose_);
        }
        if (fine_modes_.selected >= 0) {
          fine_scored_mode_pose_ = fine_modes_.modes[fine_modes_.selected].mean;
          fine_mode_score_ = model->scorePoseOnLastSupport(ldata, fine_scored_mode_pose_);
        }
      }
      fine_modes_valid_ = fine_modes_.selected >= 0;
    }
  }
  lasers_update_[laser_index] = false;
  pf_odom_pose_ = pose;
  return !global_tracking_confirmation_pending_ || sensor_updated;
}

void
AmclNode::publishParticleCloud(const pf_sample_set_t * set)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {return;}
  auto cloud_with_weights_msg = std::make_unique<nav2_msgs::msg::ParticleCloud>();
  cloud_with_weights_msg->header.stamp = this->now();
  cloud_with_weights_msg->header.frame_id = global_frame_id_;
  cloud_with_weights_msg->particles.resize(set->sample_count);

  for (int i = 0; i < set->sample_count; i++) {
    cloud_with_weights_msg->particles[i].pose.position.x = set->samples[i].pose.v[0];
    cloud_with_weights_msg->particles[i].pose.position.y = set->samples[i].pose.v[1];
    cloud_with_weights_msg->particles[i].pose.position.z = 0;
    cloud_with_weights_msg->particles[i].pose.orientation = orientationAroundZAxis(
      set->samples[i].pose.v[2]);
    cloud_with_weights_msg->particles[i].weight = set->samples[i].weight;
  }

  particle_cloud_pub_->publish(std::move(cloud_with_weights_msg));
}

bool
AmclNode::getMaxWeightHyp(
  std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
  int & max_weight_hyp, const PosteriorSnapshot * snapshot)
{
  // Read out the current hypotheses
  double max_weight = 0.0;
  const int cluster_count = snapshot ? static_cast<int>(snapshot->clusters.size()) :
    pf_->sets[pf_->current_set].cluster_count;
  hyps.resize(cluster_count);
  for (int hyp_count = 0; hyp_count < cluster_count; hyp_count++)
  {
    double weight;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (snapshot) {
      const auto & cluster = snapshot->clusters[hyp_count];
      weight = cluster.weight; pose_mean = cluster.mean; pose_cov = cluster.covariance;
    } else if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
      RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
      return false;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;

    if (hyps[hyp_count].weight > max_weight) {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }

  if (max_weight > 0.0) {
    const bool chart_valid = snapshot ? snapshot->clusters[max_weight_hyp].angular_chart_valid :
      pf_->sets[pf_->current_set].clusters[max_weight_hyp].angular_chart_valid != 0;
    if (!chart_valid) {
      RCLCPP_WARN(get_logger(), "AMCL pose rejected: highest-mass cluster has no narrow yaw chart");
      return false;
    }
    RCLCPP_DEBUG(
      get_logger(), "Max weight pose: %.3f %.3f %.3f",
      hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      hyps[max_weight_hyp].pf_pose_mean.v[2]);

    max_weight_hyps = hyps[max_weight_hyp];
    return true;
  }
  return false;
}

void
AmclNode::publishAmclPose(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(
        get_logger(), "AMCL cannot publish a pose or update the transform. "
        "Please set the initial pose...");
      last_time_printed_msg_ = now();
    }
    return;
  }

  auto p = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  // Fill in the header
  p->header.frame_id = global_frame_id_;
  p->header.stamp = laser_scan->header.stamp;
  // Copy in the pose
  p->pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
  p->pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
  p->pose.pose.orientation = orientationAroundZAxis(hyps[max_weight_hyp].pf_pose_mean.v[2]);
  // Mean and full local covariance describe the same selected cluster.
  constexpr int pose_axes[] = {0, 1, 5};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      p->pose.covariance[6 * pose_axes[i] + pose_axes[j]] =
        hyps[max_weight_hyp].pf_pose_cov.m[i][j];
    }
  }
  const bool finite = std::all_of(p->pose.covariance.begin(), p->pose.covariance.end(),
    [](double value) {return std::isfinite(value);}) &&
    std::isfinite(p->pose.pose.position.x) && std::isfinite(p->pose.pose.position.y) &&
    std::isfinite(hyps[max_weight_hyp].pf_pose_mean.v[2]);
  if (finite) {
    RCLCPP_DEBUG(get_logger(), "Publishing pose");
    last_published_pose_ = *p;
    first_pose_sent_ = true;
    pose_pub_->publish(std::move(p));
  } else {
    RCLCPP_WARN(
      get_logger(), "AMCL covariance or pose is non-finite, likely due to an invalid "
      "configuration or faulty sensor measurements! Pose is not available!");
  }

  RCLCPP_DEBUG(
    get_logger(), "New pose: %6.3f %6.3f %6.3f",
    hyps[max_weight_hyp].pf_pose_mean.v[0],
    hyps[max_weight_hyp].pf_pose_mean.v[1],
    hyps[max_weight_hyp].pf_pose_mean.v[2]);
}

void
AmclNode::calculateMaptoOdomTransform(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp,
  const duojin02_scan_geometry::TimedScan * geometry)
{
  // subtracting base to odom from map to base and send map to odom instead
  geometry_msgs::msg::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
    tf2::Transform tmp_tf(q, tf2::Vector3(
        hyps[max_weight_hyp].pf_pose_mean.v[0],
        hyps[max_weight_hyp].pf_pose_mean.v[1],
        0.0));

    if (geometry) {
      const auto & reference = geometry->odom_from_base_reference;
      tf2::Quaternion reference_rotation;
      reference_rotation.setRPY(0.0, 0.0, reference.yaw);
      const tf2::Transform odom_from_base(reference_rotation,
        tf2::Vector3(reference.x, reference.y, 0.0));
      latest_tf_ = odom_from_base * tmp_tf.inverse();
      latest_tf_valid_ = true;
      return;
    }

    geometry_msgs::msg::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException & e) {
    RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform: (%s)", e.what());
    return;
  }

  tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;
}

void
AmclNode::sendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
{
  // AMCL will update transform only when it has knowledge about robot's initial position
  if (!initial_pose_is_known_) {return;}
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
  estimate_contract_.tf_stamp_ns = rclcpp::Time(tmp_tf_stamped.header.stamp).nanoseconds();
}

nav2_amcl::Laser *
AmclNode::createLaserObject()
{
  const auto backend = get_parameter("observation_backend").as_string();
  const auto min_particles = get_parameter("observation_cuda_min_particles").as_int();
  const auto log_every = get_parameter("observation_log_every").as_int();
  std::unique_ptr<nav2_amcl::Laser> laser;
  if (sensor_model_type_ == "beam") {
    laser = std::make_unique<nav2_amcl::BeamModel>(
      z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, 0.0, max_beams_, map_);
    RCLCPP_INFO(get_logger(), "AMCL beam model uses native CPU observation");
    laser->configureObservationBackend("cpu", 0, log_every);
  } else {
    if (sensor_model_type_ == "likelihood_field_prob") {
      laser = std::make_unique<nav2_amcl::LikelihoodFieldModelProb>(
        z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, do_beamskip_,
        beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_, max_beams_, map_);
    } else {
      laser = std::make_unique<nav2_amcl::LikelihoodFieldModel>(
        z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, max_beams_, map_);
    }
    laser->configureObservationBackend(backend, min_particles, log_every);
  }
  laser->setLikelihoodTemperatureBeta(get_parameter("likelihood_temperature_beta").as_double());
  RCLCPP_INFO(get_logger(),
    "AMCL observation configured model=%s backend=%s beam_aggregation=%s beam_noise_assumption=%s",
    sensor_model_type_.c_str(), backend.c_str(),
    sensor_model_type_ == "likelihood_field_prob" ? "product_potential" : "additive_cubic_score",
    sensor_model_type_ == "likelihood_field_prob" ?
    "conditional_factorization_given_fixed_trajectory" : "no_joint_independence_claim");
  return laser.release();
}

void
AmclNode::initParameters()
{
  const double hp = get_parameter("fine_mode_position_bandwidth").as_double();
  const double ha = get_parameter("fine_mode_yaw_bandwidth").as_double();
  if (!std::isfinite(hp) || !std::isfinite(ha) || hp <= 0.0 || ha <= 0.0 || ha > M_PI) {
    throw std::invalid_argument("fine mode bandwidths must be positive finite, yaw <= pi");
  }

  double save_pose_rate;
  double tmp_tol;

  get_parameter("alpha1", alpha1_);
  get_parameter("alpha2", alpha2_);
  get_parameter("alpha3", alpha3_);
  get_parameter("alpha4", alpha4_);
  get_parameter("alpha5", alpha5_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("beam_skip_distance", beam_skip_distance_);
  get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);
  get_parameter("beam_skip_threshold", beam_skip_threshold_);
  get_parameter("do_beamskip", do_beamskip_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("lambda_short", lambda_short_);
  get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  get_parameter("laser_max_range", laser_max_range_);
  get_parameter("laser_min_range", laser_min_range_);
  get_parameter("laser_model_type", sensor_model_type_);
  const auto observation_backend = get_parameter("observation_backend").as_string();
  ObservationBackend::validate(observation_backend);
  if (get_parameter("global_acquisition_enabled").as_bool() ||
    get_parameter("local_initialization_enabled").as_bool()) {
    if (sensor_model_type_ != "likelihood_field_prob") {
      throw std::invalid_argument("global acquisition currently requires likelihood_field_prob beam selection");
    }
    const auto beams = get_parameter("max_beams").as_int();
    const double sigma = get_parameter("sigma_hit").as_double();
    if (beams < 2 || beams > std::numeric_limits<int>::max() ||
      !std::isfinite(sigma) || sigma <= 0 || !std::isfinite(2 * sigma))
    {throw std::invalid_argument("global acquisition requires max_beams>=2 and positive finite sigma_hit");}
  }
  {
    AcquisitionConfig config;
    config.min_score = get_parameter("global_acquisition_min_score").as_double();
    config.min_margin = get_parameter("global_acquisition_min_margin").as_double();
    const auto budget = get_parameter("global_acquisition_max_evaluations").as_int();
    if (budget <= 0) {throw std::invalid_argument("global acquisition budget must be positive");}
    config.max_evaluations = budget;
    config.local_domain = get_parameter("local_initialization_enabled").as_bool();
    config.local_xy_half_width = get_parameter("local_initialization_xy_half_width").as_double();
    GlobalAcquisition validation(config);
  }
  if (observation_backend == "cuda" && sensor_model_type_ == "beam") {
    throw std::invalid_argument("strict CUDA does not support the raycast beam model");
  }
  if (get_parameter("observation_cuda_min_particles").as_int() < 0 ||
      get_parameter("observation_log_every").as_int() < 0 ||
      get_parameter("filter_diagnostics_log_every").as_int() < 0) {
    throw std::invalid_argument("AMCL observation threshold/log interval must be nonnegative");
  }
  const double beta = get_parameter("likelihood_temperature_beta").as_double();
  const double ess_ratio = get_parameter("resample_ess_ratio").as_double();
  const double pending_timeout = get_parameter("scan_pending_timeout_sec").as_double();
  if (get_parameter("scan_pending_queue_size").as_int() <= 0 ||
    !std::isfinite(pending_timeout) || pending_timeout <= 0.0) {
    throw std::invalid_argument("scan pending capacity and timeout must be finite and positive");
  }
  if (scan_geometry_config_.enabled && sensor_model_type_ == "beam") {
    throw std::invalid_argument("scan motion compensation requires an endpoint likelihood field model");
  }
  if (!std::isfinite(beta) || beta <= 0.0 || beta > 1.0 ||
      !std::isfinite(ess_ratio) || ess_ratio < 0.0 || ess_ratio > 1.0) {
    throw std::invalid_argument("AMCL beta must be in (0,1], ESS ratio in [0,1]");
  }
  if (sensor_model_type_ != "likelihood_field_prob" && beta != 1.0) {
    throw std::invalid_argument("likelihood_temperature_beta requires likelihood_field_prob");
  }

  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose.x", initial_pose_x_);
  get_parameter("initial_pose.y", initial_pose_y_);
  get_parameter("initial_pose.z", initial_pose_z_);
  get_parameter("initial_pose.yaw", initial_pose_yaw_);
  get_parameter("max_beams", max_beams_);
  get_parameter("max_particles", max_particles_);
  get_parameter("min_particles", min_particles_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("pf_err", pf_err_);
  get_parameter("pf_z", pf_z_);
  if (!std::isfinite(pf_err_) || pf_err_ <= 0.0 || !std::isfinite(pf_z_)) {
    throw std::invalid_argument("AMCL pf_err must be positive finite and pf_z must be finite");
  }
  const auto confidence = get_parameter("kld_confidence_probability").as_double();
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count("kld_confidence_probability") && overrides.count("pf_z")) {
    throw std::invalid_argument("pf_z and kld_confidence_probability are mutually exclusive");
  }
  if (confidence != -1.0) {
    const double quantile = pf_kld_probability_to_z(confidence);
    if (!std::isfinite(quantile)) {
      throw std::invalid_argument("kld_confidence_probability must be finite and in (0,1)");
    }
    pf_z_ = quantile;
    RCLCPP_INFO(get_logger(), "AMCL KLD confidence_probability=%.17g normal_quantile=%.17g",
      confidence, pf_z_);
  } else {
    RCLCPP_INFO(get_logger(), "AMCL KLD legacy pf_z normal_quantile=%.17g", pf_z_);
  }
  get_parameter("recovery_alpha_fast", alpha_fast_);
  get_parameter("recovery_alpha_slow", alpha_slow_);
  if (!std::isfinite(alpha_fast_) || alpha_fast_ < 0.0 || alpha_fast_ > 1.0 ||
    !std::isfinite(alpha_slow_) || alpha_slow_ < 0.0 || alpha_slow_ > 1.0) {
    throw std::invalid_argument("AMCL recovery averaging rates must be finite and in [0,1]");
  }
  get_parameter("resample_interval", resample_interval_);
  get_parameter("robot_model_type", robot_model_type_);
  get_parameter("save_pose_rate", save_pose_rate);
  get_parameter("sigma_hit", sigma_hit_);
  get_parameter("tf_broadcast", tf_broadcast_);
  get_parameter("transform_tolerance", tmp_tol);
  get_parameter("update_min_a", a_thresh_);
  get_parameter("update_min_d", d_thresh_);
  get_parameter("z_hit", z_hit_);
  get_parameter("z_max", z_max_);
  get_parameter("z_rand", z_rand_);
  get_parameter("z_short", z_short_);
  get_parameter("first_map_only", first_map_only_);
  get_parameter("always_reset_initial_pose", always_reset_initial_pose_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("map_topic", map_topic_);

  save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  last_time_printed_msg_ = now();

  // Semantic checks
  if (laser_likelihood_max_dist_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set laser_likelihood_max_dist to be negative,"
      " this isn't allowed so it will be set to default value 2.0.");
    laser_likelihood_max_dist_ = 2.0;
  }
  if (max_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set max_particles to be negative,"
      " this isn't allowed so it will be set to default value 2000.");
    max_particles_ = 2000;
  }

  if (min_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be negative,"
      " this isn't allowed so it will be set to default value 500.");
    min_particles_ = 500;
  }

  if (min_particles_ > max_particles_) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be greater than max particles,"
      " this isn't allowed so max_particles will be set to min_particles.");
    max_particles_ = min_particles_;
  }

  if (resample_interval_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "You've set resample_interval to be zero or negative,"
      " this isn't allowed so it will be set to default value to 1.");
    resample_interval_ = 1;
  }

  if (always_reset_initial_pose_) {
    initial_pose_is_known_ = false;
  }
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
AmclNode::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  double save_pose_rate;
  double tmp_tol;

  int max_particles = max_particles_;
  int min_particles = min_particles_;

  bool reinit_pf = false;
  bool reinit_odom = false;
  bool reinit_laser = false;
  bool reinit_map = false;

  // Validate new numerical contracts before mutating any live configuration.
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (global_acquisition_pending_ || global_tracking_confirmation_pending_) {
      result.successful = false;
      result.reason = "configuration is frozen while global initialization is pending; complete or cancel with initialpose first";
      return result;
    }
    if (get_parameter("global_acquisition_enabled").as_bool() ||
      get_parameter("local_initialization_enabled").as_bool()) {
      if ((param_name == "laser_model_type" &&
        (param_type != ParameterType::PARAMETER_STRING || parameter.as_string() != "likelihood_field_prob")) ||
        (param_name == "max_beams" && (param_type != ParameterType::PARAMETER_INTEGER ||
        parameter.as_int() < 2 || parameter.as_int() > std::numeric_limits<int>::max())) ||
        (param_name == "sigma_hit" && (param_type != ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(parameter.as_double()) || parameter.as_double() <= 0 ||
        !std::isfinite(2 * parameter.as_double()))))
      {
        result.successful = false;
        result.reason = "global acquisition requires likelihood_field_prob, max_beams>=2, positive finite sigma_hit";
        return result;
      }
    }

    if (param_name == "recovery_alpha_fast" || param_name == "recovery_alpha_slow") {
      if (param_type != ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0 ||
        parameter.as_double() > 1.0)
      {
        result.successful = false;
        result.reason = "recovery averaging rates must be finite and in [0,1]";
        return result;
      }
    }

    if (param_name == "pf_z" && get_parameter("kld_confidence_probability").as_double() != -1.0) {
      result.successful = false;
      result.reason = "pf_z cannot change while kld_confidence_probability is enabled";
      return result;
    }
    if (param_name == "pf_z" || param_name == "pf_err") {
      if (param_type != ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(parameter.as_double()) ||
        (param_name == "pf_err" && parameter.as_double() <= 0.0))
      {
        result.successful = false;
        result.reason = "pf_err must be positive finite and pf_z must be finite";
        return result;
      }
    }
  }

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "alpha1") {
        alpha1_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha1_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha1 to be negative,"
            " this isn't allowed, so the alpha1 will be set to be zero.");
          alpha1_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha2") {
        alpha2_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha2_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha2 to be negative,"
            " this isn't allowed, so the alpha2 will be set to be zero.");
          alpha2_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha3") {
        alpha3_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha3_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha3 to be negative,"
            " this isn't allowed, so the alpha3 will be set to be zero.");
          alpha3_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha4") {
        alpha4_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha4_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha4 to be negative,"
            " this isn't allowed, so the alpha4 will be set to be zero.");
          alpha4_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "alpha5") {
        alpha5_ = parameter.as_double();
        //alpha restricted to be non-negative
        if (alpha5_ < 0.0) {
          RCLCPP_WARN(
            get_logger(), "You've set alpha5 to be negative,"
            " this isn't allowed, so the alpha5 will be set to be zero.");
          alpha5_ = 0.0;
        }
        reinit_odom = true;
      } else if (param_name == "beam_skip_distance") {
        beam_skip_distance_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_error_threshold") {
        beam_skip_error_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "beam_skip_threshold") {
        beam_skip_threshold_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "lambda_short") {
        lambda_short_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_likelihood_max_dist") {
        laser_likelihood_max_dist_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_max_range") {
        laser_max_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "laser_min_range") {
        laser_min_range_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "pf_err") {
        pf_err_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "pf_z") {
        pf_z_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "recovery_alpha_fast") {
        alpha_fast_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "recovery_alpha_slow") {
        alpha_slow_ = parameter.as_double();
        reinit_pf = true;
      } else if (param_name == "save_pose_rate") {
        save_pose_rate = parameter.as_double();
        save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
      } else if (param_name == "sigma_hit") {
        sigma_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "transform_tolerance") {
        tmp_tol = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(tmp_tol);
        reinit_laser = true;
      } else if (param_name == "update_min_a") {
        a_thresh_ = parameter.as_double();
      } else if (param_name == "update_min_d") {
        d_thresh_ = parameter.as_double();
      } else if (param_name == "z_hit") {
        z_hit_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_max") {
        z_max_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_rand") {
        z_rand_ = parameter.as_double();
        reinit_laser = true;
      } else if (param_name == "z_short") {
        z_short_ = parameter.as_double();
        reinit_laser = true;
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "base_frame_id") {
        base_frame_id_ = parameter.as_string();
      } else if (param_name == "global_frame_id") {
        global_frame_id_ = parameter.as_string();
      } else if (param_name == "map_topic") {
        map_topic_ = parameter.as_string();
        reinit_map = true;
      } else if (param_name == "laser_model_type") {
        if (parameter.as_string() == "beam" && scan_geometry_config_.enabled) {
          result.successful = false;
          result.reason = "scan motion compensation requires an endpoint likelihood field model";
          return result;
        }
        if (parameter.as_string() == "beam" && get_parameter("observation_backend").as_string() == "cuda") {
          result.successful = false;
          result.reason = "strict CUDA does not support beam model";
          return result;
        }
        sensor_model_type_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "odom_frame_id") {
        odom_frame_id_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "scan_topic") {
        scan_topic_ = parameter.as_string();
        reinit_laser = true;
      } else if (param_name == "robot_model_type") {
        robot_model_type_ = parameter.as_string();
        reinit_odom = true;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == "do_beamskip") {
        do_beamskip_ = parameter.as_bool();
        reinit_laser = true;
      } else if (param_name == "tf_broadcast") {
        tf_broadcast_ = parameter.as_bool();
      } else if (param_name == "set_initial_pose") {
        set_initial_pose_ = parameter.as_bool();
      } else if (param_name == "first_map_only") {
        first_map_only_ = parameter.as_bool();
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "max_beams") {
        max_beams_ = parameter.as_int();
        reinit_laser = true;
      } else if (param_name == "max_particles") {
        max_particles_ = parameter.as_int();
        reinit_pf = true;
      } else if (param_name == "min_particles") {
        min_particles_ = parameter.as_int();
        reinit_pf = true;
      } else if (param_name == "resample_interval") {
        resample_interval_ = parameter.as_int();
      }
    }
  }

  // Checking if the minimum particles is greater than max_particles_
  if (min_particles_ > max_particles_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "You've set min_particles to be greater than max particles,"
      " this isn't allowed.");
    // sticking to the old values
    max_particles_ = max_particles;
    min_particles_ = min_particles;
    result.successful = false;
    return result;
  }

  // Re-initialize the particle filter
  if (reinit_pf) {
    if (pf_ != NULL) {
      pf_free(pf_);
      pf_ = NULL;
    }
    initParticleFilter();
  }

  // Re-initialize the odometry
  if (reinit_odom) {
    motion_model_.reset();
    initOdometry();
  }

  // Re-initialize the lasers and it's filters
  if (reinit_laser) {
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();
    laser_scan_connection_.disconnect();
    laser_scan_filter_.reset();
    laser_scan_sub_.reset();

    initMessageFilters();
  }

  // Re-initialize the map
  if (reinit_map) {
    map_sub_.reset();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
  }

  result.successful = true;
  return result;
}

void
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "AmclNode: A new map was received.");
  if (!nav2_util::validateMsg(*msg)) {
    RCLCPP_ERROR(get_logger(), "Received map message is malformed. Rejecting.");
    return;
  }
  if (first_map_only_ && first_map_received_) {
    return;
  }
  handleMapMessage(*msg);
  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);

  RCLCPP_INFO(
    get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg.info.width,
    msg.info.height,
    msg.info.resolution);
  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could"
      " cause issues with reading published topics",
      msg.header.frame_id.c_str(),
      global_frame_id_.c_str());
  }
  freeMapDependentMemory();
  global_acquisition_.reset();
  global_acquisition_pending_ = false;
  global_tracking_confirmation_pending_ = false;
  global_acquisition_status_ = "map_changed_requires_initialization";
  map_ = convertMap(msg);
  ++estimate_contract_.map_generation;
  estimate_contract_.resetObservation();
  fine_modes_valid_ = false;
  estimate_contract_.map_identity = occupancyIdentity(msg.header.frame_id,
    msg.info.width, msg.info.height, msg.info.resolution,
    {msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z,
      msg.info.origin.orientation.x, msg.info.origin.orientation.y,
      msg.info.origin.orientation.z, msg.info.origin.orientation.w}, msg.data);

#if NEW_UNIFORM_SAMPLING
  createFreeSpaceVector();
#endif
}

void
AmclNode::createFreeSpaceVector()
{
  // Index of free space
  free_space_indices.resize(0);
  for (int i = 0; i < map_->size_x; i++) {
    for (int j = 0; j < map_->size_y; j++) {
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
        free_space_indices.push_back(std::make_pair(i, j));
      }
    }
  }
}

void
AmclNode::freeMapDependentMemory()
{
  if (global_acquisition_) {
    latest_tf_valid_ = false;
    first_pose_sent_ = false;
    sent_first_transform_ = false;
    initial_pose_is_known_ = false;
  }
  global_acquisition_.reset();
  global_acquisition_pending_ = false;
  global_tracking_confirmation_pending_ = false;
  acquisition_consumed_scan_ = false;
  acquisition_consumed_stamp_ns_ = std::numeric_limits<int64_t>::min();
  global_acquisition_status_ = "map_unavailable";
  if (map_ != NULL) {
    map_free(map_);
    map_ = NULL;
  }

  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
}

// Convert an OccupancyGrid map message into the internal representation. This function
// allocates a map_t and returns it.
map_t *
AmclNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  map->cells =
    reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

  // Convert to player format
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if (map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }

  return map;
}

void
AmclNode::initTransforms()
{
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  sent_first_transform_ = false;
  latest_tf_valid_ = false;
  latest_tf_ = tf2::Transform::getIdentity();
}

void
AmclNode::initMessageFilters()
{
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
      rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_, rmw_qos_profile_sensor_data, sub_opt);

  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);


  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &AmclNode::laserReceived,
      this, std::placeholders::_1));
}

void
AmclNode::initPubSub()
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  estimate_contract_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "amcl_estimate_contract", rclcpp::QoS(10));

  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud",
    rclcpp::SensorDataQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
}

void
AmclNode::publishEstimateContract()
{
  if (!estimate_contract_pub_ || !estimate_contract_pub_->is_activated()) {return;}
  try {
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  message.header.frame_id = global_frame_id_;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "amcl/estimate_contract";
  status.hardware_id = "amcl_pf";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "conditional PF state; physical precision unvalidated";
  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue field;
      field.key = key; field.value = value; status.values.push_back(std::move(field));
    };
  const auto & c = estimate_contract_;
  add("owner", "amcl_pf");
  add("global_acquisition_pending", global_acquisition_pending_ || global_tracking_confirmation_pending_ ? "true" : "false");
  add("first_pf_confirmation_pending", global_tracking_confirmation_pending_ ? "true" : "false");
  add("global_acquisition_status", global_acquisition_status_);
  add("global_acquisition_semantics", "certified_finite_lattice_anchor_not_physical_confidence");
  if (global_acquisition_pending_ || global_tracking_confirmation_pending_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "global initialization pending; no valid map pose published";
  }
  add("map_generation", std::to_string(c.map_generation));
  add("map_identity", c.map_identity);
  add("source_id", c.source_id);
  add("source_identity_policy", "frame_and_raw_stamp_uniqueness_unvalidated");
  add("freshness_policy", "constant_memory_reference_time_high_watermark");
  add("source_deduplication", "not_enforced_compatibility_path");
  add("state_reference_stamp_ns", std::to_string(c.state_reference_stamp_ns));
  add("last_effective_observation_stamp_ns", std::to_string(c.last_effective_observation_stamp_ns));
  add("received_ros_stamp_ns", std::to_string(c.received_ros_stamp_ns));
  add("received_steady_ns", std::to_string(c.received_steady_ns));
  add("receipt_scope", "amcl_tf_filtered_callback");
  add("first_available_steady_ns", std::to_string(c.first_available_steady_ns));
  add("steady_clock_scope", "host_monotonic");
  add("tf_stamp_ns", std::to_string(c.tf_stamp_ns));
  add("valid_observation", c.valid_observation ? "true" : "false");
  add("duplicate_source_consumed", c.duplicate_source_consumed ? "true" : "false");
  add("nonmonotonic_source", c.nonmonotonic_source ? "true" : "false");
  add("physical_calibration", "unknown");
  add("shared_map_uncertainty", "unknown");
  add("covariance_semantics", "conditional_selected_cluster");
  add("mode_status", get_parameter("fine_mode_diagnostics").as_bool() ?
    (fine_modes_valid_ ? "fine_kde_diagnostic_unvalidated" : "fine_diagnostic_unavailable") :
    "coarse_kld_only");
  add("published_pose_mode_semantics", "coarse_kld_cluster_not_verified_unimodal");
  add("observability", "unknown_no_surface_information_model");
  add("support_boundary", "unknown_not_a_bounded_search_result");
  if (fine_modes_valid_) {
    const double mass = fine_modes_.modes[fine_modes_.selected].weight;
    add("fine_mode_count", std::to_string(fine_modes_.modes.size()));
    add("selected_mode_id", std::to_string(fine_modes_.selected));
    add("selected_mode_mass", diagnosticNumber(mass));
    add("other_mode_mass", diagnosticNumber(std::max(0.0, 1.0 - mass)));
    add("mode_separation_m", diagnosticNumber(fine_modes_.separation_m));
    add("mode_ambiguous", 1.0 - mass >= 0.1 ? "true" : "false");
    add("mode_ambiguity_rule", "diagnostic_other_mass_ge_0.1_unvalidated");
    add("mean_score", std::isfinite(fine_mean_score_) ? diagnosticNumber(fine_mean_score_) : "unknown");
    add("mode_score", std::isfinite(fine_mode_score_) ? diagnosticNumber(fine_mode_score_) : "unknown");
    add("score_semantics", "same_effective_beams_beta_log_likelihood_no_prior");
    add("score_pose_scope", "pre_resample_diagnostic_candidates_not_published_pose");
    const char * axes[] = {"x", "y", "yaw"};
    for (int i = 0; i < 3; ++i) {
      add(std::string("mean_pose_") + axes[i], std::isfinite(fine_scored_mean_pose_.v[i]) ?
        diagnosticNumber(fine_scored_mean_pose_.v[i]) : "unknown");
      add(std::string("mode_pose_") + axes[i], std::isfinite(fine_scored_mode_pose_.v[i]) ?
        diagnosticNumber(fine_scored_mode_pose_.v[i]) : "unknown");
    }
  }
  add("precision_ready", "false");
  add("output_statistics", get_parameter("pre_resample_pose_output").as_bool() ?
    "pre_resample" : "post_resample");
  message.status.push_back(std::move(status));
  estimate_contract_pub_->publish(std::move(message));
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "AMCL estimate contract unavailable: %s", error.what());
  }
}

void
AmclNode::initServices()
{
  global_loc_srv_ = create_service<std_srvs::srv::Empty>(
    "reinitialize_global_localization",
    std::bind(&AmclNode::globalLocalizationCallback, this, _1, _2, _3));

  initial_guess_srv_ = create_service<nav2_msgs::srv::SetInitialPose>(
    "set_initial_pose",
    std::bind(&AmclNode::initialPoseReceivedSrv, this, _1, _2, _3));

  nomotion_update_srv_ = create_service<std_srvs::srv::Empty>(
    "request_nomotion_update",
    std::bind(&AmclNode::nomotionUpdateCallback, this, _1, _2, _3));
}

void
AmclNode::initOdometry()
{
  // TODO(mjeronimo): We should handle persistance of the last known pose of the robot. We could
  // then read that pose here and initialize using that.

  // When pausing and resuming, remember the last robot pose so we don't start at 0:0 again
  init_pose_[0] = last_published_pose_.pose.pose.position.x;
  init_pose_[1] = last_published_pose_.pose.pose.position.y;
  init_pose_[2] = tf2::getYaw(last_published_pose_.pose.pose.orientation);

  if (!initial_pose_is_known_) {
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
  } else {
    init_cov_[0] = last_published_pose_.pose.covariance[0];
    init_cov_[1] = last_published_pose_.pose.covariance[7];
    init_cov_[2] = last_published_pose_.pose.covariance[35];
  }

  motion_model_ = plugin_loader_.createSharedInstance(robot_model_type_);
  motion_model_->initialize(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);

  latest_odom_pose_ = geometry_msgs::msg::PoseStamped();
}

void
AmclNode::initParticleFilter()
{
  global_acquisition_.reset();
  global_acquisition_pending_ = false;
  global_tracking_confirmation_pending_ = false;
  acquisition_consumed_scan_ = false;
  acquisition_consumed_stamp_ns_ = std::numeric_limits<int64_t>::min();
  global_acquisition_status_ = "filter_reset";
  estimate_contract_.resetObservation();
  fine_modes_valid_ = false;
  // Create the particle filter
  pf_ = pf_alloc(
    min_particles_, max_particles_, alpha_slow_, alpha_fast_,
    (pf_init_model_fn_t)AmclNode::uniformPoseGenerator);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];

  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];

  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);

  pf_init_ = false;
  resample_count_ = 0;
  memset(&pf_odom_pose_, 0, sizeof(pf_odom_pose_));
}

void
AmclNode::initLaserScan()
{
  scan_error_count_ = 0;
  last_laser_received_ts_ = rclcpp::Time(0);
}

}  // namespace nav2_amcl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_amcl::AmclNode)
