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

#ifndef NAV2_AMCL__AMCL_NODE_HPP_
#define NAV2_AMCL__AMCL_NODE_HPP_

#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_amcl/estimate_contract.hpp"
#include "nav2_amcl/global_acquisition.hpp"
#include "nav2_amcl/acquisition_executor.hpp"
#include "nav2_amcl/pf/posterior_snapshot.hpp"
#include "nav2_amcl/pf/fine_modes.hpp"
#include "duojin02_scan_geometry/ros_scan_geometry.hpp"
#include "message_filters/subscriber.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "pluginlib/class_loader.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop

#define NEW_UNIFORM_SAMPLING 1

namespace nav2_amcl
{
/*
 * @class AmclNode
 * @brief ROS wrapper for AMCL
 */
class AmclNode : public nav2_util::LifecycleNode
{
public:
  explicit AmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AmclNode();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Since the sensor data from gazebo or the robot is not lifecycle enabled, we won't
  // respond until we're in the active state
  std::atomic<bool> active_{false};

  // Dedicated callback group and executor for services and subscriptions in AmclNode,
  // in order to isolate TF timer used in message filter.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;

  // Pose hypothesis
  typedef struct
  {
    double weight;             // Total weight (weights sum to 1)
    pf_vector_t pf_pose_mean;  // Mean of pose esimate
    pf_matrix_t pf_pose_cov;   // Covariance of pose estimate
  } amcl_hyp_t;

  // Map-related
  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
  void createFreeSpaceVector();
  void freeMapDependentMemory();
  map_t * map_{nullptr};
  /*
   * @brief Convert an occupancy grid map to an AMCL map
   * @param map_msg Map message
   * @return pointer to map for AMCL to use
   */
  map_t * convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);
  bool first_map_only_{true};
  std::atomic<bool> first_map_received_{false};
  amcl_hyp_t * initial_pose_hyp_;
  std::recursive_mutex mutex_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int>> free_space_indices;
#endif

  // Transforms
  void initTransforms();
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool sent_first_transform_{false};
  bool latest_tf_valid_{false};
  tf2::Transform latest_tf_;

  // Message filters
  void initMessageFilters();
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    rclcpp_lifecycle::LifecycleNode>> laser_scan_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  // Publishers and subscribers
  void initPubSub();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    particle_cloud_pub_;
  void initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);
  void processLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan,
    const duojin02_scan_geometry::TimedScan * geometry = nullptr,
    int64_t received_ros_ns = 0, int64_t received_steady_ns = 0);
  void drainPendingScans();
  void publishEstimateContract();
  EstimateContract estimate_contract_;
  std::unique_ptr<AcquisitionExecutor> global_acquisition_;
  bool global_acquisition_pending_{false};
  bool global_tracking_confirmation_pending_{false};
  bool acquisition_consumed_scan_{false};
  int64_t acquisition_consumed_stamp_ns_{std::numeric_limits<int64_t>::min()};
  std::string global_acquisition_status_{"not_requested"};
  FineModeSet fine_modes_;
  double fine_mean_score_{NAN}, fine_mode_score_{NAN};
  pf_vector_t fine_scored_mean_pose_{{NAN, NAN, NAN}};
  pf_vector_t fine_scored_mode_pose_{{NAN, NAN, NAN}};
  bool fine_modes_valid_{false};
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    estimate_contract_pub_;
  duojin02_scan_geometry::Config scan_geometry_config_;
  std::unique_ptr<duojin02_scan_geometry::RosScanGeometry> scan_geometry_;
  struct PendingScan {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    std::chrono::steady_clock::time_point received;
    int64_t received_ros_ns;
  };
  std::deque<PendingScan> pending_scans_;
  rclcpp::TimerBase::SharedPtr scan_retry_timer_;

  // Services and service callbacks
  void initServices();
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  void globalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr initial_guess_srv_;
  void initialPoseReceivedSrv(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> response);

  // Let amcl update samples without requiring motion
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
  /*
   * @brief Request an AMCL update even though the robot hasn't moved
   */
  void nomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Nomotion update control. Used to temporarily let amcl update samples even when no motion occurs
  std::atomic<bool> force_update_{false};

  // Odometry
  void initOdometry();
  std::shared_ptr<nav2_amcl::MotionModel> motion_model_;
  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose_;
  double init_pose_[3];  // Initial robot pose
  double init_cov_[3];
  pluginlib::ClassLoader<nav2_amcl::MotionModel> plugin_loader_{"duojin02_amcl_cuda",
    "nav2_amcl::MotionModel"};
  /*
   * @brief Get robot pose in odom frame using TF
   */
  bool getOdomPose(
    geometry_msgs::msg::PoseStamped & pose,
    double & x, double & y, double & yaw,
    const rclcpp::Time & sensor_timestamp, const std::string & frame_id);
  std::atomic<bool> first_pose_sent_;

  // Particle filter
  void initParticleFilter();
  /*
   * @brief Pose-generating function used to uniformly distribute particles over the map
   */
  static pf_vector_t uniformPoseGenerator(void * arg);
  pf_t * pf_{nullptr};
  bool pf_init_;
  pf_vector_t pf_odom_pose_;
  int resample_count_{0};

  // Laser scan related
  void initLaserScan();
  nav2_amcl::Laser * createLaserObject();
  int scan_error_count_{0};
  std::vector<std::unique_ptr<nav2_amcl::Laser>> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;
  rclcpp::Time last_laser_received_ts_;

  bool checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time);
  rclcpp::Time last_time_printed_msg_;
  bool addNewScanner(
    int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::string & laser_scan_frame_id,
    geometry_msgs::msg::PoseStamped & laser_pose);
  bool shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta);

  // Replay the original scan-gated prediction sequence from a delayed anchor.
  // The final nonzero tail is consumed so subsequent PF motion starts at now.
  void applyAcquisitionMotion(const AcquisitionDispatch & dispatch);
  bool updateFilter(
    const int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const pf_vector_t & pose,
    const duojin02_scan_geometry::TimedScan * geometry = nullptr,
    int64_t raw_stamp_ns = 0);
  void publishParticleCloud(const pf_sample_set_t * set);
  bool getMaxWeightHyp(
    std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
    int & max_weight_hyp, const PosteriorSnapshot * snapshot = nullptr);
  /*
   * @brief Publish robot pose in map frame from AMCL
   */
  void publishAmclPose(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp);
  /*
   * @brief Determine TF transformation from map to odom
   */
  void calculateMaptoOdomTransform(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps,
    const int & max_weight_hyp,
    const duojin02_scan_geometry::TimedScan * geometry = nullptr);
  void sendMapToOdomTransform(const tf2::TimePoint & transform_expiration);
  void handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  bool init_pose_received_on_inactive{false};
  bool initial_pose_is_known_{false};
  bool set_initial_pose_{false};
  bool always_reset_initial_pose_;
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_yaw_;

  void initParameters();
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double alpha5_;
  std::string base_frame_id_;
  double beam_skip_distance_;
  double beam_skip_error_threshold_;
  double beam_skip_threshold_;
  bool do_beamskip_;
  std::string global_frame_id_;
  double lambda_short_;
  double laser_likelihood_max_dist_;
  double laser_max_range_;
  double laser_min_range_;
  std::string sensor_model_type_;
  int max_beams_;
  int max_particles_;
  int min_particles_;
  std::string odom_frame_id_;
  double pf_err_;
  double pf_z_;
  double alpha_fast_;
  double alpha_slow_;
  int resample_interval_;
  std::string robot_model_type_;
  tf2::Duration save_pose_period_;
  double sigma_hit_;
  bool tf_broadcast_;
  tf2::Duration transform_tolerance_;
  double a_thresh_;
  double d_thresh_;
  double z_hit_;
  double z_max_;
  double z_short_;
  double z_rand_;
  std::string scan_topic_{"scan"};
  std::string map_topic_{"map"};
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__AMCL_NODE_HPP_
