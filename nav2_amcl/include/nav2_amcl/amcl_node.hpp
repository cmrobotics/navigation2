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
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "nav2_amcl/sensors/external_pose/external_pose.hpp"
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "pluginlib/class_loader.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include "cmr_msgs/srv/get_status.hpp"

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
  /*
   * @brief AMCL constructor
   * @param options Additional options to control creation of the node.
   */
  explicit AmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /*
   * @brief AMCL destructor
   */
  ~AmclNode();

protected:
  /*
   * @brief Lifecycle configure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle activate
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle deactivate
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle cleanup
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /*
   * @brief Lifecycle shutdown
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
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
  /*
   * @brief Get new map from ROS topic to localize in
   * @param msg Map message
   */
  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  /*
   * @brief Handle a new map message
   * @param msg Map message
   */
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
  /*
   * @brief Creates lookup table of free cells in map
   */
  void createFreeSpaceVector();
  /*
   * @brief Handle a new external pose message
   * @param msg External pose message
   */
  void externalPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  /*
   * @brief Frees allocated map related memory
   */
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
  /*
   * @brief Initialize required ROS transformations
   */
  void initTransforms();
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool sent_first_transform_{false};
  bool latest_tf_valid_{false};
  tf2::Transform latest_tf_;

  // Message filters
  /*
   * @brief Initialize incoming data message subscribers and filters
   */
  void initMessageFilters();
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
    rclcpp_lifecycle::LifecycleNode>> laser_scan_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  // Publishers and subscribers
  /*
   * @brief Initialize pub subs of AMCL
   */
  void initPubSub();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    external_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
    particle_cloud_pub_;
  /*
   * @brief Handle with an initial pose estimate is received
   */
  void initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  /*
   * @brief Handle when a laser scan is received
   */
  void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);

  // Services and service callbacks
  /*
   * @brief Initialize state services
   */
  void initServices();
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  /*
   * @brief Service callback for a global relocalization request
   */
  void globalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Let amcl update samples without requiring motion
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
  /*
   * @brief Request an AMCL update even though the robot hasn't moved
   */
  void nomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Report whether initial pose set or not
  rclcpp::Service<cmr_msgs::srv::GetStatus>::SharedPtr get_initial_pose_status_srv_;
  /*
   * @brief Service callback for sending intial pose status
   */
  void getInitialPoseStatusCallback(
    const std::shared_ptr<cmr_msgs::srv::GetStatus::Request>,
    std::shared_ptr<cmr_msgs::srv::GetStatus::Response> response);

  // Nomotion update control. Used to temporarily let amcl update samples even when no motion occurs
  std::atomic<bool> force_update_{false};

  // Odometry
  /*
   * @brief Initialize odometry
   */
  void initOdometry();
  std::shared_ptr<nav2_amcl::MotionModel> motion_model_;
  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose_;
  double init_pose_[3];  // Initial robot pose
  double init_cov_[3];
  pluginlib::ClassLoader<nav2_amcl::MotionModel> plugin_loader_{"nav2_amcl",
    "nav2_amcl::MotionModel"};
  
  /*
   * @brief Initialize Diagnostic
   */
  void initDiagnostic();
  
  /*
   * @brief Get robot pose in odom frame using TF
   */
  bool getOdomPose(
    // Helper to get odometric pose from transform system
    geometry_msgs::msg::PoseStamped & pose,
    double & x, double & y, double & yaw,
    const rclcpp::Time & sensor_timestamp, const std::string & frame_id);
  std::atomic<bool> first_pose_sent_;

  // Particle filter
  /*
   * @brief Initialize particle filter
   */
  void initParticleFilter();
  /*
   * @brief Pose-generating function used to uniformly distribute particles over the map
   */
  static pf_vector_t uniformPoseGenerator(void * arg);
  pf_t * pf_{nullptr};
  bool pf_init_;
  pf_vector_t pf_odom_pose_;
  int resample_count_{0};

  // Random seeds
  long amcl_seed_;
  long gaussian_pdf_seed_;
  long pf_seed_;
  
  // External pose fusion related
  /*
   * @brief Check if external pose has been received
   */
  bool isExtPoseActive();
  /*
   * @brief Initialize external pose data source
   */
  void initExternalPose();
  /*
   * @brief Diagnostic checking robot's covariance
   * @ true if deviation gets large
   *
   */
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  void amclDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  double std_warn_level_x_;
  double std_warn_level_y_;
  double std_warn_level_yaw_;
  double aug_mcl_score_threshold_;

  /*
    * @brief Check localization health
  */
  void healthCheck();

  void updateMetrics();
  void updateLaserImportance(double correction_factor);

  std::unordered_map<std::string, double> metrics_;

  std::unique_ptr<ExternalPoseBuffer> ext_pose_buffer_;

  rclcpp::Time last_ext_pose_received_ts_;
  std::chrono::seconds ext_pose_check_interval_;
  bool ext_pose_active_;

  // Laser scan related
  /*
   * @brief Initialize laser scan
   */
  void initLaserScan();
  /*
   * @brief Create a laser object
   */
  nav2_amcl::Laser * createLaserObject();
  int scan_error_count_{0};
  std::vector<nav2_amcl::Laser *> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;
  rclcpp::Time last_laser_received_ts_;

  /*
   * @brief Check if sufficient time has elapsed to get an update
   */
  bool checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time);
  rclcpp::Time last_time_printed_msg_;
  /*
   * @brief Add a new laser scanner if a new one is received in the laser scallbacks
   */
  bool addNewScanner(
    int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::string & laser_scan_frame_id,
    geometry_msgs::msg::PoseStamped & laser_pose);
  /*
   * @brief Whether the pf needs to be updated
   */
  bool shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta);
  /*
   * @brief Update the PF
   */
  bool updateFilter(
    const int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const pf_vector_t & pose);
  /*
   * @brief Publish particle cloud
   */
  void publishParticleCloud(const pf_sample_set_t * set);
  /*
  * @brief Get the current state estimate hypothesis from the particle cloud
  * by calculating mean weighted centroid among cluster centroids
  */
  bool getMeanWeightedClustersCentroid(amcl_hyp_t & mean_centroid_hyp);
  /*
   * @brief Get the current state estimat hypothesis from the particle cloud
   */
  bool getMaxWeightHyp(amcl_hyp_t & max_weight_hyp);
  /*
   * @brief Publish robot pose in map frame from AMCL
   */
  void publishAmclPose(const amcl_hyp_t & best_hyp,
    const builtin_interfaces::msg::Time & timestamp_msg);
  /*
   * @brief Determine TF transformation from map to odom
   */
  void calculateMaptoOdomTransform(const amcl_hyp_t & best_hyp,
    const builtin_interfaces::msg::Time & timestamp_msg);
  /*
   * @brief Publish TF transformation from map to odom
   */
  void sendMapToOdomTransform(const tf2::TimePoint & transform_expiration);
  /*
   * @brief Update input pose with odometric change happened since the timestamp of that pose
  */
  bool integrateOdometricChange(
    const geometry_msgs::msg::Pose & input_pose,
    const rclcpp::Time & input_pose_time,
    geometry_msgs::msg::Pose & transformed_pose_msg
  );
  /*
   * @brief Handle a new pose estimate callback
   */
  void handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  bool init_pose_received_on_inactive{false};
  
  // Flag signaling that the initial pose has been set
  bool initial_pose_is_known_{false};

  // Flag signaling that the initial pose has been received
  bool initial_pose_is_ready_{false};
  bool set_initial_pose_{false};
  bool always_reset_initial_pose_;
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_yaw_;

  /*
   * @brief Get ROS parameters for node
   */
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
  double min_laser_hit_sample_dist_;
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
  tf2::Duration transform_tol_scan_odom_sec_;
  tf2::Duration transform_tol_scan_robot_sec_;
  tf2::Duration pose_valid_in_future_sec_;
  tf2::Duration transform_lookup_timeout_;
  double a_thresh_;
  double d_thresh_;
  double z_hit_;
  double z_max_;
  double z_short_;
  double z_rand_;
  double laser_importance_factor_;
  double max_particle_gen_prob_ext_pose_;
  double ext_pose_search_tolerance_sec_;
  std::string scan_topic_{"scan"};
  std::string map_topic_{"map"};
  bool use_cluster_averaging_;
  bool use_augmented_mcl_;
  bool dynamic_laser_importance_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__AMCL_NODE_HPP_
