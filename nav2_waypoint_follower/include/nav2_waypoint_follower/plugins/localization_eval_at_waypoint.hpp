
#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__LOCALIZATION_EVAL_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__LOCALIZATION_EVAL_AT_WAYPOINT_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief After arriving at the waypoint
 */
class FiducialsPoseAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
/**
 * @brief Construct a new Localization eval at waypoint object
 *
 */
  LocalizationEvalAtWaypoint();

  /**
   * @brief Destroy the Localization eval at waypoint object
   *
   */
  ~LocalizationEvalAtWaypoint();

  /**
   * @brief declares and loads parameters used (??)
   *
   * @param parent parent node that plugin will be created withing(waypoint_follower in this case)
   * @param plugin_name
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  // the robot will sleep waypoint_pause_duration_ milliseconds
  int waypoint_pause_duration_;
  bool is_enabled_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
};

}  // namespace nav2_waypoint_follower
#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__LOCALIZATION_EVAL_AT_WAYPOINT_HPP_
