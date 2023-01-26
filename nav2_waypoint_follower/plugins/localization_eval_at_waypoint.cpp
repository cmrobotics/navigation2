// Copyright (c) 2020 Fetullah Atas
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <exception>

#include "nav2_util/node_utils.hpp"
#include "nav2_waypoint_follower/plugins/localization_eval_at_waypoint.hpp"

namespace nav2_waypoint_follower
{

LocalizationEvalAtWaypoint::LocalizationEvalAtWaypoint():
  is_enabled_(true)
{
}

LocalizationEvalAtWaypoint::~LocalizationEvalAtWaypoint()
{
}

void LocalizationEvalAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
  }
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  node->get_parameter(
    plugin_name + ".enabled",
    is_enabled_);

  localization_eval_trigger_client_ = node->create_client<std_srvs::srv::Empty>("trigger_localization_eval");
}



bool LocalizationEvalAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint",
    curr_waypoint_index);
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = localization_eval_trigger_client_->async_send_request(req);

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  return true;
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::LocalizationEvalAtWaypoint,
  nav2_core::WaypointTaskExecutor)
