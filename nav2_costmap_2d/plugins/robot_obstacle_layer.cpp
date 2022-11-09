/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Steve Macenski
 *********************************************************************/
#include "nav2_costmap_2d/robot_obstacle_layer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::RobotObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;

namespace nav2_costmap_2d
{

RobotObstacleLayer::~RobotObstacleLayer()
{
  for (auto & notifier : observation_notifiers_) {
    notifier.reset();
  }
}

void RobotObstacleLayer::onInitialize()
{
  bool track_unknown_space;
  double transform_tolerance;
  std::string robot_poses_topic, robot_paths_topic;
  // TODO(mjeronimo): these four are candidates for dynamic update
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("marking_width", rclcpp::ParameterValue(1.0));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("robot_poses_topic", rclcpp::ParameterValue(std::string("other_robots_poses")));
  declareParameter("robot_paths_topic", rclcpp::ParameterValue(std::string("other_robots_paths")));
  declareParameter("other_robots_footprint", rclcpp::ParameterValue(std::string("[ [0.48, 0.36], [-0.48, 0.36], [-0.48, -0.36], [0.48, -0.36] ]")));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "marking_width", marking_width_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter(name_ + "." + "robot_poses_topic", robot_poses_topic);
  node->get_parameter(name_ + "." + "robot_paths_topic", robot_paths_topic);
  node->get_parameter(name_ + "." + "other_robots_footprint", other_robots_footprint_string_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);

  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }

  RobotObstacleLayer::matchSize();
  current_ = true;
  was_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  robot_location_subscription_ = node->create_subscription<nav_msgs::msg::Path>(
                 robot_poses_topic, rclcpp::SystemDefaultsQoS(), std::bind(&RobotObstacleLayer::robots_poses_callback_, this, std::placeholders::_1));
}

void RobotObstacleLayer::robots_poses_callback_(const nav_msgs::msg::Path::SharedPtr robot_poses)
{
  all_robot_poses_ = robot_poses->poses;
}

void
RobotObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);
  
  // clear old robot locations
  resetMaps();

  // Update entire map
  *min_x = std::numeric_limits<double>::lowest();
  *min_y = std::numeric_limits<double>::lowest();
  *max_x = std::numeric_limits<double>::max();
  *max_y = std::numeric_limits<double>::max();

  RCLCPP_WARN(
      logger_,
      "Size (%.2f, %.2f) . ",getSizeInMetersX(), getSizeInMetersY());

  // Add robots 
  for(geometry_msgs::msg::PoseStamped & robot_pose: all_robot_poses_)
  {
    std::vector<geometry_msgs::msg::Point> robot_footprint, transformed_footprint;


    // All this just to get yaw
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(robot_pose.pose.orientation, tf2_quat);
    double r, p, yaw;
    tf2::Matrix3x3 m(tf2_quat);
    m.getRPY(r, p, yaw);

    if(makeFootprintFromString( other_robots_footprint_string_, robot_footprint))
    {
      transformFootprint(robot_pose.pose.position.x, robot_pose.pose.position.y, yaw, 
                          robot_footprint, transformed_footprint);
      setConvexPolygonCost(transformed_footprint, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
      
  }

  for(nav_msgs::msg::Path & robot_path: all_robot_paths_)
  {
    if(robot_path.poses.size() < 2)
      continue;
    
    for(unsigned int pose_index = 1; pose_index < robot_path.poses.size(); pose_index++)
    {
      geometry_msgs::msg::PoseStamped waypoint_1 = robot_path.poses.at(pose_index-1);
      geometry_msgs::msg::PoseStamped waypoint_2 = robot_path.poses.at(pose_index);

      std::vector<geometry_msgs::msg::Point> rectangle_along_path;

      geometry_msgs::msg::Point vertex;
      
      geometry_msgs::msg::Point straight_line_vector;

      //tf2::Vector3 straight_line_vector;

      straight_line_vector.x = waypoint_2.pose.position.x - waypoint_1.pose.position.x;
      straight_line_vector.y = waypoint_2.pose.position.y - waypoint_1.pose.position.y;
      straight_line_vector.z = 0;
      double norm = hypot(straight_line_vector.x, straight_line_vector.y);
      straight_line_vector.x /= norm;
      straight_line_vector.y /= norm;

      geometry_msgs::msg::Point unit_perp_vector; // rotate vector by 90deg
      unit_perp_vector.x = straight_line_vector.y;
      unit_perp_vector.y = -straight_line_vector.x;

      // Get 4 vertices to get a rectangle
      // Translate straight line path along _|_ normal in two directions

      vertex.x = waypoint_1.pose.position.x + unit_perp_vector.x * marking_width_/2.0;
      vertex.y = waypoint_1.pose.position.y + unit_perp_vector.y * marking_width_/2.0;

      rectangle_along_path.push_back(vertex);

      vertex.x = waypoint_2.pose.position.x + unit_perp_vector.x * marking_width_/2.0;
      vertex.y = waypoint_2.pose.position.y + unit_perp_vector.y * marking_width_/2.0;

      rectangle_along_path.push_back(vertex);

      vertex.x = waypoint_2.pose.position.x - unit_perp_vector.x * marking_width_/2.0;
      vertex.y = waypoint_2.pose.position.y - unit_perp_vector.y * marking_width_/2.0;

      rectangle_along_path.push_back(vertex);

      vertex.x = waypoint_1.pose.position.x - unit_perp_vector.x * marking_width_/2.0;
      vertex.y = waypoint_1.pose.position.y - unit_perp_vector.y * marking_width_/2.0;

      rectangle_along_path.push_back(vertex);

      setConvexPolygonCost(rectangle_along_path, nav2_costmap_2d::LETHAL_OBSTACLE);

    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
RobotObstacleLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
RobotObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // if not current due to reset, set current now after clearing
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void
RobotObstacleLayer::activate()
{
  for (auto & notifier : observation_notifiers_) {
    notifier->clear();
  }

  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->subscribe();
    }
  }
}

void
RobotObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != NULL) {
      observation_subscribers_[i]->unsubscribe();
    }
  }
}

void
RobotObstacleLayer::reset()
{
  resetMaps();
  current_ = false;
  was_reset_ = true;
}


}  // namespace nav2_costmap_2d
