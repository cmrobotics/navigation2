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
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_
#define NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace nav2_costmap_2d
{

/**
 * @class RobotObstacleLayer
 * @brief Takes in laser and pointcloud data to populate into 2D costmap
 */
class RobotObstacleLayer : public CostmapLayer
{
public:
  /**
   * @brief A constructor
   */
  RobotObstacleLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  /**
   * @brief A destructor
   */
  virtual ~RobotObstacleLayer();
  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();
  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();

  /**
   * @brief Activate the layer
   */
  virtual void activate();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return true;}

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void robots_poses_callback_(const nav_msgs::msg::Path::SharedPtr robot_poses);
protected:

  unsigned char other_robots_footprint_marking_value_, other_robots_path_marking_value_;
  unsigned char cost_decrement_step_per_waypoint_;
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  std::vector<geometry_msgs::msg::PoseStamped> all_robot_poses_;
  std::vector<nav_msgs::msg::Path> all_robot_paths_;
  std::mutex all_robot_poses_lock_, all_robot_paths_lock_;
  std::string other_robots_footprint_string_;

  bool footprint_clearing_enabled_;
  /**
   * @brief Clear costmap layer info below the robot's footprint
   */
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  double marking_width_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr robot_location_subscription_;

  /// @brief Used to project laser scans into point clouds
  laser_geometry::LaserProjection projector_;
  /// @brief Used for the observation message filters
  std::vector<std::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
  /// @brief Used to make sure that transforms are available for each sensor
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
  /// @brief Used to store observations from various sensors
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> observation_buffers_;
  /// @brief Used to store observation buffers used for marking obstacles
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> marking_buffers_;
  /// @brief Used to store observation buffers used for clearing obstacles
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> clearing_buffers_;

  // Used only for testing purposes
  
  bool rolling_window_;
  bool was_reset_;
  int combination_method_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__OBSTACLE_LAYER_HPP_
