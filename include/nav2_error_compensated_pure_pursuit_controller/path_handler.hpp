// Copyright (c) 2022 Samsung Research America
// Copyright (c) 2026 Fumiya Ohnishi
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
//
// Modified to retain the continuous projection segment during path transformation.

#ifndef NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_error_compensated_pure_pursuit_controller
{

/**
 * @brief Plugin-local global-plan transformer that retains the projection segment predecessor.
 *
 * Navigation2's RPP PathHandler begins its transformed path at the nearest sampled path vertex.
 * This implementation instead finds the nearest continuous non-zero segment within the progress
 * search bound and starts transformation/pruning at that segment's front vertex. Sparse and dense
 * sampling therefore make the same progress choice, with exact ties resolved toward earlier path
 * progress. The selected projection is returned in the robot frame for error, lookahead, and
 * cusp calculations. Navigation2 itself is not modified.
 */
class PathHandler
{
public:
  struct TransformedPlan
  {
    nav_msgs::msg::Path path;
    arc_length_lookahead::PathProjection projection;
  };

  PathHandler(
    tf2::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  TransformedPlan transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double max_robot_pose_search_dist,
    bool reject_unit_path = false);

  bool transformPose(
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  void setPlan(const nav_msgs::msg::Path & path) {global_plan_ = path;}

  nav_msgs::msg::Path getPlan() const {return global_plan_;}

private:
  double getCostmapMaxExtent() const;

  rclcpp::Logger logger_{rclcpp::get_logger("ECPPPathHandler")};
  tf2::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace nav2_error_compensated_pure_pursuit_controller

#endif  // NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__PATH_HANDLER_HPP_
