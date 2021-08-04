/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <rmf_robot_sim_common/readonly_blockade_common.hpp>
#include <rmf_robot_sim_common/utils.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/location.hpp>

namespace rmf_robot_sim_common {

//==============================================================================
ReadOnlyBlockadeCommon::ReadOnlyBlockadeCommon()
: _destination(std::make_shared<std::string>())
{
  // Do nothing
}

//==============================================================================
const std::string& ReadOnlyBlockadeCommon::get_name() const
{
  return _name;
}

//==============================================================================
void ReadOnlyBlockadeCommon::_init(
  const std::string& name,
  const rclcpp::Node::SharedPtr& node)
{
  _logger = std::make_shared<rclcpp::Logger>(node->get_logger());

  _building.start(node);
  _robot_state_pub =
    node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
      "/robot_state", 10);

  _destination_sub =
    node->create_subscription<std_msgs::msg::String>(
    "/" + name + "/destination", rclcpp::QoS(10),
    [dest = _destination](const std_msgs::msg::String::SharedPtr msg)
      {
        *dest = msg->data;
      });
}

//==============================================================================
std::string ReadOnlyBlockadeCommon::_get_level_name(double z) const
{
  return _building.get_level_of(z).value_or("");
}

//==============================================================================
void ReadOnlyBlockadeCommon::on_update(
  const Eigen::Isometry3d& pose,
  double sim_time)
{
  if (_last_update_time.has_value())
  {
    if (sim_time - *_last_update_time < _update_threshold)
      return;
  }

  _last_update_time = sim_time;
  const auto now = rclcpp::Time(rmf_plugins_utils::simulation_now(sim_time));

  auto mode = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotMode>()
      .mode(rmf_fleet_msgs::msg::RobotMode::MODE_MOVING)
      .mode_request_id(0);

  auto location = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::Location>()
      .t(now)
      .x(pose.translation().x())
      .y(pose.translation().y())
      .yaw(rmf_plugins_utils::compute_yaw(pose))
      .level_name(_get_level_name(pose.translation().z()))
      .index(0);

  auto msg =
    rmf_fleet_msgs::build<rmf_fleet_msgs::msg::RobotState>()
      .name(_name)
      .model("")
      .task_id(*_destination)
      .seq(_seq++)
      .mode(mode)
      .battery_percent(100.0)
      .location(location)
      .path({});

  _robot_state_pub->publish(msg);
}

} // namespace rmf_robot_sim_common
