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

#include <rmf_robot_sim_common/BuildingClient.hpp>

namespace rmf_robot_sim_common {

//==============================================================================
BuildingClient::BuildingClient()
{
  // Do nothing
}

//==============================================================================
void BuildingClient::start(const rclcpp::Node::SharedPtr& node)
{
  _shared = std::make_shared<Shared>(node->get_logger());

  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local().reliable();
  _building_map_sub =
    node->create_subscription<BuildingMap>(
    "/map",
    qos_profile,
    [shared = _shared](const BuildingMap::SharedPtr msg)
    {
      shared->map_cb(msg);
    });
}

//==============================================================================
std::optional<std::string> BuildingClient::get_level_of(double elevation) const
{
  if (!_shared)
    return std::nullopt;

  if (!_shared->initialized_levels)
  {
    if (!_shared->warned_about_missing_map)
    {
      RCLCPP_WARN(
        _shared->logger,
        "[BuildingClient::get_level_of] Have not yet received building map");

      _shared->warned_about_missing_map = true;
    }

    return std::nullopt;
  }

  auto min_distance = std::numeric_limits<double>::max();
  const auto& level_map = _shared->level_to_elevation;
  std::optional<std::string> level_name;
  for (auto it = level_map.begin(); it != level_map.end(); ++it)
  {
    const double disp = std::abs(it->second - elevation);
    if (disp < min_distance)
    {
      min_distance = disp;
      level_name = it->first;
    }
  }

  return level_name;
}

//==============================================================================
BuildingClient::Shared::Shared(rclcpp::Logger logger_)
: logger(std::move(logger_))
{
  // Do nothing
}

//==============================================================================
void BuildingClient::Shared::map_cb(
  const rmf_building_map_msgs::msg::BuildingMap::SharedPtr msg)
{
  if (msg->levels.empty())
  {
    RCLCPP_ERROR(
      logger, "[BuildingClient::Shared::map_cb] Received empty building map");

    return;
  }

  RCLCPP_INFO(logger, "[BuildingClient] Received new map");

  for (const auto& level : msg->levels)
  {
    level_to_elevation.insert({level.name, level.elevation});
  }

  initialized_levels = true;
}

} // namespace rmf_robot_sim_common
