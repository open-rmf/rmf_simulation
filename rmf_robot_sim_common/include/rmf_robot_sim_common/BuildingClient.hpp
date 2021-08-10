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

#ifndef RMF_ROBOT_SIM_COMMON__BUILDINGCLIENT_HPP
#define RMF_ROBOT_SIM_COMMON__BUILDINGCLIENT_HPP

#include <rclcpp/node.hpp>

#include <rmf_building_map_msgs/msg/building_map.hpp>

namespace rmf_robot_sim_common {

//==============================================================================
class BuildingClient
{
public:

  BuildingClient();

  void start(const rclcpp::Node::SharedPtr& node);

  std::optional<std::string> get_level_of(double elevation) const;

private:

  struct Shared
  {
    Shared(rclcpp::Logger logger_);

    void map_cb(rmf_building_map_msgs::msg::BuildingMap::SharedPtr msg);

    rclcpp::Logger logger;

    std::unordered_map<std::string, double> level_to_elevation;
    bool initialized_levels = false;
    bool warned_about_missing_map = false;
  };

  std::shared_ptr<Shared> _shared;

  using BuildingMap = rmf_building_map_msgs::msg::BuildingMap;
  rclcpp::Subscription<BuildingMap>::SharedPtr _building_map_sub;
};

} // namespace rmf_robot_sim_common

#endif // RMF_ROBOT_SIM_COMMON__BUILDINGCLIENT_HPP
