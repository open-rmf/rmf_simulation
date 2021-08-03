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

#ifndef RMF_ROBOT_SIM_COMMON__READONLY_BLOCKADE_COMMON_HPP
#define RMF_ROBOT_SIM_COMMON__READONLY_BLOCKADE_COMMON_HPP

#include <rclcpp/node.hpp>

#include <std_msgs/msg/string.hpp>

#include <rmf_fleet_msgs/msg/robot_state.hpp>

#include <Eigen/Geometry>

#include <rmf_robot_sim_common/BuildingClient.hpp>

namespace rmf_robot_sim_common {

//==============================================================================
class ReadOnlyBlockadeCommon
{
public:

  const std::string& get_name() const;

  template<typename SdfPtrT>
  void read_sdf(const SdfPtrT& sdf);

  void init(const std::string& name, const rclcpp::Node::SharedPtr& node);
  void on_update(const Eigen::Isometry3d& pose, double sim_time);

private:

  std::string get_level_name(double z) const;

  std::string _name;

  using RobotState = rmf_fleet_msgs::msg::RobotState;
  rclcpp::Publisher<RobotState>::SharedPtr _robot_state_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _destination_sub;

  std::shared_ptr<std::string> _destination;
  BuildingClient _building;

  std::optional<double> _last_update_time;
  double _update_threshold = 0.5; // update every 0.5s
  uint64_t _seq = 0;
};

} // namespace rmf_robot_sim_common

#endif // RMF_ROBOT_SIM_COMMON__READONLY_BLOCKADE_COMMON_HPP
