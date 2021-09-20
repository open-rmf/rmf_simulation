/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_BUILDING_SIM_COMMON__CHARGER_COMMON_HPP
#define RMF_BUILDING_SIM_COMMON__CHARGER_COMMON_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmf_charger_msgs/msg/charger_state.hpp>
#include <rmf_charger_msgs/msg/charger_request.hpp>
#include <rmf_charger_msgs/msg/charger_cancel.hpp>

namespace rmf_building_sim_common {

class ChargerCommon
{
public:
  ChargerCommon();

  void init_ros_node(std::shared_ptr<rclcpp::Node> node);

  void add_charger(std::string name, double x, double y, std::string level);

  void idle(std::string name);

  void update_charger_charge(std::string name, rclcpp::Duration charge);

  void complete_charger_charge(std::string name);

  void start_charging(std::string name);

private:
  rclcpp::Logger logger() const
  {
    return rclcpp::get_logger("charger_stub");
  }
  void on_charger_request_recieved(
    const rmf_charger_msgs::msg::ChargerRequest::SharedPtr request);

  void on_charger_request_cancelled(
    const rmf_charger_msgs::msg::ChargerCancel::SharedPtr cancel);

  void status_publisher_timer_callback();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rmf_charger_msgs::msg::ChargerState>::SharedPtr
    state_publisher_;

  rclcpp::Subscription<rmf_charger_msgs::msg::ChargerRequest>::SharedPtr
    request_subscription_;
  rclcpp::Subscription<rmf_charger_msgs::msg::ChargerCancel>::SharedPtr
    cancel_subscription_;

  rclcpp::TimerBase::SharedPtr status_publisher_timer_;

  enum State
  {
    IDLE = rmf_charger_msgs::msg::ChargerState::CHARGER_IDLE,
    ASSIGNED = rmf_charger_msgs::msg::ChargerState::CHARGER_ASSIGNED, 
    CHARGING = rmf_charger_msgs::msg::ChargerState::CHARGER_CHARGING, 
  };

  struct Charger
  {
    std::string name_;
    double x_;
    double y_;
    std::string level_;
    State state_;
    std::string assigned_robot_name_;
    std::string assigned_robot_fleet_;
    rclcpp::Duration charge_time_;
    int curr_req_ = 0;
  };
  std::unordered_map<std::string, Charger> chargers_;
  int req_id_ = 0;
  std::mutex mtx_;

};

}  // namespace rmf_robot_sim_common

#endif /* RMF_BUILDING_SIM_COMMON__CHARGER_COMMON_HPP */
