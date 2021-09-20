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
#include <rmf_building_sim_common/charger_common.hpp>
#include <functional>

namespace rmf_building_sim_common {

ChargerCommon::ChargerCommon()
{
  // Do nothing
}

void ChargerCommon::init_ros_node(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
  rclcpp::SystemDefaultsQoS qos;
  state_publisher_ =
    node_->create_publisher<rmf_charger_msgs::msg::ChargerState>(
      "rmf_charger/state", 1);
  request_subscription_ =
    node_->create_subscription<rmf_charger_msgs::msg::ChargerRequest>(
      "rmf_charger/requests",
      qos,
      std::bind(
        &ChargerCommon::on_charger_request_recieved,
        this,
        std::placeholders::_1));
  cancel_subscription_ =
    node_->create_subscription<rmf_charger_msgs::msg::ChargerCancel>(
      "rmf_charger/cancel",
      qos,
      std::bind(
        &ChargerCommon::on_charger_request_cancelled,
        this,
        std::placeholders::_1));

  status_publisher_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ChargerCommon::status_publisher_timer_callback, this));
}

void ChargerCommon::add_charger(
  std::string name,
  double x,
  double y,
  std::string level)
{
  Charger charger {name,x,y,level,State::IDLE,"", "", rclcpp::Duration(0)};
  chargers_.insert({name, charger});
}

void ChargerCommon::update_charger_charge(
  std::string name, rclcpp::Duration charge)
{
  auto it = chargers_.find(name);
  if(it == chargers_.end()) return;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    it->second.state_ = State::CHARGING;
    it->second.charge_time_ = charge;
  }
}

void ChargerCommon::on_charger_request_recieved(
  const rmf_charger_msgs::msg::ChargerRequest::SharedPtr request)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = chargers_.find(request->charger_name);
  if(it == chargers_.end())
  {
    RCLCPP_ERROR(logger(),
      "Charger not found: Recieved request with invalid charger name");
    return;
  }

  if(it->second.state_ != State::IDLE)
  {
    RCLCPP_ERROR(logger(),
      "Charger %s is busy", request->charger_name.c_str());
  }
  it->second.state_ = State::ASSIGNED;
  it->second.assigned_robot_fleet_ = request->fleet_name;
  it->second.assigned_robot_name_ = request->robot_name;
  it->second.curr_req_ = req_id_++;
}

void ChargerCommon::complete_charger_charge(std::string name)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = chargers_.find(name);
  if(it == chargers_.end()) return;
  {
    //it->second.state_ = State::COMPLETED;
  }
}

void ChargerCommon::status_publisher_timer_callback()
{
  std::lock_guard<std::mutex> lock(mtx_);
  for(auto& [name, charger]: chargers_)
  {
    rmf_charger_msgs::msg::ChargerState msg;
    {
      msg.charger_name = name;
      msg.state = charger.state_;
      msg.request_id = charger.curr_req_;
      msg.robot_fleet = charger.assigned_robot_fleet_;
      msg.robot_name = charger.assigned_robot_name_;
    }
    state_publisher_->publish(msg);
  }
}

}