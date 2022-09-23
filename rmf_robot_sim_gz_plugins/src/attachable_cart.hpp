/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <rclcpp/rclcpp.hpp>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>

#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>


namespace attachable_cart {


class IGNITION_GAZEBO_VISIBLE AttachableCartPlugin
  : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;

  void Configure(const ignition::gazebo::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    ignition::gazebo::EntityComponentManager& ecm,
    ignition::gazebo::EventManager& event_mgr) override;

  // inherit from ISystemPreUpdate
  void PreUpdate(const ignition::gazebo::UpdateInfo& info,
    ignition::gazebo::EntityComponentManager& ecm) override;

private:

  static constexpr double STATE_PUBLISH_DT = 1.0;

  enum class AttachingRequest
  {
    NONE,
    ATTACH,
    DETACH
  };

  void AttachToRobot(ignition::gazebo::EntityComponentManager& ecm,
      const std::string& robot_name,
      bool attach,
      double cur_t);

  rclcpp::Node::SharedPtr _ros_node;

  std::string _name;
  ignition::gazebo::Entity _entity;

  rclcpp::Subscription<DispenserRequest>::SharedPtr _dispenser_request_sub;
  rclcpp::Subscription<IngestorRequest>::SharedPtr _ingestor_request_sub;

  rclcpp::Publisher<DispenserResult>::SharedPtr _dispenser_result_pub;
  rclcpp::Publisher<IngestorResult>::SharedPtr _ingestor_result_pub;
  rclcpp::Publisher<DispenserState>::SharedPtr _dispenser_state_pub;
  rclcpp::Publisher<IngestorState>::SharedPtr _ingestor_state_pub;

  AttachingRequest _pending_request = AttachingRequest::NONE;
  std::string _target_robot;
  std::string _request_guid;

  ignition::gazebo::Entity _joint_entity = ignition::gazebo::kNullEntity;

  DispenserState _dispenser_state;
  IngestorState _ingestor_state;

  double _last_state_pub = 0.0;
};

} //namespace attachable_cart
