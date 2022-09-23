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

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>

#include "attachable_cart.hpp"

using namespace ignition::gazebo;

namespace attachable_cart {

//=================================================
void AttachableCartPlugin::Configure(
  const Entity& entity,
  const std::shared_ptr<const sdf::Element>& /*sdf*/,
  EntityComponentManager& ecm,
  EventManager& /*event_mgr*/)
{
  auto model = Model(entity);
  char const** argv = NULL;
  _name = model.Name(ecm);
  // Entity has to be to the model's link
  auto entities = ecm.ChildrenByComponents(entity, components::Link());
  if (entities.size() != 1)
  {
    ignwarn << "Cart should only have one link, using first" << std::endl;
  }
  _entity = entities[0];
  if (!rclcpp::ok())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + _name);
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

  _dispenser_result_pub =
    _ros_node->create_publisher<DispenserResult>(
    "dispenser_results", 10);

  _ingestor_result_pub =
    _ros_node->create_publisher<IngestorResult>(
    "ingestor_results", 10);

  _dispenser_state_pub =
    _ros_node->create_publisher<DispenserState>(
    "dispenser_states", 10);

  _ingestor_state_pub =
    _ros_node->create_publisher<IngestorState>(
    "ingestor_states", 10);

  _dispenser_state.guid = _name;
  _dispenser_state.mode = _dispenser_state.IDLE;
  _ingestor_state.guid = _name;
  _ingestor_state.mode = _ingestor_state.IDLE;

  _dispenser_request_sub = _ros_node->create_subscription<DispenserRequest>(
    "dispenser_requests", rclcpp::SystemDefaultsQoS(),
    [&](DispenserRequest::UniquePtr msg)
    {
      if (msg->target_guid == _name)
      {
        ignmsg << "Received dispensing request for " << _name << std::endl;
        if (msg->items.size() != 1)
        {
          ignwarn << "Unexpected items size " << msg->items.size() << " aborting..." << std::endl;
          return;
        }
        else if (_joint_entity != kNullEntity)
        {
          ignmsg << "Joint already added, skipping" << std::endl;
          return;
        }
        _pending_request = AttachingRequest::ATTACH;
        _request_guid = msg->request_guid;
        _target_robot = msg->items[0].type_guid;
      }
    });

  _ingestor_request_sub = _ros_node->create_subscription<IngestorRequest>(
    "ingestor_requests", rclcpp::SystemDefaultsQoS(),
    [&](IngestorRequest::UniquePtr msg)
    {
      if (msg->target_guid == _name)
      {
        ignmsg << "Received ingesting request for " << _name << std::endl;
        if (msg->items.size() != 1)
        {
          ignwarn << "Unexpected items size " << msg->items.size() << " aborting..." << std::endl;
          return;
        }
        if (msg->items[0].type_guid != _target_robot)
        {
          ignwarn << "Requested to detach from " << msg->items[0].type_guid <<
            " but item is attached to " << _target_robot << " ignoring..." << std::endl;
          return;
        }
        else if (_joint_entity == kNullEntity)
        {
          ignmsg << "Joint already removed, skipping" << std::endl;
          return;
        }
        _pending_request = AttachingRequest::DETACH;
        _request_guid = msg->request_guid;
        _target_robot = "";
      }
    });
}

//=================================================
void AttachableCartPlugin::AttachToRobot(EntityComponentManager& ecm,
    const std::string& robot_name,
    bool attach,
    double cur_t)
{
  if (attach)
  {
    DispenserResult res;
    res.time.sec = int(cur_t);
    res.time.nanosec = (cur_t - int(cur_t)) *1e9;
    res.request_guid = _request_guid;
    res.source_guid = _name;
    res.status = res.FAILED;
    // ATTACH
    _joint_entity = ecm.CreateEntity();
    // Get the robot by name, then its parent
    auto robot_entity = ecm.EntityByComponents(components::Name(robot_name),
        components::Model());

    if (robot_entity == kNullEntity)
    {
      ignerr << "Robot " << robot_name << " not found, not attaching" << std::endl;
      _dispenser_result_pub->publish(res);
      return;
    }

    auto robot_link_entities = ecm.ChildrenByComponents(robot_entity, components::Link());
    if (robot_link_entities.size() != 1)
    {
      ignwarn << "Robot should only have one link, using first" << std::endl;
    }
    auto robot_link_entity = robot_link_entities[0];

    ecm.CreateComponent(
      _joint_entity,
      components::DetachableJoint({robot_link_entity,
                                   _entity, "fixed"}));
    ignmsg << "Added joint between " << _name << " and " << robot_name << std::endl;
    res.status = res.ACKNOWLEDGED;
    _dispenser_result_pub->publish(res);
    res.status = res.SUCCESS;
    _dispenser_result_pub->publish(res);
    _request_guid = "";
  }
  else
  {
    // DETACH
    ecm.RequestRemoveEntity(_joint_entity);
    _joint_entity = kNullEntity;
    ignmsg << "Removed joint between " << _name << " and " << robot_name << std::endl;
    IngestorResult res;
    res.time.sec = int(cur_t);
    res.time.nanosec = (cur_t - int(cur_t)) *1e9;
    res.request_guid = _request_guid;
    res.source_guid = _name;
    res.status = res.ACKNOWLEDGED;
    _ingestor_result_pub->publish(res);
    res.status = res.SUCCESS;
    _ingestor_result_pub->publish(res);
    _request_guid = "";
  }
}

//=================================================
void AttachableCartPlugin::PreUpdate(
  const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  double cur_t = info.simTime.count() / 1e9;
  rclcpp::spin_some(_ros_node);

  if (info.paused)
    return;

  if (_pending_request == AttachingRequest::ATTACH)
    AttachToRobot(ecm, _target_robot, true, cur_t);
  else if (_pending_request == AttachingRequest::DETACH)
    AttachToRobot(ecm, _target_robot, false, cur_t);
  // Reset the request
  _pending_request = AttachingRequest::NONE;


  if (cur_t - _last_state_pub > STATE_PUBLISH_DT)
  {
     _dispenser_state.time.sec = int(cur_t);
     _dispenser_state.time.nanosec = (cur_t - int(cur_t)) *1e9;
     _ingestor_state.time.sec = int(cur_t);
     _ingestor_state.time.nanosec = (cur_t - int(cur_t)) *1e9;
    _dispenser_state_pub->publish(_dispenser_state);
    _ingestor_state_pub->publish(_ingestor_state);
    _last_state_pub = cur_t;
  }
}

IGNITION_ADD_PLUGIN(
  AttachableCartPlugin,
  System,
  AttachableCartPlugin::ISystemConfigure,
  AttachableCartPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(AttachableCartPlugin,
  "attachable_cart")

} //namespace attachable_cart
