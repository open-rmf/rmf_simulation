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

#include <vector>
#include <unordered_map>

#include <gz/plugin/Register.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/AxisAlignedBox.hh>

#include <gz/math/AxisAlignedBox.hh>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

#include <rmf_robot_sim_common/utils.hpp>

using namespace gz::sim;
using namespace rmf_plugins_utils;

namespace rmf_robot_sim_gz_plugins {

class GZ_SIM_VISIBLE TeleportDispenserPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  TeleportDispenserPlugin();
  ~TeleportDispenserPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  gz::transport::Node _ign_node;

  Entity _dispenser;
  Entity _item_en; // Item that dispenser may contain
  gz::math::AxisAlignedBox _dispenser_vicinity_box;

  bool _dispense = false;
  DispenserRequest _latest; // Only store and act on last received request

  std::string _guid; // Plugin name

  double _last_pub_time = 0.0;
  double _sim_time = 0.0;

  bool _item_en_found = false; // True if entity to be dispensed has been determined. Used when locating item in future
  bool _dispenser_filled = false;

  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;
  DispenserState _current_state;
  rclcpp::Node::SharedPtr _ros_node;

  bool tried_fill_dispenser = false; // Set to true if fill_dispenser() has been called at least once

  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;
  std::unordered_map<std::string, bool> _past_request_guids;

  void init_ros_node();
  void send_dispenser_response(uint8_t status) const;
  void fleet_state_cb(FleetState::UniquePtr msg);
  void dispenser_request_cb(DispenserRequest::UniquePtr msg);
  void on_update(EntityComponentManager& ecm);

  void try_refill_dispenser(EntityComponentManager& ecm);
  SimEntity find_nearest_model(
    EntityComponentManager& ecm,
    const std::vector<SimEntity>& entities, bool& found) const;
  void place_on_entity(EntityComponentManager& ecm,
    const SimEntity& obj, const Entity& to_move);
  void fill_robot_list(EntityComponentManager& ecm,
    FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_list);
  bool dispense_on_nearest_robot(EntityComponentManager& ecm,
    const std::string& fleet_name);
  void fill_dispenser(EntityComponentManager& ecm);
  void create_dispenser_bounding_box(EntityComponentManager& ecm);
};

TeleportDispenserPlugin::TeleportDispenserPlugin()
{
}

TeleportDispenserPlugin::~TeleportDispenserPlugin()
{
  rclcpp::shutdown();
}

void TeleportDispenserPlugin::init_ros_node()
{
  if (!_ros_node)
  {
    RCLCPP_ERROR(_ros_node->get_logger(),
      "No ROS node created for TeleportDispenser plugin!");
    return;
  }

  _fleet_state_sub = _ros_node->create_subscription<FleetState>(
    "/fleet_states",
    rclcpp::SystemDefaultsQoS().keep_last(10),
    std::bind(&TeleportDispenserPlugin::fleet_state_cb, this,
    std::placeholders::_1));

  _state_pub = _ros_node->create_publisher<DispenserState>(
    "/dispenser_states", 10);

  _request_sub = _ros_node->create_subscription<DispenserRequest>(
    "/dispenser_requests",
    rclcpp::SystemDefaultsQoS().keep_last(10).reliable(),
    std::bind(&TeleportDispenserPlugin::dispenser_request_cb, this,
    std::placeholders::_1));

  _result_pub = _ros_node->create_publisher<DispenserResult>(
    "/dispenser_results", 10);

  _current_state.guid = _guid;
  _current_state.mode = DispenserState::IDLE;
}

void TeleportDispenserPlugin::send_dispenser_response(uint8_t status) const
{
  auto response = make_response<DispenserResult>(
    status, _sim_time, _latest.request_guid, _guid);
  _result_pub->publish(*response);
}

void TeleportDispenserPlugin::fleet_state_cb(FleetState::UniquePtr msg)
{
  _fleet_states[msg->name] = std::move(msg);
}

void TeleportDispenserPlugin::dispenser_request_cb(
  DispenserRequest::UniquePtr msg)
{
  _latest = *msg;

  if (_guid == _latest.target_guid)
  {
    // check if task has been completed previously
    const auto it = _past_request_guids.find(_latest.request_guid);
    if (it != _past_request_guids.end())
    {
      if (it->second)
      {
        RCLCPP_WARN(_ros_node->get_logger(),
          "Request already succeeded: [%s]", _latest.request_guid.c_str());
        send_dispenser_response(DispenserResult::SUCCESS);
      }
      else
      {
        RCLCPP_WARN(_ros_node->get_logger(),
          "Request already failed: [%s]", _latest.request_guid.c_str());
        send_dispenser_response(DispenserResult::FAILED);
      }
      return;
    }

    _dispense = true; // Mark true to dispense item next time PreUpdate() is called
  }
}

void TeleportDispenserPlugin::on_update(EntityComponentManager& ecm)
{
  try_refill_dispenser(ecm);

  // periodic pub on dispenser state
  constexpr double interval = 2.0;
  if (_sim_time - _last_pub_time >= interval || _dispense)
  {
    _last_pub_time = _sim_time;
    _current_state.time = simulation_now(_sim_time);

    if (_dispense)
    {
      _current_state.mode = DispenserState::BUSY;
      _current_state.request_guid_queue = {_latest.request_guid};
    }
    else
    {
      _current_state.mode = DispenserState::IDLE;
      _current_state.request_guid_queue.clear();
    }
    _state_pub->publish(_current_state);
  }

  // `dispense` is set to true if the dispenser plugin node has
  // received a valid DispenserRequest
  if (_dispense)
  {
    send_dispenser_response(DispenserResult::ACKNOWLEDGED);

    bool is_success = false;
    if (_dispenser_filled)
    {
      RCLCPP_INFO(_ros_node->get_logger(), "Dispensing item");
      bool res = dispense_on_nearest_robot(ecm, _latest.transporter_type);
      if (res)
      {
        is_success = true;
        send_dispenser_response(DispenserResult::SUCCESS);
        RCLCPP_INFO(_ros_node->get_logger(), "Success");
      }
      else
      {
        send_dispenser_response(DispenserResult::FAILED);
        RCLCPP_WARN(_ros_node->get_logger(), "Unable to dispense item");
      }
    }
    else
    {
      RCLCPP_WARN(_ros_node->get_logger(),
        "No item to dispense: [%s]", _latest.request_guid.c_str());
      send_dispenser_response(DispenserResult::FAILED);
    }

    _past_request_guids.emplace(_latest.request_guid, is_success);

    _dispense = false;
  }
}

void TeleportDispenserPlugin::try_refill_dispenser(EntityComponentManager& ecm)
{
  constexpr double interval = 2.0;
  if (_sim_time - _last_pub_time >= interval)
  {
    // Occasionally check to see if dispensed item has been returned to it
    if (!_dispenser_filled && _item_en_found &&
      _dispenser_vicinity_box.Contains(
      ecm.Component<components::Pose>(_item_en)->Data().Pos()))
    {
      _dispenser_filled = true;
    }
  }
}

bool TeleportDispenserPlugin::dispense_on_nearest_robot(
  EntityComponentManager& ecm,
  const std::string& fleet_name)
{
  if (!_dispenser_filled)
    return false;

  const auto fleet_state_it = _fleet_states.find(fleet_name);
  if (fleet_state_it == _fleet_states.end())
  {
    RCLCPP_WARN(_ros_node->get_logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return false;
  }

  std::vector<SimEntity> robot_list;
  fill_robot_list(ecm, fleet_state_it, robot_list);

  bool found = false;
  SimEntity robot_model = find_nearest_model(ecm, robot_list, found);
  if (!found)
  {
    RCLCPP_WARN(_ros_node->get_logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return false;
  }

  place_on_entity(ecm, robot_model, _item_en);
  _dispenser_filled = false; // Assumes Dispenser is configured to only dispense a single object
  return true;
}

SimEntity TeleportDispenserPlugin::find_nearest_model(
  EntityComponentManager& ecm,
  const std::vector<SimEntity>& entities,
  bool& found) const
{
  double nearest_dist = 1e6;
  SimEntity robot_entity(0); // Placeholder value
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  for (const auto& sim_obj : entities)
  {
    Entity en = sim_obj.get_entity();
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(dispenser_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = sim_obj;
      found = true;
    }
  }
  return robot_entity;
}

// Move entity `to_move` onto `base_obj`
void TeleportDispenserPlugin::place_on_entity(EntityComponentManager& ecm,
  const SimEntity& base_obj, const Entity& to_move)
{
  const Entity base = base_obj.get_entity();
  auto new_pose = ecm.Component<components::Pose>(base)->Data();

  // Make service request to Slotcar to get its height instead of accessing
  // it's AABB component directly
  gz::msgs::Entity req;
  req.set_id(base);

  const unsigned int timeout = 5000;
  bool result = false;
  gz::msgs::Double rep;
  const std::string height_srv_name = "/slotcar_height_" + std::to_string(base);

  bool executed = _ign_node.Request(height_srv_name, req, timeout, rep, result);
  if (executed && result)
  {
    // Assumes that the base pose's Z value refers to bottom of object
    new_pose.SetZ(ecm.Component<components::Pose>(base)->Data().Z()
      + rep.data());
  }
  else
  {
    RCLCPP_WARN(
      _ros_node->get_logger(),
      "Either base entity or item to be dispensed does not have an AxisAlignedBox component. \
      Attempting to dispense item to approximate location.");
    new_pose = gz::math::Pose3<double>(0, 0, 0.5, 0, 0, 0) * new_pose;
  }

  enableComponent<components::WorldPoseCmd>(ecm, to_move);
  ecm.Component<components::WorldPoseCmd>(to_move)->Data() = new_pose;
}

void TeleportDispenserPlugin::fill_robot_list(EntityComponentManager& ecm,
  FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_list)
{
  for (const auto& rs : fleet_state_it->second->robots)
  {
    std::vector<Entity> entities =
      ecm.EntitiesByComponents(components::Name(rs.name),
        components::Model(), components::Static(false));
    for (Entity& en : entities)
    {
      robot_list.push_back(SimEntity(en));
    }
  }
}

// Searches vicinity of Dispenser for closest valid item. If found, the item is assigned to `_item_en`
void TeleportDispenserPlugin::fill_dispenser(EntityComponentManager& ecm)
{
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  double nearest_dist = 1.0;
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& en,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _guid)
      {
        const auto dist = pose->Data().Pos().Distance(dispenser_pos);

        if (dist < nearest_dist
        && _dispenser_vicinity_box.Contains(pose->Data().Pos()))
        {
          _item_en = en;
          nearest_dist = dist;
          _dispenser_filled = true;
          _item_en_found = true;
        }
      }
      return true;
    });

  if (!_dispenser_filled)
  {
    RCLCPP_WARN(_ros_node->get_logger(),
      "Could not find dispenser item model within 1 meter, "
      "this dispenser will not be operational");
  }
  else
  {
    RCLCPP_INFO(_ros_node->get_logger(),
      "Found dispenser item: [%s]",
      ecm.Component<components::Name>(_item_en)->Data().c_str());
  }
}

void TeleportDispenserPlugin::create_dispenser_bounding_box(
  EntityComponentManager& ecm)
{
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();
  gz::math::Vector3d corner_1(dispenser_pos.X() - 0.05,
    dispenser_pos.Y() - 0.05, dispenser_pos.Z() - 0.05);
  gz::math::Vector3d corner_2(dispenser_pos.X() + 0.05,
    dispenser_pos.Y() + 0.05, dispenser_pos.Z() + 0.05);
  _dispenser_vicinity_box = gz::math::AxisAlignedBox(corner_1, corner_2);
}

void TeleportDispenserPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::ok())
    rclcpp::init(0, argv);

  _dispenser = entity;
  _guid = ecm.Component<components::Name>(_dispenser)->Data();
  gzwarn << "Initializing plugin with name " << _guid << std::endl;

  _ros_node = std::make_shared<rclcpp::Node>(_guid);
  init_ros_node();
  RCLCPP_INFO(_ros_node->get_logger(),
    "Started TeleportDispenserPlugin node...");

  create_dispenser_bounding_box(ecm);
}

void TeleportDispenserPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  _sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);

  // Don't update the pose if the simulation is paused
  if (info.paused)
    return;

  // Set item that the Dispenser will be configured to dispense. Do this only on first PreUpdate() call.
  // Happens here and not in Configure() to allow for all models to load
  if (!tried_fill_dispenser)
  {
    fill_dispenser(ecm);
    tried_fill_dispenser = true;
  }

  on_update(ecm);
}

GZ_ADD_PLUGIN(
  TeleportDispenserPlugin,
  System,
  TeleportDispenserPlugin::ISystemConfigure,
  TeleportDispenserPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
GZ_ADD_PLUGIN_ALIAS(TeleportDispenserPlugin, "teleport_dispenser")

} // namespace rmf_robot_sim_gz_plugins
