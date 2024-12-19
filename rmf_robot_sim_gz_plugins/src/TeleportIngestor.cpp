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
#include <functional>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>

#include <rmf_robot_sim_common/utils.hpp>

using namespace gz::sim;
using namespace rmf_plugins_utils;

namespace rmf_robot_sim_gz_plugins {

class GZ_SIM_VISIBLE TeleportIngestorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;
  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;

  TeleportIngestorPlugin();
  ~TeleportIngestorPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  bool _non_static_models_filled = false;
  Entity _ingestor;
  Entity _ingested_entity; // Item that ingestor may contain

  // Ingest request params
  bool _ingest = false;
  IngestorRequest _latest;

  // Ingestor params
  std::string _guid;
  bool _ingestor_filled = false;

  double _last_pub_time = 0.0;
  double _last_ingested_time = 0.0;
  double _sim_time = 0.0;

  rclcpp::Node::SharedPtr _ros_node;
  gz::transport::Node _gz_node;

  std::unordered_map<std::string, Eigen::Isometry3d>
  _non_static_models_init_poses;
  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;
  IngestorState _current_state;

  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<IngestorState>::SharedPtr _state_pub;
  rclcpp::Subscription<IngestorRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<IngestorResult>::SharedPtr _result_pub;
  std::unordered_map<std::string, bool> _past_request_guids;

  void init_ros_node();
  void send_ingestor_response(uint8_t status) const;
  void fleet_state_cb(FleetState::UniquePtr msg);
  void ingestor_request_cb(IngestorRequest::UniquePtr msg);
  void on_update(EntityComponentManager& ecm);
  bool ingest_from_nearest_robot(EntityComponentManager& ecm,
    const std::string& fleet_name);

  SimEntity find_nearest_model(const EntityComponentManager& ecm,
    const std::vector<SimEntity>& robot_model_entities,
    bool& found) const;
  bool get_payload_model(const EntityComponentManager& ecm,
    const SimEntity& robot_sim_entity,
    Entity& payload_entity);
  void fill_robot_list(EntityComponentManager& ecm,
    FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_list);
  void transport_model(EntityComponentManager& ecm);
  void send_ingested_item_home(EntityComponentManager& ecm);
  void init_non_static_models_poses(EntityComponentManager& ecm);
};

void TeleportIngestorPlugin::init_ros_node()
{
  if (!_ros_node)
  {
    RCLCPP_ERROR(_ros_node->get_logger(),
      "No ROS node created for TeleportIngestor plugin!");
    return;
  }

  _fleet_state_sub = _ros_node->create_subscription<FleetState>(
    "/fleet_states",
    rclcpp::SystemDefaultsQoS().keep_last(10),
    std::bind(&TeleportIngestorPlugin::fleet_state_cb, this,
    std::placeholders::_1));

  _state_pub = _ros_node->create_publisher<IngestorState>(
    "/ingestor_states", 10);

  _request_sub = _ros_node->create_subscription<IngestorRequest>(
    "/ingestor_requests",
    rclcpp::SystemDefaultsQoS().keep_last(10).reliable(),
    std::bind(&TeleportIngestorPlugin::ingestor_request_cb, this,
    std::placeholders::_1));

  _result_pub = _ros_node->create_publisher<IngestorResult>(
    "/ingestor_results", 10);

  _current_state.guid = _guid;
  _current_state.mode = IngestorState::IDLE;
}

void TeleportIngestorPlugin::send_ingestor_response(uint8_t status) const
{
  auto response = make_response<IngestorResult>(
    status, _sim_time, _latest.request_guid, _guid);
  _result_pub->publish(*response);
}

void TeleportIngestorPlugin::fleet_state_cb(FleetState::UniquePtr msg)
{
  _fleet_states[msg->name] = std::move(msg);
}

void TeleportIngestorPlugin::ingestor_request_cb(IngestorRequest::UniquePtr msg)
{
  _latest = *msg;

  if (_guid == _latest.target_guid && !_ingestor_filled)
  {
    const auto it = _past_request_guids.find(_latest.request_guid);
    if (it != _past_request_guids.end())
    {
      if (it->second)
      {
        RCLCPP_WARN(_ros_node->get_logger(),
          "Request already succeeded: [%s]", _latest.request_guid.c_str());
        send_ingestor_response(IngestorResult::SUCCESS);
      }
      else
      {
        RCLCPP_WARN(_ros_node->get_logger(),
          "Request already failed: [%s]", _latest.request_guid.c_str());
        send_ingestor_response(IngestorResult::FAILED);
      }
      return;
    }

    _ingest = true; // Mark true to ingest item next time PreUpdate() is called
  }
}

void TeleportIngestorPlugin::on_update(EntityComponentManager& ecm)
{
  // periodic ingestor status publisher
  constexpr double interval = 2.0;
  if (_sim_time - _last_pub_time >= interval || _ingest)
  {
    _last_pub_time = _sim_time;
    _current_state.time = simulation_now(_sim_time);

    if (_ingest)
    {
      _current_state.mode = IngestorState::BUSY;
      _current_state.request_guid_queue = {_latest.request_guid};
    }
    else
    {
      _current_state.mode = IngestorState::IDLE;
      _current_state.request_guid_queue.clear();
    }
    _state_pub->publish(_current_state);
  }

  if (_ingest)
  {
    send_ingestor_response(IngestorResult::ACKNOWLEDGED);

    bool is_success = false;
    if (!_ingestor_filled)
    {
      RCLCPP_INFO(_ros_node->get_logger(), "Ingesting item");
      bool res = ingest_from_nearest_robot(ecm, _latest.transporter_type);
      if (res)
      {
        send_ingestor_response(IngestorResult::SUCCESS);
        _last_ingested_time = _sim_time;
        is_success = true;
        RCLCPP_INFO(_ros_node->get_logger(), "Success");
      }
      else
      {
        send_ingestor_response(IngestorResult::FAILED);
        RCLCPP_WARN(_ros_node->get_logger(), "Unable to dispense item");
      }
    }
    else
    {
      RCLCPP_WARN(_ros_node->get_logger(),
        "No item to ingest: [%s]", _latest.request_guid.c_str());
      send_ingestor_response(IngestorResult::FAILED);
    }

    _past_request_guids.emplace(_latest.request_guid, is_success);

    _ingest = false;
  }

  // Periodically try to teleport ingested item back to original location
  constexpr double return_interval = 5.0;
  if (_sim_time - _last_ingested_time >=
    return_interval && _ingestor_filled)
  {
    send_ingested_item_home(ecm);
  }
}

bool TeleportIngestorPlugin::ingest_from_nearest_robot(
  EntityComponentManager& ecm,
  const std::string& fleet_name)
{
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

  if (!get_payload_model(ecm, robot_model, _ingested_entity))
  {
    RCLCPP_WARN(_ros_node->get_logger(),
      "No delivery item found on the robot.");
    return false;
  }

  transport_model(ecm);
  _ingestor_filled = true;
  return true;
}

SimEntity TeleportIngestorPlugin::find_nearest_model(
  const EntityComponentManager& ecm,
  const std::vector<SimEntity>& entities,
  bool& found) const
{
  double nearest_dist = 1e6;
  SimEntity robot_entity(0);
  const auto ingestor_pos =
    ecm.Component<components::Pose>(_ingestor)->Data().Pos();

  for (const auto& sim_obj : entities)
  {
    // If `models` has been generated with `fill_robot_list`, it is
    // guaranteed to have a valid entity field
    Entity en = sim_obj.get_entity();
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(ingestor_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = sim_obj;
      found = true;
    }
  }
  return robot_entity;
}

// Identifies item to ingest and assigns it to `payload_entity`
bool TeleportIngestorPlugin::get_payload_model(
  const EntityComponentManager& ecm,
  const SimEntity& robot_sim_entity,
  Entity& payload_entity)
{
  // There might not be a better way to loop through all the models, as we
  // might consider delivering items that were spawned during run time,
  // instead of during launch.
  const Entity robot_entity = robot_sim_entity.get_entity();
  const auto robot_model_pos =
    ecm.Component<components::Pose>(robot_entity)->Data().Pos();
  double nearest_dist = 1.0;
  bool found = false;

  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _guid
      && name->Data() != Model(robot_entity).Name(ecm))
      {
        const double dist = pose->Data().Pos().Distance(robot_model_pos);
        if (dist < nearest_dist)
        {
          payload_entity = entity;
          nearest_dist = dist;
          found = true;
        }
      }
      return true;
    });

  return found;
}

void TeleportIngestorPlugin::fill_robot_list(EntityComponentManager& ecm,
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

// Moves the identified item to ingest from its current position to the ingestor
void TeleportIngestorPlugin::transport_model(EntityComponentManager& ecm)
{
  // Ingestor assumes control of entity. Set its pose and cancel any pre-existing velocity cmds
  enableComponent<components::WorldPoseCmd>(ecm, _ingested_entity);
  auto new_pose = ecm.Component<components::Pose>(_ingestor)->Data();
  ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = new_pose;

  if (ecm.EntityHasComponentType(_ingested_entity,
    components::LinearVelocityCmd().TypeId()))
  {
    ecm.Component<components::LinearVelocityCmd>(_ingested_entity)->Data() = {
      0, 0, 0};
  }
  if (ecm.EntityHasComponentType(_ingested_entity,
    components::AngularVelocityCmd().TypeId()))
  {
    ecm.Component<components::AngularVelocityCmd>(_ingested_entity)->Data() = {
      0, 0, 0};
  }
}

void TeleportIngestorPlugin::send_ingested_item_home(
  EntityComponentManager& ecm)
{
  if (_ingestor_filled)
  {
    const auto it =
      _non_static_models_init_poses.find(
      Model(_ingested_entity).Name(ecm));
    if (it == _non_static_models_init_poses.end())
    {
      ecm.RequestRemoveEntity(_ingested_entity);
    }
    else
    {
      enableComponent<components::WorldPoseCmd>(ecm, _ingested_entity);
      auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
      if (!cmd)
      {
        ecm.CreateComponent(_ingested_entity,
          components::WorldPoseCmd(gz::math::Pose3<double>()));
      }
      ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() =
        convert_to_pose<gz::math::Pose3d>(it->second);
    }
    _ingestor_filled = false;
  }
}

void TeleportIngestorPlugin::init_non_static_models_poses(
  EntityComponentManager& ecm)
{
  // Keep track of all the non-static models
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity&,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _guid)
      {
        _non_static_models_init_poses[name->Data()] =
        convert_pose(
          pose->Data());
      }
      return true;
    });
}

TeleportIngestorPlugin::TeleportIngestorPlugin()
{
}

TeleportIngestorPlugin::~TeleportIngestorPlugin()
{
  rclcpp::shutdown();
}

void TeleportIngestorPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::ok())
    rclcpp::init(0, argv);

  _ingestor = entity;
  _guid = ecm.Component<components::Name>(_ingestor)->Data();
  gzwarn << "Initializing plugin with name " << _guid << std::endl;
  _ros_node = std::make_shared<rclcpp::Node>(_guid);
  init_ros_node();
  RCLCPP_INFO(_ros_node->get_logger(),
    "Started TeleportIngestorPlugin node...");
}

void TeleportIngestorPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  _sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();

  if (!_non_static_models_filled)
  {
    // Initialize here and not in configure to allow all models to load
    init_non_static_models_poses(ecm);
    _non_static_models_filled = true;
  }

  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);

  // Don't update the pose if the simulation is paused
  if (info.paused)
    return;

  on_update(ecm);
}

GZ_ADD_PLUGIN(
  TeleportIngestorPlugin,
  System,
  TeleportIngestorPlugin::ISystemConfigure,
  TeleportIngestorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
GZ_ADD_PLUGIN_ALIAS(TeleportIngestorPlugin, "teleport_ingestor")

} // namespace rmf_robot_sim_gz_plugins
