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

#include <regex>
#include <cstdlib>

#include <sdf/Actor.hh>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>

#include "crowd_simulator.hpp"

using namespace ignition::gazebo;

namespace crowd_simulation_ign {

//=================================================
void CrowdSimulatorPlugin::Configure(
  const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm,
  EventManager& /*event_mgr*/)
{
  _world = std::make_shared<Model>(entity);
  RCLCPP_INFO(_crowd_sim_interface->logger(),
    "Initializing world plugin with name: %s",
    _world->Name(ecm).c_str());
  _world_name = _world->Name(ecm);

  if (!_crowd_sim_interface->read_sdf(sdf))
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
      "Error loading crowd simulator plugin. Load params failed!");
    exit(EXIT_FAILURE);
  }

  if (!_crowd_sim_interface->enabled())
  {
    RCLCPP_WARN(_crowd_sim_interface->logger(),
      "CrowdSim is Disabled!");
    return;
  }

  if (!_crowd_sim_interface->init_crowd_sim())
  {
    RCLCPP_ERROR(_crowd_sim_interface->logger(),
      "Error loading crowd simulator plugin. Load [ Menge ] failed!");
    exit(EXIT_FAILURE);
  }

  if (!_spawn_agents_in_world())
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(),
      "Error loading crowd simulator plugin. Crowd Simulator failed to spawn agents in the world!");
    exit(EXIT_FAILURE);
  }

}

//=================================================
void CrowdSimulatorPlugin::PreUpdate(
  const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  // check if crowd sim is enabled
  if (!_crowd_sim_interface->enabled())
    return;

  // wait for all the models and actors loaded in ignition rendering
  if (!_initialized)
  {
    _init_spawned_agents(ecm);
    return;
  }

  // Don't update the plugin if the simulation is paused
  if (info.paused)
    return;

  // Note, the update_time_step parameter is ignored in ignition
  // through GPU animated actors the performance is good enough that
  // we can afford to update at every iteration and have smooth animations
  std::chrono::duration<double> delta_sim_time_tmp = info.simTime -
    _last_sim_time;
  double delta_sim_time = delta_sim_time_tmp.count();
  _last_sim_time = info.simTime;
  _crowd_sim_interface->one_step_sim(delta_sim_time);
  _update_all_objects(delta_sim_time, ecm);
}

//==========================================================
bool CrowdSimulatorPlugin::_spawn_agents_in_world()
{
  size_t object_count = this->_crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < object_count; ++id)
  {
    auto object_ptr = this->_crowd_sim_interface->get_object_by_id(id);
    assert(object_ptr);
    _object_dic[object_ptr->model_name] = id;

    if (!object_ptr->is_external)
    {
      auto type_ptr = _crowd_sim_interface->_model_type_db_ptr->get(
        object_ptr->type_name);
      assert(type_ptr);
      if (!this->_create_entity(object_ptr->model_name, type_ptr) )
      {
        RCLCPP_ERROR(_crowd_sim_interface->logger(),
          "Failed to insert model [ %s ] in world",
          object_ptr->model_name.c_str());
        return false;
      }
    }
  }
  return true;
}

//==========================================================
void CrowdSimulatorPlugin::_init_spawned_agents(
  EntityComponentManager& ecm)
{
  // check all the models are in the world
  std::unordered_map<std::string, size_t> objects_name;
  size_t object_count = _crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < object_count; id++)
  {
    auto obj = _crowd_sim_interface->get_object_by_id(id);
    // already found in the Dic
    if (_entity_dic.find(obj->model_name) != _entity_dic.end())
      continue;
    objects_name.insert({obj->model_name, id});
  }
  // for external agent
  ecm.Each<components::Model,
    components::Name>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name) -> bool
    {
      auto it_objects_name = objects_name.find(name->Data());
      if (it_objects_name != objects_name.end())
      {
        // update in entityDic
        _entity_dic[name->Data()] = entity;
        auto obj_ptr =
        _crowd_sim_interface->get_object_by_id(it_objects_name->second);
        // config internal spawned agent for custom trajectory
        if (!obj_ptr->is_external)
        {
          _config_spawned_agents(obj_ptr, entity, ecm);
        }
        objects_name.erase(name->Data());
        RCLCPP_INFO(_crowd_sim_interface->logger(),
        "Crowd Simulator found agent: %s", name->Data().c_str());
      }
      return true;
    }
    );
  // for internal agent
  ecm.Each<components::Actor,
    components::Name>(
    [&](const Entity& entity,
    const components::Actor*,
    const components::Name* name) -> bool
    {
      auto it_objects_name = objects_name.find(name->Data());
      if (it_objects_name != objects_name.end())
      {
        // update in entityDic
        _entity_dic[name->Data()] = entity;
        auto obj_ptr =
        _crowd_sim_interface->get_object_by_id(it_objects_name->second);
        // config internal spawned agent for custom trajectory
        if (!obj_ptr->is_external)
        {
          _config_spawned_agents(obj_ptr, entity, ecm);
        }
        objects_name.erase(name->Data());
        RCLCPP_INFO(_crowd_sim_interface->logger(),
        "Crowd Simulator found agent: %s",
        name->Data().c_str());
      }
      return true;
    }
    );

  // external agents not found or not loaded yet
  if (objects_name.size() != 0)
  {
    _initialized = false;
    return;
  }
  _initialized = true;
  RCLCPP_INFO(
    _crowd_sim_interface->logger(),
    "Ignition Models are all loaded! Start simulating...");
}

//===================================================================
bool CrowdSimulatorPlugin::_create_entity(
  const std::string& model_name,
  const crowd_simulator::ModelTypeDatabase::RecordPtr model_type_ptr) const
{
  // Use ignition create service to spawn actors
  // calling ignition gazebo create service, you can use "ign service -l" to
  // check the service available
  assert(model_type_ptr);
  std::string service = "/world/" + this->_world_name + "/create";
  ignition::msgs::EntityFactory request;
  request.set_sdf_filename(model_type_ptr->file_name);
  request.set_name(model_name);
  ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);

  ignition::msgs::Boolean response;
  bool result;
  uint32_t timeout = 5000;
  bool executed = this->_transport_node_ptr->Request(service, request, timeout,
      response, result);
  if (executed)
  {
    if (result && response.data())
    {
      RCLCPP_INFO(_crowd_sim_interface->logger(),
        "Requested creation of entity: %s",
        model_name.c_str());
      return true;
    }
    else
    {
      RCLCPP_ERROR(_crowd_sim_interface->logger(),
        "Failed request to create entity.\n %s",
        request.DebugString().c_str());
    }
  }
  else
  {
    RCLCPP_ERROR(
      _crowd_sim_interface->logger(),
      "Request to create entity from service %s timer out ...\n",
      request.DebugString().c_str());
  }
  return false;
}

//==================================================
void CrowdSimulatorPlugin::_config_spawned_agents(
  const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
  const Entity& entity,
  EntityComponentManager& ecm) const
{
  assert(obj_ptr);
  auto agent_ptr = obj_ptr->agent_ptr;
  auto model_type = _crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name);
  // different from gazebo plugin, the pose component is the origin of the trajPose
  ignition::math::Pose3d actor_pose(
    static_cast<double>(agent_ptr->_pos.x()),
    static_cast<double>(agent_ptr->_pos.y()), 0.0,
    0, 0, 0
  );

  // initialize agent animationName
  std::string animation_name = model_type->animation;
  assert(!animation_name.empty());

  enableComponent<components::AnimationName>(ecm, entity);
  ecm.Component<components::AnimationName>(entity)->Data() = animation_name;

  // check idle animation name
  auto actor_comp =
    ecm.Component<components::Actor>(entity);
  for (auto idle_anim : _crowd_sim_interface->get_switch_anim_name())
  {
    if (actor_comp->Data().AnimationNameExists(idle_anim))
    {
      model_type->idle_animation = idle_anim;
      break;
    }
  }

  // mark as one-time-change
  ecm.SetChanged(
    entity,
    components::AnimationName::typeId,
    ComponentState::OneTimeChange);
  // initialize agent animationTime
  enableComponent<components::AnimationTime>(ecm, entity);
  // having a trajectory pose prevents the actor from moving with the sdf script
  enableComponent<components::TrajectoryPose>(ecm, entity);
}

//============================================================================
void CrowdSimulatorPlugin::_update_all_objects(
  double delta_sim_time,
  EntityComponentManager& ecm) const
{
  auto objects_count = _crowd_sim_interface->get_num_objects();
  for (size_t id = 0; id < objects_count; id++)
  {
    auto obj_ptr = _crowd_sim_interface->get_object_by_id(id);
    auto it_entity = _entity_dic.find(obj_ptr->model_name);
    if (it_entity == _entity_dic.end())   //safe check
    {
      RCLCPP_ERROR(_crowd_sim_interface->logger(),
        "Didn't initialize external agent [ %s ]",
        obj_ptr->model_name.c_str());
      exit(EXIT_FAILURE);
    }
    auto entity = it_entity->second;

    // for external agent
    if (obj_ptr->is_external)
    {
      auto model_pose =
        ecm.Component<components::Pose>(entity)->Data();
      _crowd_sim_interface->update_external_agent(obj_ptr->agent_ptr,
        model_pose);
      continue;
    }

    // for internal agent
    _update_internal_object(delta_sim_time, obj_ptr, entity, ecm);
  }
}

void CrowdSimulatorPlugin::_update_internal_object(
  double delta_sim_time,
  const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
  const Entity& entity,
  EntityComponentManager& ecm) const
{
  double animation_speed = _crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name)->animation_speed;
  ignition::math::Pose3d initial_pose =
    _crowd_sim_interface->_model_type_db_ptr->get(obj_ptr->type_name)->pose.
    convert_to_ign_math_pose_3d<ignition::math::Pose3d>();
  ignition::math::Pose3d agent_pose =
    _crowd_sim_interface->get_agent_pose<ignition::math::Pose3d>(
    obj_ptr->agent_ptr, delta_sim_time);
  agent_pose += initial_pose;

  // get components to be updated
  auto traj_pose_comp =
    ecm.Component<components::TrajectoryPose>(entity);
  auto anim_name_comp =
    ecm.Component<components::AnimationName>(entity);
  auto anim_time_comp =
    ecm.Component<components::AnimationTime>(entity);
  ignition::math::Pose3d current_pose = traj_pose_comp->Data();
  auto distance_traveled_vector = agent_pose.Pos() - current_pose.Pos();
  // might need future work on 3D case
  // the center of human has a z_elevation, which will make the human keep walking even if he reached the target
  distance_traveled_vector.Z(0.0);
  double distance_traveled = distance_traveled_vector.Length();

  // switch animation
  auto model_type = _crowd_sim_interface->_model_type_db_ptr->get(
    obj_ptr->type_name);
  AnimState next_state = obj_ptr->get_next_state(
    distance_traveled < _crowd_sim_interface->get_switch_anim_distance_th() &&
    !model_type->idle_animation.empty());

  switch (next_state)
  {
    case AnimState::WALK:
      anim_time_comp->Data() +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(distance_traveled / animation_speed));
      anim_name_comp->Data() = model_type->animation;
      break;

    case AnimState::IDLE:
      anim_time_comp->Data() +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(delta_sim_time));
      agent_pose.Rot() = current_pose.Rot();
      anim_name_comp->Data() = model_type->idle_animation;
      break;
  }

  ecm.SetChanged(entity,
    components::AnimationName::typeId,
    ComponentState::PeriodicChange);
  obj_ptr->current_state = next_state;

  // set trajectory
  traj_pose_comp->Data() = agent_pose;
  ecm.SetChanged(entity,
    components::TrajectoryPose::typeId,
    ComponentState::PeriodicChange);
  ecm.SetChanged(entity,
    components::AnimationTime::typeId,
    ComponentState::PeriodicChange);
}

IGNITION_ADD_PLUGIN(
  CrowdSimulatorPlugin,
  System,
  CrowdSimulatorPlugin::ISystemConfigure,
  CrowdSimulatorPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(CrowdSimulatorPlugin,
  "crowd_simulation")

} //namespace crowd_simulation_ign
