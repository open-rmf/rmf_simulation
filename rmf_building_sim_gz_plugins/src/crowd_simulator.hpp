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

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

#include <gz/transport/Node.hh>

#include <gz/msgs/entity_factory.pb.h>

#include <rmf_building_sim_common/crowd_simulator_common.hpp>


namespace crowd_simulation_gz {

class GZ_SIM_VISIBLE CrowdSimulatorPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  using AnimState = crowd_simulator::CrowdSimInterface::AnimState;
public:
  CrowdSimulatorPlugin()
  : _transport_node_ptr(std::make_shared<gz::transport::Node>()),
    _crowd_sim_interface(std::make_shared<crowd_simulator::CrowdSimInterface>()),
    _initialized(false)
  {}

  // inherit from ISystemConfigure
  void Configure(const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_mgr) override;

  // inherit from ISystemPreUpdate
  void PreUpdate(const gz::sim::UpdateInfo& info,
    gz::sim::EntityComponentManager& ecm) override;

private:
  std::shared_ptr<gz::transport::Node> _transport_node_ptr;
  std::shared_ptr<crowd_simulator::CrowdSimInterface> _crowd_sim_interface;
  bool _initialized;
  std::chrono::steady_clock::duration _last_sim_time{0};

  std::shared_ptr<gz::sim::Model> _world;
  std::string _world_name;

  // map for <model_name, object_id>, contains both external (models) and internal agents (actors)
  std::unordered_map<std::string, size_t> _object_dic;
  // map for <model_name, entity_id> contains external and internal agents
  std::unordered_map<std::string, gz::sim::Entity> _entity_dic;

  bool _spawn_agents_in_world();
  void _init_spawned_agents(gz::sim::EntityComponentManager& ecm);
  void _config_spawned_agents(
    const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
    const gz::sim::Entity& enity,
    gz::sim::EntityComponentManager& ecm) const;
  bool _create_entity(
    const std::string& model_name,
    const crowd_simulator::ModelTypeDatabase::RecordPtr model_type_ptr) const;
  void _update_all_objects(
    double delta_sim_time,
    gz::sim::EntityComponentManager& ecm) const;
  void _update_internal_object(
    double delta_sim_time,
    const crowd_simulator::CrowdSimInterface::ObjectPtr obj_ptr,
    const gz::sim::Entity& enity,
    gz::sim::EntityComponentManager& ecm) const;
};

} //namespace crowd_simulation_gz
