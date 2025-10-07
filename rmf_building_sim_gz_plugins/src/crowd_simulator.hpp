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

#include <MengeCore/Runtime/SimulatorDB.h>
#include <MengeCore/Orca/ORCADBEntry.h>
#include <MengeCore/Orca/ORCASimulator.h>
#include <MengeCore/PluginEngine/CorePluginEngine.h>


namespace crowd_simulation_gz {

using AgentPtr = std::shared_ptr<Menge::Agents::BaseAgent>;

enum class AnimState
{
  WALK,
  IDLE,
};

//================================================================
/*
* class Object
*/
struct Object
{
  AgentPtr agent_ptr;
  std::string model_name;
  std::string type_name;
  bool is_external = false;
  AnimState current_state;
  AnimState get_next_state(bool condition);
};
using ObjectPtr = std::shared_ptr<Object>;

class AgentPose3d
{
public:
  AgentPose3d()
  : _x(0), _y(0), _z(0), _roll(0), _pitch(0), _yaw(0)
  {}
  AgentPose3d(double x, double y, double z, double roll, double pitch,
    double yaw)
  : _x(x), _y(y), _z(z), _roll(roll), _pitch(pitch), _yaw(yaw)
  {}

  double x() const {return _x;}
  double y() const {return _y;}
  double z() const {return _z;}
  double roll() const {return _roll;}
  double pitch() const {return _pitch;}
  double yaw() const {return _yaw;}

  void x(double x) {_x = x;}
  void y(double y) {_y = y;}
  void z(double z) {_z = z;}
  void roll(double roll) {_roll = roll;}
  void pitch(double pitch) {_pitch = pitch;}
  void yaw(double yaw) {_yaw = yaw;}

  template<typename IgnMathPose3d>
  IgnMathPose3d convert_to_ign_math_pose_3d()
  {
    return IgnMathPose3d(_x, _y, _z, _roll, _pitch, _yaw);
  }

private:
  double _x, _y, _z, _roll, _pitch, _yaw;
};

//================================================================
/*
* class ModelTypeDatabase
*/
class ModelTypeDatabase
{
public:
  struct Record
  {
    std::string type_name;
    std::string file_name;
    AgentPose3d pose;
    std::string animation;
    std::string idle_animation;
    double animation_speed;
  };

  using RecordPtr = std::shared_ptr<Record>;

  //Create a new record and returns a reference to the record
  RecordPtr emplace(std::string type_name, RecordPtr record_ptr);
  size_t size() const;
  RecordPtr get(const std::string& type_name) const;

private:
  std::unordered_map<std::string, RecordPtr> _records;
};

//================================================================
/*
* class MengeHandle, provides a wrap-up class handle for actual menge lib
* only exposing several menge function interface
*/
class MengeHandle : public std::enable_shared_from_this<MengeHandle>
{
public:

  static std::shared_ptr<MengeHandle> init_and_make(
    const std::string& resource_path,
    const std::string& behavior_file,
    const std::string& scene_file,
    const float sim_time_step
  );

  MengeHandle(const std::string& resource_path,
    const std::string& behavior_file,
    const std::string& scene_file,
    const float sim_time_step = 0.0
  )
  : _resource_path(resource_path),
    _behavior_file(behavior_file),
    _scene_file(scene_file),
    _sim_time_step(sim_time_step),
    _agent_count(0)
  {
    _behavior_file = this->_resource_file_path(_behavior_file);
    _scene_file = this->_resource_file_path(_scene_file);
  }

  void set_sim_time_step(float sim_time_step);
  float get_sim_time_step() const;
  size_t get_agent_count();
  void sim_step() const; //proceed one-time simulation step in _sim
  AgentPtr get_agent(size_t id) const;

private:
  std::string _resource_path;
  std::string _behavior_file;
  std::string _scene_file;
  float _sim_time_step;
  size_t _agent_count;
  std::shared_ptr<Menge::Agents::SimulatorInterface> _sim;

  std::string _resource_file_path(const std::string& relative_path) const;
  bool _load_simulation(); //initialize simulatorinterface
};


class CrowdSimulatorPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  CrowdSimulatorPlugin()
  : _transport_node_ptr(std::make_shared<gz::transport::Node>()),
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
  static constexpr double ANIMATION_DISTANCE_SWITCH_THRESHOLD = 0.01;
  std::shared_ptr<gz::transport::Node> _transport_node_ptr;
  bool _initialized;
  bool _enabled;
  std::string _resource_path;
  std::string _behavior_file;
  std::string _scene_file;
  float _sim_time_step;
  std::shared_ptr<MengeHandle> _menge_handle;
  ModelTypeDatabase _model_type_db_ptr;
  std::vector<std::string> _external_agents;
  std::vector<ObjectPtr> _objects; //Database, use id to access ObjectPtr
  std::vector<std::string> _idle_animation_names = {"idle", "stand"};
  std::chrono::steady_clock::duration _last_sim_time{0};

  std::shared_ptr<gz::sim::Model> _world;
  std::string _world_name;

  // map for <model_name, object_id>, contains both external (models) and internal agents (actors)
  std::unordered_map<std::string, size_t> _object_dic;
  // map for <model_name, entity_id> contains external and internal agents
  std::unordered_map<std::string, gz::sim::Entity> _entity_dic;

  bool read_sdf(const std::shared_ptr<const sdf::Element>& sdf);
  bool _spawn_agents_in_world();
  void _init_spawned_agents(gz::sim::EntityComponentManager& ecm);
  void _config_spawned_agents(
    const ObjectPtr obj_ptr,
    const gz::sim::Entity& enity,
    gz::sim::EntityComponentManager& ecm) const;
  bool _create_entity(
    const std::string& model_name,
    const ModelTypeDatabase::RecordPtr model_type_ptr) const;
  void _update_all_objects(
    double delta_sim_time,
    gz::sim::EntityComponentManager& ecm) const;
  void _update_internal_object(
    double delta_sim_time,
    const ObjectPtr obj_ptr,
    const gz::sim::Entity& enity,
    gz::sim::EntityComponentManager& ecm) const;
};

} //namespace crowd_simulation_gz
