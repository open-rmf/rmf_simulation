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

#include <gz/math/Pose3.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>

#include "crowd_simulator.hpp"

using namespace gz::sim;

namespace crowd_simulation_gz {

static bool _load_model_init_pose(
  const std::shared_ptr<const sdf::Element>& model_type_element,
  AgentPose3d& result)
{
  std::string pose_str;
  if (model_type_element->template Get<std::string>(
      "initial_pose", pose_str, ""))
  {
    std::regex ws_re("\\s+"); //whitespace
    std::vector<std::string> parts(
      std::sregex_token_iterator(pose_str.begin(), pose_str.end(), ws_re, -1),
      std::sregex_token_iterator());

    if (parts.size() != 6)
    {
      gzerr <<
        "Error loading <initial_pose> in <model_type>, 6 floats (x, y, z, pitch, roll, yaw) expected."
            << std::endl;
      return false;
    }

    result.x(std::stod(parts[0]) );
    result.y(std::stod(parts[1]) );
    result.z(std::stod(parts[2]) );
    result.pitch(std::stod(parts[3]) );
    result.roll(std::stod(parts[4]) );
    result.yaw(std::stod(parts[5]) );
  }
  return true;
}

static gz::math::Pose3d get_agent_pose(
  const AgentPtr agent_ptr, double delta_sim_time)
{
  //calculate future position in delta_sim_time. currently in 2d
  double px = static_cast<double>(agent_ptr->_pos.x()) +
    static_cast<double>(agent_ptr->_vel.x()) * delta_sim_time;
  double py = static_cast<double>(agent_ptr->_pos.y()) +
    static_cast<double>(agent_ptr->_vel.y()) * delta_sim_time;

  double x_rot = static_cast<double>(agent_ptr->_orient.x());
  double y_rot = static_cast<double>(agent_ptr->_orient.y());

  return gz::math::Pose3d(px, py, 0, 0, 0, std::atan2(y_rot, x_rot));
}

//=================================================
bool CrowdSimulatorPlugin::read_sdf(
  const std::shared_ptr<const sdf::Element>& sdf)
{
  char* menge_resource_path = getenv("MENGE_RESOURCE_PATH");

  if (menge_resource_path == nullptr ||
    strcmp(menge_resource_path, "") == 0)
  {
    gzwarn <<
      "MENGE_RESOURCE_PATH env is empty. Crowd simulation is disabled." <<
      std::endl;
    return true;
  }

  _enabled = true;
  _resource_path = std::string(menge_resource_path);
  gzmsg << "Crowd Sim is enabled! <env MENGE_RESOURCE_PATH> is : " <<
    _resource_path << std::endl;

  if (!sdf->HasElement("behavior_file"))
  {
    gzerr << "No behavior file found! <behavior_file> Required!" << std::endl;
    return false;
  }
  _behavior_file =
    sdf->GetElementImpl("behavior_file")->template Get<std::string>();

  if (!sdf->HasElement("scene_file"))
  {
    gzerr << "No scene file found! <scene_file> Required!" << std::endl;
    return false;
  }
  _scene_file =
    sdf->GetElementImpl("scene_file")->template Get<std::string>();

  if (!sdf->HasElement("update_time_step"))
  {
    gzerr << "No update_time_step found! <update_time_step> Required!" <<
      std::endl;
    return false;
  }
  _sim_time_step =
    sdf->GetElementImpl("update_time_step")->template Get<float>();

  if (!sdf->HasElement("model_type"))
  {
    gzerr << "No model type for agents found! <model_type> element Required!" <<
      std::endl;
    return false;
  }
  auto model_type_element = sdf->GetElementImpl("model_type");
  while (model_type_element)
  {
    std::string s;
    if (!model_type_element->template Get<std::string>("typename", s, ""))
    {
      gzerr <<
        "No model type name configured in <model_type>! <typename> Required"
            << std::endl;
      return false;
    }

    auto model_type_ptr = _model_type_db_ptr.emplace(s,
        std::make_shared<ModelTypeDatabase::Record>() ); //unordered_map
    model_type_ptr->type_name = s;

    if (!model_type_element->template Get<std::string>("filename",
      model_type_ptr->file_name, ""))
    {
      gzerr <<
        "No actor skin configured in <model_type>! <filename> Required" <<
        std::endl;
      return false;
    }

    if (!model_type_element->template Get<std::string>("animation",
      model_type_ptr->animation, ""))
    {
      gzerr <<
        "No animation configured in <model_type>! <animation> Required" <<
        std::endl;
      return false;
    }

    if (!model_type_element->template Get<double>("animation_speed",
      model_type_ptr->animation_speed, 0.0))
    {
      gzerr <<
        "No animation speed configured in <model_type>! <animation_speed> Required"
            << std::endl;
      return false;
    }

    if (!model_type_element->HasElement("initial_pose"))
    {
      gzerr <<
        "No model initial pose configured in <model_type>! <initial_pose> Required ["
            << s << "]" << std::endl;
      return false;
    }
    if (!_load_model_init_pose(model_type_element, model_type_ptr->pose))
    {
      gzerr <<
        "Error loading model initial pose in <model_type>! Check <initial_pose> in ["
            << s << "]" << std::endl;
      return false;
    }

    model_type_element = model_type_element->GetNextElement(
      "model_type");
  }

  if (!sdf->HasElement("external_agent"))
  {
    gzwarn <<
      "No external agent provided. <external_agent> is needed with a unique name defined above."
           << std::endl;
  }
  auto external_agent_element = sdf->GetElementImpl("external_agent");
  while (external_agent_element)
  {
    auto ex_agent_name = external_agent_element->template Get<std::string>();
    gzmsg << "Added external agent: [" << ex_agent_name << "]." << std::endl;
    _external_agents.emplace_back(ex_agent_name); //just store the name
    external_agent_element = external_agent_element->GetNextElement(
      "external_agent");
  }

  return true;
}

void CrowdSimulatorPlugin::Configure(
  const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm,
  EventManager& /*event_mgr*/)
{
  _world = std::make_shared<Model>(entity);
  gzmsg << "Initializing world plugin with name: " <<
    _world->Name(ecm).c_str() << std::endl;
  _world_name = _world->Name(ecm);

  if (!read_sdf(sdf))
  {
    gzerr << "Error loading crowd simulator plugin. Load params failed!" <<
      std::endl;
    exit(EXIT_FAILURE);
  }

  if (!_enabled)
  {
    gzmsg << "CrowdSim is Disabled!" << std::endl;
    return;
  }

  _menge_handle = MengeHandle::init_and_make(
    _resource_path,
    _behavior_file,
    _scene_file,
    _sim_time_step);

  //_spawn_object();

  //bool CrowdSimInterface::_spawn_object()
  {
    //External models are loaded first in scene file
    size_t external_count = _external_agents.size();
    size_t total_agent_count = _menge_handle->get_agent_count();

    for (size_t i = 0; i < external_count; ++i)
    {
      auto agent_ptr = _menge_handle->get_agent(i);
      agent_ptr->_external = true;
      _objects.emplace_back(
        new Object{agent_ptr, _external_agents[i], agent_ptr->_typeName, true,
          AnimState::WALK});
    }

    for (size_t i = external_count; i < total_agent_count; ++i)
    {
      auto agent_ptr = _menge_handle->get_agent(i);
      agent_ptr->_external = false;
      std::string model_name = "agent" + std::to_string(i);
      _objects.emplace_back(
        new Object{agent_ptr, model_name, agent_ptr->_typeName, false,
          AnimState::WALK});
    }
  }

  if (!_spawn_agents_in_world())
  {
    gzerr <<
      "Error loading crowd simulator plugin. Crowd Simulator failed to spawn agents in the world!"
          << std::endl;
    exit(EXIT_FAILURE);
  }

}

//=================================================
void CrowdSimulatorPlugin::PreUpdate(
  const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  // check if crowd sim is enabled
  if (!_enabled)
    return;

  // wait for all the models and actors loaded in gz rendering
  if (!_initialized)
  {
    _init_spawned_agents(ecm);
    return;
  }

  // Don't update the plugin if the simulation is paused
  if (info.paused)
    return;

  // Note, the update_time_step parameter is ignored in gz
  // through GPU animated actors the performance is good enough that
  // we can afford to update at every iteration and have smooth animations
  std::chrono::duration<double> delta_sim_time_tmp = info.simTime -
    _last_sim_time;
  double delta_sim_time = delta_sim_time_tmp.count();
  _last_sim_time = info.simTime;
  _menge_handle->set_sim_time_step(delta_sim_time);
  _menge_handle->sim_step();
  _update_all_objects(delta_sim_time, ecm);
}

//==========================================================
bool CrowdSimulatorPlugin::_spawn_agents_in_world()
{
  for (size_t id = 0; id < _objects.size(); ++id)
  {
    auto object_ptr = _objects[id];
    assert(object_ptr);
    _object_dic[object_ptr->model_name] = id;

    if (!object_ptr->is_external)
    {
      auto type_ptr = _model_type_db_ptr.get(object_ptr->type_name);
      assert(type_ptr);
      if (!this->_create_entity(object_ptr->model_name, type_ptr) )
      {
        gzerr << "Failed to insert model [ " << object_ptr->model_name <<
          " ] in world" << std::endl;
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
  for (size_t id = 0; id < _objects.size(); id++)
  {
    auto obj = _objects[id];
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
        auto obj_ptr = _objects[it_objects_name->second];
        // config internal spawned agent for custom trajectory
        if (!obj_ptr->is_external)
        {
          _config_spawned_agents(obj_ptr, entity, ecm);
        }
        objects_name.erase(name->Data());
        gzmsg << "Crowd Simulator found agent: " << name->Data() << std::endl;
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
        auto obj_ptr = _objects[it_objects_name->second];
        // config internal spawned agent for custom trajectory
        if (!obj_ptr->is_external)
        {
          _config_spawned_agents(obj_ptr, entity, ecm);
        }
        objects_name.erase(name->Data());
        gzmsg << "Crowd Simulator found agent: " << name->Data() << std::endl;
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
  gzmsg << "Gazebo Models are all loaded! Start simulating..." << std::endl;
}

//===================================================================
bool CrowdSimulatorPlugin::_create_entity(
  const std::string& model_name,
  const ModelTypeDatabase::RecordPtr model_type_ptr) const
{
  // Use gz create service to spawn actors
  // calling gz gazebo create service, you can use "ign service -l" to
  // check the service available
  assert(model_type_ptr);
  std::string service = "/world/" + this->_world_name + "/create";
  gz::msgs::EntityFactory request;
  request.set_sdf_filename(model_type_ptr->file_name);
  request.set_name(model_name);
  gz::math::Pose3d pose(0, 0, 0, 0, 0, 0);

  gz::msgs::Boolean response;
  bool result;
  uint32_t timeout = 5000;
  bool executed = this->_transport_node_ptr->Request(service, request, timeout,
      response, result);
  if (executed)
  {
    if (result && response.data())
    {
      gzmsg << "Requested creation of entity: " << model_name << std::endl;
      return true;
    }
    else
    {
      gzerr << "Failed request to create entity:" << std::endl <<
        request.DebugString() << std::endl;
    }
  }
  else
  {
    gzerr << "Request to create entity from service " <<
      request.DebugString() << " time out ..." << std::endl;
  }
  return false;
}

//==================================================
void CrowdSimulatorPlugin::_config_spawned_agents(
  const ObjectPtr obj_ptr,
  const Entity& entity,
  EntityComponentManager& ecm) const
{
  assert(obj_ptr);
  auto agent_ptr = obj_ptr->agent_ptr;
  auto model_type = _model_type_db_ptr.get(obj_ptr->type_name);
  // different from gazebo plugin, the pose component is the origin of the trajPose
  gz::math::Pose3d actor_pose(
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
  for (const auto& idle_anim : _idle_animation_names)
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
  for (size_t id = 0; id < _objects.size(); id++)
  {
    auto obj_ptr = _objects[id];
    auto it_entity = _entity_dic.find(obj_ptr->model_name);
    if (it_entity == _entity_dic.end())   //safe check
    {
      gzerr << "Didn't initialize external agent [ " << obj_ptr->model_name <<
        " ]" << std::endl;
      exit(EXIT_FAILURE);
    }
    auto entity = it_entity->second;

    // for external agent
    if (obj_ptr->is_external)
    {
      auto model_pose =
        ecm.Component<components::Pose>(entity)->Data();
      obj_ptr->agent_ptr->_pos.setX(model_pose.Pos().X());
      obj_ptr->agent_ptr->_pos.setY(model_pose.Pos().Y());
      continue;
    }

    // for internal agent
    _update_internal_object(delta_sim_time, obj_ptr, entity, ecm);
  }
}

void CrowdSimulatorPlugin::_update_internal_object(
  double delta_sim_time,
  const ObjectPtr obj_ptr,
  const Entity& entity,
  EntityComponentManager& ecm) const
{
  double animation_speed =
    _model_type_db_ptr.get(obj_ptr->type_name)->animation_speed;
  gz::math::Pose3d initial_pose =
    _model_type_db_ptr.get(obj_ptr->type_name)->pose.
    convert_to_ign_math_pose_3d<gz::math::Pose3d>();
  gz::math::Pose3d agent_pose = get_agent_pose(obj_ptr->agent_ptr,
      delta_sim_time);
  agent_pose += initial_pose;

  // get components to be updated
  auto traj_pose_comp =
    ecm.Component<components::TrajectoryPose>(entity);
  auto anim_name_comp =
    ecm.Component<components::AnimationName>(entity);
  auto anim_time_comp =
    ecm.Component<components::AnimationTime>(entity);
  gz::math::Pose3d current_pose = traj_pose_comp->Data();
  auto distance_traveled_vector = agent_pose.Pos() - current_pose.Pos();
  // might need future work on 3D case
  // the center of human has a z_elevation, which will make the human keep walking even if he reached the target
  distance_traveled_vector.Z(0.0);
  double distance_traveled = distance_traveled_vector.Length();

  // switch animation
  auto model_type = _model_type_db_ptr.get(obj_ptr->type_name);
  AnimState next_state = obj_ptr->get_next_state(
    distance_traveled < ANIMATION_DISTANCE_SWITCH_THRESHOLD &&
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

//============================================
ModelTypeDatabase::RecordPtr ModelTypeDatabase::emplace(
  std::string type_name,
  RecordPtr record_ptr)
{
  auto pair = this->_records.emplace(type_name, record_ptr); //return pair<iterator, bool>
  assert(pair.second);
  return pair.first->second;
}

ModelTypeDatabase::RecordPtr ModelTypeDatabase::get(
  const std::string& type_name)
const
{
  auto it = this->_records.find(type_name);
  if (it == this->_records.end())
  {
    std::cout << "The model type [ " << type_name <<
      " ] is not defined in scene file!" << std::endl;
    return nullptr;
  }
  return it->second;
}

size_t ModelTypeDatabase::size() const
{
  return this->_records.size();
}

//================================================================
std::shared_ptr<MengeHandle> MengeHandle::init_and_make(
  const std::string& resource_path,
  const std::string& behavior_file,
  const std::string& scene_file,
  const float sim_time_step
)
{
  auto menge_handle = std::make_shared<MengeHandle>(
    resource_path, behavior_file, scene_file, sim_time_step);
  if (!menge_handle->_load_simulation())
  {
    return nullptr;
  }
  return menge_handle;
}

void MengeHandle::set_sim_time_step(float sim_time_step)
{
  this->_sim_time_step = sim_time_step;
  // Set it if a valid handle is present
  if (this->_sim)
  {
    this->_sim->setTimeStep(sim_time_step);
  }
}

float MengeHandle::get_sim_time_step() const
{
  return this->_sim_time_step;
}

size_t MengeHandle::get_agent_count()
{
  if (this->_agent_count == 0)
  {
    this->_agent_count = this->_sim->getNumAgents();
  }
  return this->_agent_count;
}

void MengeHandle::sim_step() const
{
  this->_sim->step();
}

AgentPtr MengeHandle::get_agent(size_t id) const
{
  return AgentPtr(this->_sim->getAgent(id));
}

std::string MengeHandle::_resource_file_path(const std::string& relative_path)
const
{
  std::string full_path = this->_resource_path + "/" + relative_path;
  std::cout << "Finding resource file: " << full_path << std::endl;
  std::ifstream ifile(full_path);
  if (!static_cast<bool>(ifile))
  {
    std::cerr << "File not found! " << full_path << std::endl;
    assert(static_cast<bool>(ifile));
  }
  std::cout << "Found." << std::endl;
  return full_path;
}

bool MengeHandle::_load_simulation()
{
  Menge::SimulatorDB sim_db;
  Menge::PluginEngine::CorePluginEngine engine(&sim_db);

  std::cout << "Start CrowdSimulator initializing [Menge]..." << std::endl;

  this->_sim = std::shared_ptr<Menge::Agents::SimulatorInterface>(
    sim_db.getDBEntry("orca")->getSimulator(
      this->_agent_count,
      this->_sim_time_step,
      0,
      std::numeric_limits<float>::infinity(),
      this->_behavior_file,
      this->_scene_file,
      "",
      "",
      false)
  );

  if (this->_sim)
  {
    std::cout << std::endl << "Crowd Simulator initialized success [Menge]. " <<
      std::endl;
    return true;
  }
  std::cout <<
    "Error in provided navmesh. Menge simulator initialized false." <<
    std::endl;
  return false;
}

//=============================================
AnimState Object::get_next_state(
  bool condition)
{
  if (condition)
    return AnimState::IDLE;
  else
    return AnimState::WALK;

  return current_state;
}

GZ_ADD_PLUGIN(
  CrowdSimulatorPlugin,
  System,
  CrowdSimulatorPlugin::ISystemConfigure,
  CrowdSimulatorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(CrowdSimulatorPlugin,
  "crowd_simulation")

} //namespace crowd_simulation_gz
