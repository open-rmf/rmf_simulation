#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/JointAxis.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>

using namespace ignition::gazebo;

namespace rmf_building_sim_gz_plugins {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE RegisterComponentPlugin
  : public System,
  public ISystemConfigure
{
private:
  std::optional<DoorData> read_door_data(
    const Entity& entity,
    EntityComponentManager& ecm,
    const std::string& door_name,
    const std::shared_ptr<const sdf::Element>& sdf)
  {
    DoorData data;
    std::string right_joint_name;
    std::string left_joint_name;

    auto door_element = sdf->FindElement("door");

    if (door_element == nullptr ||
        !door_element->Get<std::string>("left_joint_name", left_joint_name, "") ||
        !door_element->Get<std::string>("right_joint_name", right_joint_name, ""))
    {
      ignerr << "Missing required parameters for plugin " << door_name << std::endl;
      return std::nullopt;
    }

    if ((left_joint_name == "empty_joint" &&
      right_joint_name == "empty_joint") ||
      (left_joint_name.empty() && right_joint_name.empty()))
    {
      ignerr << "Both door joint names are missing for " << door_name << " plugin, at least one"
        " is required" << std::endl;
      return std::nullopt;
    }

    std::unordered_set<std::string> joint_names;
    if (!left_joint_name.empty()
      && left_joint_name != "empty_joint")
      joint_names.insert(left_joint_name);
    if (!right_joint_name.empty()
      && right_joint_name != "empty_joint")
      joint_names.insert(right_joint_name);

    sdf->Get<double>("v_max_door", data.v_max, 0.2);
    sdf->Get<double>("a_max_door", data.a_max, 0.2);
    sdf->Get<double>("a_nom_door", data.a_nom, 0.08);
    sdf->Get<double>("dx_min_door", data.dx_min, 0.01);
    sdf->Get<double>("f_max_door", data.f_max, 100.0);
    sdf->Get<bool>("ros_interface", data.ros_interface, false);

    // TODO this logic in the actual door controller
    auto model = Model(entity);
    for (auto joint_name: joint_names)
    {
      auto joint_entity = model.JointByName(ecm, joint_name);
      if (joint_entity == kNullEntity)
      {
        continue;
      }
      const auto* joint_axis =
        ecm.Component<components::JointAxis>(joint_entity);

      double lower_limit = -1.57;
      double upper_limit = 0.0;
      if (joint_axis != nullptr)
      {
        lower_limit = joint_axis->Data().Lower();
        upper_limit = joint_axis->Data().Upper();
      }

      if (joint_name == right_joint_name)
      {
        // Right joint is flipped
        data.door_joints.push_back({joint_name, upper_limit, lower_limit});
      }
      else if (joint_name == left_joint_name)
      {
        data.door_joints.push_back({joint_name, lower_limit, upper_limit});
      }
      else
      {
        ignwarn << "Unsupported joint_name " << joint_name << " Ignoring..." << std::endl;
        continue;
      }

    }
    return data;
    
  }

public:
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    ignerr << "Entering register component plugin" << std::endl;

    if (sdf->HasElement("component"))
    {
      auto component_element = sdf->FindElement("component");
      while (component_element)
      {
        auto name = component_element->Get<std::string>("name");
        if (name == "Door")
        {
          if (auto data = read_door_data(entity, ecm, name, component_element))
            ecm.CreateComponent(entity, components::Door(*data));
        }
        ignwarn << "Registered component of name " << name << " for entity " << entity << std::endl;
        component_element = component_element->GetNextElement("component");
      }
    }
  }
};

IGNITION_ADD_PLUGIN(
  RegisterComponentPlugin,
  System,
  RegisterComponentPlugin::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(RegisterComponentPlugin, "register_component")

} // namespace rmf_building_sim_gz_plugins
