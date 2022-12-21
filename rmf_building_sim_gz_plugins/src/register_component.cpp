#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/JointAxis.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/gazebo/components/AxisAlignedBox.hh>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>
#include <rmf_building_sim_gz_plugins/components/Lift.hpp>

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

    sdf->Get<double>("v_max_door", data.params.v_max, 0.2);
    sdf->Get<double>("a_max_door", data.params.a_max, 0.2);
    sdf->Get<double>("a_nom_door", data.params.a_nom, 0.08);
    sdf->Get<double>("dx_min_door", data.params.dx_min, 0.01);
    sdf->Get<double>("f_max_door", data.params.f_max, 100.0);
    sdf->Get<bool>("ros_interface", data.ros_interface, false);

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
        data.joints.push_back({joint_name, lower_limit, upper_limit});
      }
      else if (joint_name == left_joint_name)
      {
        // Left joint is flipped
        data.joints.push_back({joint_name, upper_limit, lower_limit});
      }
      else
      {
        ignwarn << "Unsupported joint_name " << joint_name << " Ignoring..." << std::endl;
        continue;
      }

    }
    return data;
  }

  std::optional<LiftData> read_lift_data(
    const std::string& lift_name,
    const std::shared_ptr<const sdf::Element>& sdf)
  {
    LiftData data;

    // load lift cabin motion parameters
    sdf->Get<double>("v_max_cabin", data.params.v_max, 0.2);
    sdf->Get<double>("a_max_cabin", data.params.a_max, 0.2);
    sdf->Get<double>("a_nom_cabin", data.params.a_nom, 0.08);
    sdf->Get<double>("dx_min_cabin", data.params.dx_min, 0.01);
    sdf->Get<double>("f_max_cabin", data.params.f_max, 100.0);

    if (!sdf->Get<std::string>("cabin_joint_name", data.cabin_joint, ""))
      return std::nullopt;

    // load the floor name and elevation for each floor
    auto floor_element = sdf->FindElement("floor");
    if (!floor_element)
    {
      ignerr << "Missing required floor element for [" << lift_name << "] plugin" << std::endl;
      return std::nullopt;
    }

    std::optional<std::string> first_found_floor;
    while (floor_element)
    {
      Floor floor;
      std::string floor_name;
      if (!floor_element->Get<std::string>("name", floor_name, "") ||
          !floor_element->Get<double>("elevation", floor.elevation, 0.0))
      {
        ignerr << "Missing required floor name or elevation attributes for [" << lift_name << "] plugin" << std::endl;
        return std::nullopt;
      }

      if (!first_found_floor.has_value())
        first_found_floor = floor_name;

      auto door_pair_element = floor_element->FindElement("door_pair");
      while (door_pair_element)
      {
        FloorDoorPair doors;
        if (!door_pair_element->Get<std::string>("cabin_door", doors.cabin_door, "") ||
            !door_pair_element->Get<std::string>("shaft_door", doors.shaft_door, ""))
        {
          ignerr << "Missing required lift door attributes for [" << lift_name << "] plugin" << std::endl;
          return std::nullopt;
        }
        floor.doors.push_back(doors);

        door_pair_element = door_pair_element->GetNextElement("door_pair");
      }

      data.floors.insert({floor_name, floor});
      floor_element = floor_element->GetNextElement("floor");
    }

    if (!first_found_floor.has_value())
    {
      ignerr << "No floors enabled for [" << lift_name << "] plugin" << std::endl;
      return std::nullopt;
    }

    sdf->Get<std::string>("initial_floor", data.initial_floor, "");

    if (data.floors.find(data.initial_floor) == data.floors.end())
    {
      ignerr << "Initial floor [" << data.initial_floor << "] not available, changing to default" << std::endl;
      data.initial_floor = *first_found_floor;
    }

    return data;
  }


public:
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
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
        else if (name == "Lift")
        {
          if (auto data = read_lift_data(name, component_element))
          {
            ignerr << "Found lift plugin data" << std::endl;
            ecm.CreateComponent(entity, components::Lift(*data));
            // Bounding boxes are needed to move payloads, enable here to
            // simplify logic at the lift plugin level
            enableComponent<components::AxisAlignedBox>(ecm, entity);
          }
        }
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
