#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Factory.hh>

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
          // TODO populate with actual properties
          components::DoorData data;
          data.v_max = 100;
          ecm.CreateComponent(entity, components::Door(data));
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
