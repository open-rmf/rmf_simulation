#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/Name.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_building_sim_common/utils.hpp>
#include <rmf_building_sim_common/door_common.hpp>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>

using namespace ignition::gazebo;

using namespace rmf_building_sim_common;

namespace rmf_building_sim_gz_plugins {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE DoorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  static constexpr double STATE_PUB_DT = 1.0;
  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Publisher<DoorState>::SharedPtr _door_state_pub;
  rclcpp::Subscription<DoorRequest>::SharedPtr _door_request_sub;

  std::unordered_map<std::string, double> _last_state_pub;

  // Used to do open loop joint position control
  std::unordered_map<Entity, double> _last_cmd_vel;

  bool _first_iteration = true;

  bool is_joint_at_position(double joint_position, double dx_min, double target_position) const
  {
    return std::abs(target_position - joint_position) < dx_min;
  }

  DoorCommand get_current_mode(const Entity& entity, EntityComponentManager& ecm, const DoorData& door) const {
    bool all_open = true;
    bool all_closed = true;
    for (const auto& joint : door.joints)
    {
      auto joint_entity = Model(entity).JointByName(ecm, joint.name);
      if (joint_entity == kNullEntity)
        continue;
      const auto* joint_component =
        ecm.Component<components::JointPosition>(joint_entity);
      const double joint_position = joint_component->Data()[0];
      if (!is_joint_at_position(joint_position, door.params.dx_min, joint.open_position))
        all_open = false;
      if (!is_joint_at_position(joint_position, door.params.dx_min, joint.closed_position))
        all_closed = false;
    }
    if (all_open)
      return DoorCommand::OPEN;
    else if (all_closed)
      return DoorCommand::CLOSE;
    return DoorCommand::MOVING;
  }

  double calculate_target_velocity(
    const double target,
    const double current_position,
    const double current_velocity,
    const double dt,
    const MotionParams& params) const
  {
    double dx = target - current_position;
    if (std::abs(dx) < params.dx_min / 2.0)
      dx = 0.0;

    double door_v = compute_desired_rate_of_change(
      dx, current_velocity, params, dt);

    return door_v;
  }

  void command_door(const Entity& entity, EntityComponentManager& ecm, const DoorData& door, double dt, DoorCommand cmd)
  {
    auto model = Model(entity);
    for (const auto& joint : door.joints)
    {
      auto joint_entity = model.JointByName(ecm, joint.name);
      if (joint_entity != kNullEntity)
      {
        auto cur_pos = ecm.Component<components::JointPosition>(joint_entity)->Data()[0];
        auto target_pos = cmd == DoorCommand::OPEN ? joint.open_position : joint.closed_position;
        auto target_vel = calculate_target_velocity(target_pos, cur_pos, _last_cmd_vel[joint_entity], dt, door.params);
        ecm.CreateComponent<components::JointPositionReset>(joint_entity, components::JointPositionReset({cur_pos + target_vel * dt}));
        _last_cmd_vel[joint_entity] = target_vel;
      }
    }
  }

public:
  void Configure(const Entity& /*entity*/,
    const std::shared_ptr<const sdf::Element>& /*sdf*/,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    _ros_node = std::make_shared<rclcpp::Node>("rmf_simulation_door_manager");

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading DoorManager");

    // Subscribe to door requests, publish door states
    _door_state_pub = _ros_node->create_publisher<DoorState>(
      "door_states", rclcpp::SystemDefaultsQoS());

    _door_request_sub = _ros_node->create_subscription<DoorRequest>(
      "door_requests", rclcpp::SystemDefaultsQoS(),
      [&](DoorRequest::UniquePtr msg)
      {
        // Find entity with the name and create a DoorCmd component
        auto entity = ecm.EntityByComponents(components::Name(msg->door_name));
        if (entity != kNullEntity)
        {
          auto door_command = msg->requested_mode.value == msg->requested_mode.MODE_OPEN ?
            DoorCommand::OPEN : DoorCommand::CLOSE;
          ecm.CreateComponent<components::DoorCmd>(entity, components::DoorCmd(door_command));
        }
        else
        {
          ignwarn << "Request received for door " << msg->door_name <<
            " but it is not being simulated" << std::endl;
        }
      });
  }

  void initialize_components(EntityComponentManager& ecm)
  {
    ecm.Each<components::Door>([&](const Entity& entity, const components::Door* door) -> bool
        {
          for (auto joint : door->Data().joints)
          {
            auto joint_entity = Model(entity).JointByName(ecm, joint.name);
            enableComponent<components::JointPosition>(ecm, joint_entity);
          }
          return true;
        });
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    rclcpp::spin_some(_ros_node);
    if (_first_iteration)
    {
      _first_iteration = false;
      initialize_components(ecm);
      return;
    }

    // Don't update if the simulation is paused
    if (info.paused)
      return;

    // Process commands
    ecm.Each<components::Door, components::DoorCmd>([&](const Entity& entity, const components::Door* door_comp, const components::DoorCmd* door_cmd_comp) -> bool
        {
          double dt =
            (std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).
            count()) * 1e-9;
          const auto& door = door_comp->Data();
          const auto& door_cmd = door_cmd_comp->Data();
          // TODO(luca) consider reading when the state is equal to the
          // requested state and remove DoorCmd components when that is
          // the case to reduce number of times this loop is called
          //ignmsg << "Opening door" << std::endl;
          command_door(entity, ecm, door, dt, door_cmd);
          return true;
        });

    // Update states
    ecm.Each<components::Door>([&](const Entity& entity, const components::Door* door_comp) -> bool
        {
          const auto& door = door_comp->Data();
          const auto cur_mode = get_current_mode(entity, ecm, door);
          ecm.CreateComponent<components::DoorStateComp>(entity, components::DoorStateComp(cur_mode));
          return true;
        });

    // Publish states
    ecm.Each<components::Door, components::DoorStateComp, components::Name>([&](const Entity& entity, const components::Door* door_comp, const components::DoorStateComp* door_state_comp, const components::Name* name_comp) -> bool
        {
          const auto& door_state = door_state_comp->Data();
          const auto& door = door_comp->Data();
          if (door.ros_interface == false)
            return true;
          double t =
            (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
            count()) * 1e-9;
          const auto& name = name_comp->Data();
          if (_last_state_pub.find(name) == _last_state_pub.end())
            _last_state_pub[name] = static_cast<double>(std::rand()) / RAND_MAX;
          if (t - _last_state_pub[name] >= STATE_PUB_DT)
          {
            DoorState msg;
            msg.door_name = name;
            msg.door_time.sec = t;
            msg.door_time.nanosec = (t - static_cast<int>(t)) * 1e9;
            switch (door_state) {
              case DoorCommand::OPEN: {
                msg.current_mode.value = msg.current_mode.MODE_OPEN;
                break;
              }
              case DoorCommand::MOVING: {
                msg.current_mode.value = msg.current_mode.MODE_MOVING;
                break;
              }
              case DoorCommand::CLOSE: {
                msg.current_mode.value = msg.current_mode.MODE_CLOSED;
                break;
              }
            }
            //msg.current_mode = get_current_mode(entity, ecm, door);
            _door_state_pub->publish(msg);
            _last_state_pub[name] = t;
          }
          return true;
        });
  }
};

IGNITION_ADD_PLUGIN(
  DoorPlugin,
  System,
  DoorPlugin::ISystemConfigure,
  DoorPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DoorPlugin, "door")

} // namespace rmf_building_sim_gz_plugins
