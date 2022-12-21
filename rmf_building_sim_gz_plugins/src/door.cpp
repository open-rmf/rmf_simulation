#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
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

  bool _first_iteration = true;

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    enableComponent<components::JointPosition>(ecm, entity);
    enableComponent<components::JointVelocity>(ecm, entity);
    ecm.CreateComponent<components::JointVelocityCmd>(entity, components::JointVelocityCmd({0.0}));
  }

  bool is_joint_at_position(double joint_position, double dx_min, double target_position) const
  {
    return std::abs(target_position - joint_position) < dx_min;
  }

  DoorMode get_current_mode(const Entity& entity, EntityComponentManager& ecm, const DoorData& door) const {
    DoorMode mode;
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
      mode.value = mode.MODE_OPEN;
    else if (all_closed)
      mode.value = mode.MODE_CLOSED;
    else
      mode.value = mode.MODE_MOVING;

    return mode;
  }

  double calculate_target_velocity(
    const double target,
    const double current_position,
    const double current_velocity,
    const double dt,
    const MotionParams& params)
  {
    double dx = target - current_position;
    if (std::abs(dx) < params.dx_min / 2.0)
      dx = 0.0;

    double door_v = compute_desired_rate_of_change(
      dx, current_velocity, params, dt);

    return door_v;
  }

  void close_door(const Entity& entity, EntityComponentManager& ecm, const DoorData& door, double dt) {
    auto model = Model(entity);

    for (const auto& joint : door.joints)
    {
      auto joint_entity = model.JointByName(ecm, joint.name);
      if (joint_entity != kNullEntity)
      {
        auto cur_pos = ecm.Component<components::JointPosition>(joint_entity)->Data();
        auto cur_vel = ecm.Component<components::JointVelocity>(joint_entity)->Data();
        auto target_vel = calculate_target_velocity(joint.closed_position, cur_pos[0], cur_vel[0], dt, door.params);
        ecm.CreateComponent<components::JointVelocityCmd>(joint_entity, components::JointVelocityCmd({target_vel}));
      }
    }
  }

  void open_door(const Entity& entity, EntityComponentManager& ecm, const DoorData& door, double dt) {
    auto model = Model(entity);

    for (const auto& joint : door.joints)
    {
      auto joint_entity = model.JointByName(ecm, joint.name);
      if (joint_entity != kNullEntity)
      {
        auto cur_pos = ecm.Component<components::JointPosition>(joint_entity)->Data();
        auto cur_vel = ecm.Component<components::JointVelocity>(joint_entity)->Data();
        auto target_vel = calculate_target_velocity(joint.open_position, cur_pos[0], cur_vel[0], dt, door.params);
        ecm.CreateComponent<components::JointVelocityCmd>(joint_entity, components::JointVelocityCmd({target_vel}));
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
            create_entity_components(joint_entity, ecm);
          }
          return true;
        });
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    rclcpp::spin_some(_ros_node);
    // JointPosition and JointVelocity components are populated by Physics
    // system in Update, hence they are uninitialized in the first PreUpdate.
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
          if (door_cmd_comp->Data() == DoorCommand::CLOSE)
          {
            close_door(entity, ecm, door, dt);
          }
          else if (door_cmd_comp->Data() == DoorCommand::OPEN)
          {
            open_door(entity, ecm, door, dt);
          }
          return true;
        });

    // Publish states
    ecm.Each<components::Door, components::Name>([&](const Entity& entity, const components::Door* door_comp, const components::Name* name_comp) -> bool
        {
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
            msg.current_mode = get_current_mode(entity, ecm, door);
            _door_state_pub->publish(msg);
            _last_state_pub[name] = t;
          }
          return true;
        });

    return;
  }

};

IGNITION_ADD_PLUGIN(
  DoorPlugin,
  System,
  DoorPlugin::ISystemConfigure,
  DoorPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DoorPlugin, "door")

} // namespace rmf_building_sim_gz_plugins
