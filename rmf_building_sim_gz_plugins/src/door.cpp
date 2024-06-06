#include <gz/plugin/Register.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>
#include <rmf_building_sim_gz_plugins/utils.hpp>

using namespace gz::sim;

using namespace rmf_building_sim_gz_plugins;
using DoorMode = rmf_door_msgs::msg::DoorMode;
using DoorState = rmf_door_msgs::msg::DoorState;
using DoorRequest = rmf_door_msgs::msg::DoorRequest;

class DoorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  // TODO(luca) make this a parameter of the door manager
  static constexpr double PUBLISH_DT = 1.0;
  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Publisher<DoorState>::SharedPtr _door_state_pub;
  rclcpp::Subscription<DoorRequest>::SharedPtr _door_request_sub;

  // Used to do open loop joint position control
  std::unordered_map<Entity, double> _last_cmd_vel;

  // Saves the last timestamp a door state was sent
  std::unordered_map<Entity, double> _last_state_pub;

  bool _first_iteration = true;

  Entity get_joint_entity(const EntityComponentManager& ecm,
    const Entity& model_entity,
    const std::string& joint_name) const
  {
    auto joint_entity = Model(model_entity).JointByName(ecm, joint_name);
    if (joint_entity == kNullEntity)
    {
      // Try for its parent (i.e. for lift nested cabin doors)
      joint_entity = Model(ecm.ParentEntity(model_entity)).JointByName(ecm,
          joint_name);
      if (joint_entity == kNullEntity)
      {
        gzwarn << "Joint " << joint_name << " not found" << std::endl;
      }
    }
    return joint_entity;
  }

  bool is_joint_at_position(double joint_position, double dx_min,
    double target_position) const
  {
    return std::abs(target_position - joint_position) < dx_min;
  }

  DoorModeCmp get_current_mode(const Entity& entity,
    EntityComponentManager& ecm,
    const DoorData& door) const
  {
    bool all_open = true;
    bool all_closed = true;
    for (const auto& joint : door.joints)
    {
      auto joint_entity = get_joint_entity(ecm, entity, joint.name);
      if (joint_entity == kNullEntity)
      {
        continue;
      }
      const auto* joint_component =
        ecm.Component<components::JointPosition>(joint_entity);
      const double joint_position = joint_component->Data()[0];
      if (!is_joint_at_position(joint_position, door.params.dx_min,
        joint.open_position))
      {
        all_open = false;
      }
      if (!is_joint_at_position(joint_position, door.params.dx_min,
        joint.closed_position))
      {
        all_closed = false;
      }
    }
    if (all_open)
      return DoorModeCmp::OPEN;
    else if (all_closed)
      return DoorModeCmp::CLOSE;
    return DoorModeCmp::MOVING;
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

  void command_door(const Entity& entity, EntityComponentManager& ecm,
    const DoorData& door, double dt, DoorModeCmp cmd)
  {
    auto model = Model(entity);
    for (const auto& joint : door.joints)
    {
      auto joint_entity = get_joint_entity(ecm, entity, joint.name);
      if (joint_entity != kNullEntity)
      {
        auto cur_pos =
          ecm.Component<components::JointPosition>(joint_entity)->Data()[0];
        auto target_pos = cmd ==
          DoorModeCmp::OPEN ? joint.open_position : joint.closed_position;
        auto target_vel = calculate_target_velocity(target_pos, cur_pos,
            _last_cmd_vel[joint_entity],
            dt, door.params);
        ecm.CreateComponent<components::JointPositionReset>(joint_entity,
          components::JointPositionReset(
            {cur_pos + target_vel * dt}));
        _last_cmd_vel[joint_entity] = target_vel;
      }
    }
  }

  void publish_state(const double t, const std::string& name,
    const DoorModeCmp& door_state)
  {
    DoorState msg;
    msg.door_name = name;
    msg.door_time.sec = t;
    msg.door_time.nanosec = (t - static_cast<int>(t)) * 1e9;
    switch (door_state)
    {
      case DoorModeCmp::OPEN: {
        msg.current_mode.value = msg.current_mode.MODE_OPEN;
        break;
      }
      case DoorModeCmp::MOVING: {
        msg.current_mode.value = msg.current_mode.MODE_MOVING;
        break;
      }
      case DoorModeCmp::CLOSE: {
        msg.current_mode.value = msg.current_mode.MODE_CLOSED;
        break;
      }
    }
    _door_state_pub->publish(msg);
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
    const auto pub_qos = rclcpp::QoS(100).reliable();
    _door_state_pub = _ros_node->create_publisher<DoorState>(
      "door_states", pub_qos);

    _door_request_sub = _ros_node->create_subscription<DoorRequest>(
      "door_requests", rclcpp::SystemDefaultsQoS(),
      [ecm = &ecm](DoorRequest::UniquePtr msg)
      {
        // Find entity with the name and create a DoorCmd component
        // TODO(luca) cache this to avoid expensive iteration over all entities?
        auto entity = ecm->EntityByComponents(components::Name(
          msg->door_name));
        const auto* door = ecm->Component<components::Door>(entity);
        if (entity != kNullEntity && door != nullptr)
        {
          if (door->Data().ros_interface == false)
          {
            gzmsg << "Ignoring door " << msg->door_name <<
              " because it doesn't have a ros interface" << std::endl;
            return;
          }
          auto door_command = msg->requested_mode.value ==
          msg->requested_mode.MODE_OPEN ?
          DoorModeCmp::OPEN : DoorModeCmp::CLOSE;
          ecm->CreateComponent<components::DoorCmd>(entity,
          components::DoorCmd(door_command));
        }
        else
        {
          gzwarn << "Request received for door " << msg->door_name <<
            " but it is not being simulated" << std::endl;
        }
      });
  }

  void initialize_components(EntityComponentManager& ecm)
  {
    ecm.Each<components::Door>([&](const Entity& entity,
      const components::Door* door) -> bool
      {
        for (auto joint : door->Data().joints)
        {
          auto joint_entity = get_joint_entity(ecm, entity, joint.name);
          std::vector<double> position = {0.0};
          ecm.CreateComponent<components::JointPosition>(joint_entity,
          components::JointPosition(position));
        }
        enableComponent<components::DoorStateComp>(ecm, entity);
        ecm.CreateComponent<components::DoorCmd>(entity,
        components::DoorCmd(DoorModeCmp::CLOSE));
        return true;
      });
  }

  void initialize_pub_times(EntityComponentManager& ecm)
  {
    ecm.Each<components::Door>([&](const Entity& e,
      const components::Door* door_comp) -> bool
      {
        if (door_comp->Data().ros_interface == false)
          return true;
        _last_state_pub[e] = ((double) std::rand()) /
        ((double) RAND_MAX/PUBLISH_DT);
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
      initialize_pub_times(ecm);
      return;
    }

    // Don't update if the simulation is paused
    if (info.paused)
      return;

    const double t = to_seconds(info.simTime);
    // Process commands
    ecm.Each<components::Door, components::DoorCmd,
      components::DoorStateComp, components::Name>([&](const Entity& entity,
      const components::Door* door_comp,
      const components::DoorCmd* door_cmd_comp,
      components::DoorStateComp* door_state_comp,
      const components::Name* name_comp) -> bool
      {
        double dt = to_seconds(info.dt);
        const auto& name = name_comp->Data();
        const auto& door = door_comp->Data();
        const auto& door_cmd = door_cmd_comp->Data();
        command_door(entity, ecm, door, dt, door_cmd);
        // Publish state if we are past the deadline
        const auto cur_mode = get_current_mode(entity, ecm, door);
        if (door_comp->Data().ros_interface)
        {
          auto it = _last_state_pub.find(entity);
          if (it != _last_state_pub.end() && t - it->second >= PUBLISH_DT)
          {
            it->second = t;
            publish_state(t, name, cur_mode);
          }
        }
        door_state_comp->Data() = cur_mode;
        return true;
      });
  }
};

GZ_ADD_PLUGIN(
  DoorPlugin,
  System,
  DoorPlugin::ISystemConfigure,
  DoorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DoorPlugin, "door")
