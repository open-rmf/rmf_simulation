#include <gz/plugin/Register.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/PhysicsEnginePlugin.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>
#include <rmf_building_sim_gz_plugins/components/Lift.hpp>
#include <rmf_building_sim_gz_plugins/utils.hpp>

#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

using namespace gz::sim;

using DoorMode = rmf_door_msgs::msg::DoorMode;
using LiftState = rmf_lift_msgs::msg::LiftState;
using LiftRequest = rmf_lift_msgs::msg::LiftRequest;

namespace rmf_building_sim_gz_plugins {

//==============================================================================

class LiftPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  // TODO(luca) make this a parameter of the lift manager
  static constexpr double PUBLISH_DT = 1.0;
  rclcpp::Node::SharedPtr _ros_node;

  rclcpp::Publisher<LiftState>::SharedPtr _lift_state_pub;
  rclcpp::Subscription<LiftRequest>::SharedPtr _lift_request_sub;

  std::unordered_map<Entity, gz::math::AxisAlignedBox> _initial_aabbs;
  std::unordered_map<Entity, gz::math::Pose3d> _initial_poses;
  mutable std::unordered_map<std::string, Entity> _cached_entity_by_names;

  std::unordered_map<Entity, double> _last_cmd_vel;
  std::unordered_map<Entity, LiftCommand> _last_lift_command;
  std::unordered_map<Entity, LiftState> _last_states;

  // Saves the last timestamp a door state was sent
  std::unordered_map<Entity, double> _last_state_pub;

  bool _components_initialized = false;
  bool _aabb_read = false;

  std::vector<Entity> get_payloads(EntityComponentManager& ecm,
    const Entity& lift_entity)
  {
    std::vector<Entity> payloads;
    const auto& lift_pose =
      ecm.Component<components::Pose>(lift_entity)->Data();
    auto pose = _initial_poses.find(lift_entity);
    if (pose == _initial_poses.end())
      return payloads;
    auto aabb_it = _initial_aabbs.find(lift_entity);
    if (aabb_it == _initial_aabbs.end())
      return payloads;
    const gz::math::Vector3d displacement =
      lift_pose.CoordPositionSub(pose->second);
    // Calculate current AABB of lift assuming it hasn't tilted/deformed
    gz::math::AxisAlignedBox lift_aabb = aabb_it->second + displacement;

    ecm.Each<components::Model, components::Pose>(
      [&](const Entity& entity,
      const components::Model*,
      const components::Pose* pose
      ) -> bool
      {
        const auto payload_position = pose->Data().Pos();
        if (entity != lift_entity)
        { // Could possibly check bounding box intersection too, but this suffices
          if (lift_aabb.Contains(payload_position))
          {
            payloads.push_back(entity);
          }
        }
        return true;
      });
    return payloads;
  }

  void initialize_components(EntityComponentManager& ecm)
  {
    ecm.Each<components::Lift>([&](const Entity& entity,
      const components::Lift* lift_comp) -> bool
      {
        const auto& lift = lift_comp->Data();
        auto cabin_joint_entity =
        Model(entity).JointByName(ecm, lift.cabin_joint);
        enableComponent<components::AxisAlignedBox>(ecm, entity);
        enableComponent<components::JointPosition>(ecm, cabin_joint_entity);
        ecm.CreateComponent<components::JointVelocityCmd>(cabin_joint_entity,
        components::JointVelocityCmd({0.0}));

        LiftCommand lift_command;
        lift_command.request_type = LiftRequest::REQUEST_AGV_MODE;
        // Set the initial floor
        const auto target_it = lift.floors.find(lift.initial_floor);
        auto initial_floor = std::string("");
        auto target_elevation = std::numeric_limits<double>::max();
        if (target_it != lift.floors.end())
        {
          initial_floor = target_it->first;
          target_elevation = target_it->second.elevation;
        }
        else
        {
          gzwarn << "Initial floor not found for lift [" << lift.name
                 << "], setting elevation to first floor" << std::endl;
          for (const auto& [name, floor]: lift.floors)
          {
            if (floor.elevation < target_elevation)
            {
              target_elevation = floor.elevation;
              initial_floor = name;
            }
          }

          if (lift.floors.empty())
          {
            gzwarn << "The lift [" << lift.name << "] does not support any "
                   << "floors. This is probably an error in your building "
                   << "configuration." << std::endl;
            target_elevation = 0.0;
          }
        }
        lift_command.destination_floor = initial_floor;
        _last_lift_command[entity] = lift_command;

        std::vector<double> joint_position = {target_elevation};
        ecm.CreateComponent<components::JointPositionReset>(entity,
        components::JointPositionReset{joint_position});
        ecm.CreateComponent<components::LiftCmd>(entity,
        components::LiftCmd{lift_command});
        return true;
      });
  }

  void read_aabbs(EntityComponentManager& ecm)
  {
    ecm.Each<components::AxisAlignedBox,
      components::Pose>([&](const Entity& entity,
      const components::AxisAlignedBox* aabb,
      const components::Pose* pose) -> bool
      {
        // Optimization: Read and store lift's pose and AABB whenever available, then
        // delete the AABB component once read. Not deleting it causes rtf to drop by
        // a 3-4x factor whenever the lift moves.
        const double volume = aabb->Data().Volume();
        if (volume > 0 && std::isfinite(volume))
        {
          // TODO(luca) this could be a component instead of a hash map
          _initial_aabbs[entity] = aabb->Data();
          _initial_poses[entity] = pose->Data();
          enableComponent<components::AxisAlignedBox>(ecm, entity, false);
        }
        return true;
      });
  }

  std::vector<std::string> get_available_floors(const LiftData& lift) const
  {
    std::vector<std::string> floors;
    floors.reserve(lift.floors.size());
    for (const auto& [name, floor] : lift.floors)
    {
      floors.push_back(name);
    }
    return floors;
  }

  std::string get_current_floor(const Entity& entity,
    EntityComponentManager& ecm,
    const LiftData& lift) const
  {
    // TODO update current_floor only when lift reaches its destination
    const auto joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
    if (joint_entity == kNullEntity)
    {
      gzwarn << "Cabin entity not found" << std::endl;
      return "";
    }

    const auto lift_pos =
      ecm.Component<components::JointPosition>(joint_entity)->Data()[0];

    double smallest_error = std::numeric_limits<double>::infinity();
    std::string closest_floor_name;
    for (const auto& [name, floor] : lift.floors)
    {
      double tmp_error = abs(lift_pos - floor.elevation);
      if (tmp_error < smallest_error)
      {
        smallest_error = tmp_error;
        closest_floor_name = name;
      }
    }
    return closest_floor_name;
  }

  // Returns all the doors for the given floor
  std::vector<Entity> get_floor_doors(EntityComponentManager& ecm,
    const LiftData& lift,
    const std::string& floor_name) const
  {
    std::vector<Entity> doors;
    const auto floor_it = lift.floors.find(floor_name);
    if (floor_it == lift.floors.end())
    {
      return doors;
    }

    for (const auto& door_pair : floor_it->second.doors)
    {
      const auto shaft_door = entity_by_name(ecm, door_pair.shaft_door);
      if (shaft_door != kNullEntity)
        doors.push_back(shaft_door);
      const auto cabin_door = entity_by_name(ecm, door_pair.cabin_door);
      if (cabin_door != kNullEntity)
        doors.push_back(cabin_door);
    }
    return doors;
  }

  uint8_t get_door_state(const Entity& entity, EntityComponentManager& ecm,
    const LiftData& lift) const
  {
    auto cur_floor = get_current_floor(entity, ecm, lift);
    auto doors = get_floor_doors(ecm, lift, cur_floor);

    bool all_open = true;
    bool all_closed = true;

    for (const auto& door: doors)
    {
      const auto door_state = ecm.Component<components::DoorStateComp>(door);
      if (door_state == nullptr)
      {
        gzwarn << "Door state for lift not found" << std::endl;
        continue;
      }
      if (door_state->Data() != DoorModeCmp::OPEN)
        all_open = false;
      if (door_state->Data() != DoorModeCmp::CLOSE)
        all_closed = false;
    }

    if (all_open)
      return DoorMode::MODE_OPEN;
    else if (all_closed)
      return DoorMode::MODE_CLOSED;
    return DoorMode::MODE_MOVING;
  }

  uint8_t get_motion_state(const Entity& entity, EntityComponentManager& ecm,
    const LiftData& lift,
    const std::string& destination_floor) const
  {
    const auto joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
    if (joint_entity == kNullEntity)
    {
      gzwarn << "Cabin entity not found" << std::endl;
      return LiftState::MOTION_STOPPED;
    }

    const auto lift_pos =
      ecm.Component<components::JointPosition>(joint_entity)->Data()[0];
    const auto target_it = lift.floors.find(destination_floor);

    if (target_it != lift.floors.end())
    {
      const auto& target_elevation = target_it->second.elevation;
      if (std::abs(target_elevation - lift_pos) < lift.params.dx_min)
        return LiftState::MOTION_STOPPED;
      if (target_elevation - lift_pos > 0)
        return LiftState::MOTION_UP;
      if (target_elevation - lift_pos < 0)
        return LiftState::MOTION_DOWN;
    }
    return LiftState::MOTION_STOPPED;
  }

  void command_doors(EntityComponentManager& ecm,
    const std::vector<Entity>& doors,
    DoorModeCmp door_state) const
  {
    for (const auto& entity : doors)
      ecm.CreateComponent<components::DoorCmd>(entity,
        components::DoorCmd(door_state));
  }

  // TODO(luca) move this to a common block and reuse in all plugins
  Entity entity_by_name(EntityComponentManager& ecm,
    const std::string& name) const
  {
    // Lookup the cache first
    auto it = _cached_entity_by_names.find(name);
    if (it != _cached_entity_by_names.end())
      return it->second;
    auto entity = ecm.EntityByComponents(components::Name(name));
    if (entity != kNullEntity)
    {
      _cached_entity_by_names[name] = entity;
    }
    return entity;
  }

  bool all_doors_at_state(EntityComponentManager& ecm,
    const std::vector<Entity>& doors,
    DoorModeCmp cmd) const
  {
    for (const auto& entity : doors)
    {
      const auto* door_state_comp = ecm.Component<components::DoorStateComp>(
        entity);
      // TODO log
      if (door_state_comp == nullptr)
      {
        continue;
      }
      if (door_state_comp->Data() != cmd)
        return false;
    }
    return true;
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

    return compute_desired_rate_of_change(
      dx, current_velocity, params, dt);
  }

  double command_lift(const Entity& entity, EntityComponentManager& ecm,
    const LiftData& lift, double dt, double target_elevation,
    double cur_elevation)
  {
    auto joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
    auto target_vel = 0.0;
    if (joint_entity != kNullEntity)
    {
      target_vel = calculate_target_velocity(target_elevation, cur_elevation,
          _last_cmd_vel[joint_entity], dt,
          lift.params);
      ecm.Component<components::JointVelocityCmd>(joint_entity)->Data() =
      {target_vel};
      _last_cmd_vel[joint_entity] = target_vel;
    }
    return target_vel;
  }

  LiftState get_current_state(const Entity& entity, EntityComponentManager& ecm,
    const LiftData& lift,
    const components::LiftCmd* lift_cmd)
  {
    LiftState msg;

    msg.available_floors = get_available_floors(lift);
    msg.current_floor = get_current_floor(entity, ecm, lift);
    msg.destination_floor = lift_cmd != nullptr ?
      lift_cmd->Data().destination_floor : msg.current_floor;

    msg.door_state = get_door_state(entity, ecm, lift);
    msg.motion_state = lift_cmd != nullptr ?
      get_motion_state(entity, ecm, lift,
        lift_cmd->Data().destination_floor) : msg.MOTION_STOPPED;
    msg.current_mode = msg.MODE_AGV;
    msg.session_id = lift_cmd != nullptr ?
      lift_cmd->Data().session_id : "";
    auto it = _last_lift_command.find(entity);
    if (it != _last_lift_command.end())
    {
      msg.session_id = it->second.session_id;
    }
    return msg;
  }

public:

  void Configure(const Entity& /*entity*/,
    const std::shared_ptr<const sdf::Element>& /*sdf*/,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    std::string plugin_name("rmf_simulation_lift_manager");
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    const auto reliable_qos = rclcpp::QoS(100).reliable();
    _lift_state_pub = _ros_node->create_publisher<LiftState>(
      "lift_states", reliable_qos);

    _lift_request_sub = _ros_node->create_subscription<LiftRequest>(
      "lift_requests", reliable_qos,
      [this, ecm = &ecm](LiftRequest::UniquePtr msg)
      {
        // Find entity with the name and create a DoorCmd component
        auto entity = entity_by_name(*ecm, msg->lift_name);
        const auto* lift_comp = ecm->Component<components::Lift>(entity);
        if (entity != kNullEntity && lift_comp != nullptr)
        {
          const auto& available_floors = lift_comp->Data().floors;
          if (available_floors.find(msg->destination_floor) ==
          available_floors.end())
          {
            gzwarn << "Received request for unavailable floor [" <<
              msg->destination_floor << "]" << std::endl;
            return;
          }
          LiftCommand lift_command;
          lift_command.request_type = msg->request_type;
          lift_command.destination_floor = msg->destination_floor;
          lift_command.session_id = msg->request_type ==
          msg->REQUEST_END_SESSION ?
          "" : msg->session_id;

          lift_command.door_state = msg->door_state == msg->DOOR_OPEN ?
          DoorModeCmp::OPEN : DoorModeCmp::CLOSE;

          // Trigger an error if a request, different from previous one, comes in
          const auto* cur_lift_cmd_comp =
          ecm->Component<components::LiftCmd>(entity);
          if (cur_lift_cmd_comp)
          {
            const auto& cur_lift_cmd = cur_lift_cmd_comp->Data();
            if (cur_lift_cmd.destination_floor != msg->destination_floor ||
            cur_lift_cmd.request_type != msg->request_type ||
            cur_lift_cmd.session_id != msg->session_id)
            {
              gzwarn << "Discarding request: [" << msg->lift_name <<
                "] is busy at the moment" << std::endl;
              return;
            }
          }
          else
          {
            auto it = _last_lift_command.find(entity);
            if (it != _last_lift_command.end() &&
            (it->second.destination_floor != msg->destination_floor ||
            it->second.request_type != msg->request_type ||
            it->second.session_id != msg->session_id))
            {
              RCLCPP_INFO(_ros_node->get_logger(),
              "Lift [%s] requested at level [%s]",
              msg->lift_name.c_str(), msg->destination_floor.c_str());
              _last_lift_command[entity] = lift_command;
              ecm->CreateComponent<components::LiftCmd>(entity,
              components::LiftCmd(lift_command));
            }
          }
        }
        else
        {
          gzwarn << "Request received for lift " << msg->lift_name <<
            " but it is not being simulated" << std::endl;
        }
      });

    RCLCPP_INFO(_ros_node->get_logger(),
      "Starting LiftManager");
  }

  void initialize_pub_times(EntityComponentManager& ecm)
  {
    ecm.Each<components::Lift>([&](const Entity& e,
      const components::Lift*) -> bool
      {
        _last_state_pub[e] = ((double) std::rand()) /
        ((double) RAND_MAX/PUBLISH_DT);
        return true;
      });
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // Read from components that may not have been initialized in configure()
    if (!_components_initialized)
    {
      initialize_components(ecm);
      initialize_pub_times(ecm);
      _components_initialized = true;
      return;
    }

    if (!_aabb_read)
    {
      read_aabbs(ecm);
      _aabb_read = true;
    }

    rclcpp::spin_some(_ros_node);

    // Don't update the pose if the simulation is paused
    if (info.paused)
      return;

    std::unordered_set<Entity> finished_cmds;

    const double dt = to_seconds(info.dt);
    // Command lifts
    ecm.Each<components::Lift,
      components::Pose,
      components::LiftCmd>([&](const Entity& entity,
      const components::Lift* lift_comp,
      const components::Pose* pose_comp,
      const components::LiftCmd* lift_cmd_comp) -> bool
      {
        const auto& lift = lift_comp->Data();
        const auto& pose = pose_comp->Data();

        const auto& lift_cmd = lift_cmd_comp->Data();

        const auto& destination_floor = lift_cmd.destination_floor;
        const double target_elevation = lift.floors.at(
          destination_floor).elevation;
        const auto target_door_state = lift_cmd.door_state;
        const std::string cur_floor = get_current_floor(entity, ecm, lift);

        const auto doors = get_floor_doors(ecm, lift, cur_floor);

        if (std::abs(pose.Z() - target_elevation) < lift.params.dx_min)
        {
          // Just command the doors to the target state
          command_doors(ecm, doors, target_door_state);
          // Clear the command if it was finished
          if (destination_floor == cur_floor &&
          all_doors_at_state(ecm, doors, target_door_state))
            finished_cmds.insert(entity);
        }
        else
        {
          // Make sure doors are closed before moving to next floor
          command_doors(ecm, doors, DoorModeCmp::CLOSE);
          if (all_doors_at_state(ecm, doors, DoorModeCmp::CLOSE))
          {
            auto target_velocity =
            command_lift(entity, ecm, lift, dt, target_elevation, pose.Z());
            // Move payloads as well
            if (target_velocity != 0.0)
            {
              auto payloads = get_payloads(ecm, entity);
              for (const Entity& payload : payloads)
              {
                if (ecm.EntityHasComponentType(payload,
                components::LinearVelocityCmd().TypeId()))
                {
                  auto lin_vel_cmd =
                  ecm.Component<components::LinearVelocityCmd>(payload);
                  lin_vel_cmd->Data()[2] = target_velocity;
                }
              }
            }
          }
        }

        return true;
      });

    const double t = to_seconds(info.simTime);
    // Update states
    ecm.Each<components::Lift,
      components::Name>([&](const Entity& entity,
      const components::Lift* lift_comp,
      const components::Name* name_comp) -> bool
      {
        const auto& name = name_comp->Data();
        const auto& lift = lift_comp->Data();

        const auto* lift_cmd_comp = ecm.Component<components::LiftCmd>(entity);

        auto current_state = get_current_state(entity, ecm, lift,
        lift_cmd_comp);
        // Will compare to default initialized last state which should be true at startup
        if (current_state != _last_states[entity])
        {
          _last_states[entity] = current_state;
          current_state.lift_name = name;
          current_state.lift_time.sec = t;
          current_state.lift_time.nanosec = (t - static_cast<int>(t)) * 1e9;

          _lift_state_pub->publish(current_state);
        }

        return true;
      });

    // Clear finished commands
    for (const auto& e : finished_cmds)
    {
      enableComponent<components::LiftCmd>(ecm, e, false);
    }

    // Publish state
    ecm.Each<components::Lift,
      components::Name>([&](const Entity& e,
      const components::Lift* lift_comp,
      const components::Name* name_comp) -> bool
      {
        auto it = _last_state_pub.find(e);
        if (it != _last_state_pub.end() && t - it->second >= PUBLISH_DT)
        {
          it->second = t;
          const auto& name = name_comp->Data();
          const auto& lift = lift_comp->Data();

          const auto* lift_cmd_comp = ecm.Component<components::LiftCmd>(
            e);

          auto msg = get_current_state(e, ecm, lift, lift_cmd_comp);
          _last_states[e] = msg;
          msg.lift_time.sec = t;
          msg.lift_time.nanosec = (t - static_cast<int>(t)) * 1e9;
          msg.lift_name = name;
          _lift_state_pub->publish(msg);
        }
        return true;
      });
  }
};


GZ_ADD_PLUGIN(
  LiftPlugin,
  System,
  LiftPlugin::ISystemConfigure,
  LiftPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LiftPlugin, "lift")

} // namespace building_sim_gz_plugins
