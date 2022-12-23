#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_building_sim_common/utils.hpp>
#include <rmf_building_sim_common/lift_common.hpp>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>
#include <rmf_building_sim_gz_plugins/components/Lift.hpp>

using namespace ignition::gazebo;

using namespace rmf_building_sim_common;

namespace building_sim_ign {

enum class PhysEnginePlugin {DEFAULT, TPE};
std::unordered_map<std::string, PhysEnginePlugin> plugin_names {
  {"ignition-physics-tpe-plugin", PhysEnginePlugin::TPE}};

//==============================================================================

class IGNITION_GAZEBO_VISIBLE LiftPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  static constexpr double STATE_PUB_DT = 1.0;
  rclcpp::Node::SharedPtr _ros_node;
  //std::vector<Entity> _payloads;

  rclcpp::Publisher<LiftState>::SharedPtr _lift_state_pub;
  rclcpp::Subscription<LiftRequest>::SharedPtr _lift_request_sub;

  std::unordered_map<std::string, ignition::math::AxisAlignedBox> _initial_aabbs;
  std::unordered_map<std::string, ignition::math::Pose3d> _initial_poses;

  std::unordered_map<Entity, double> _last_cmd_vel;

  std::unordered_map<std::string, double> _last_state_pub;

  PhysEnginePlugin _phys_plugin = PhysEnginePlugin::DEFAULT;
  bool _first_iteration = true;

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    enableComponent<components::LinearVelocityCmd>(ecm, entity);
    enableComponent<components::WorldPoseCmd>(ecm, entity);
    const auto pos = ecm.Component<components::Pose>(entity);
    ecm.Component<components::WorldPoseCmd>(entity)->Data() = pos->Data();
  }

  void create_joint_components(Entity entity, EntityComponentManager& ecm)
  {
    enableComponent<components::JointPosition>(ecm, entity);
  }

  void fill_physics_engine(Entity entity, EntityComponentManager& ecm)
  {
    Entity parent = entity;
    while (ecm.ParentEntity(parent))
    {
      parent = ecm.ParentEntity(parent);
    }
    if (ecm.EntityHasComponentType(parent,
      components::PhysicsEnginePlugin().TypeId()))
    {
      const std::string physics_plugin_name =
        ecm.Component<components::PhysicsEnginePlugin>(parent)->Data();
      const auto it = plugin_names.find(physics_plugin_name);
      if (it != plugin_names.end())
      {
        _phys_plugin = it->second;
      }
    }
  }

  std::vector<Entity> get_payloads(EntityComponentManager& ecm)
  {
    std::vector<Entity> payloads;
    /*
    const auto& lift_pose =
      ecm.Component<components::Pose>(_lift_entity)->Data();
    const ignition::math::Vector3d displacement =
      lift_pose.CoordPositionSub(_initial_pose);
    // Calculate current AABB of lift assuming it hasn't tilted/deformed
    ignition::math::AxisAlignedBox lift_aabb = _initial_aabb + displacement;

    ecm.Each<components::Model, components::Pose>(
      [&](const Entity& entity,
      const components::Model*,
      const components::Pose* pose
      ) -> bool
      {
        const auto payload_position = pose->Data().Pos();
        if (entity != _lift_entity)
        { // Could possibly check bounding box intersection too, but this suffices
          if (lift_aabb.Contains(payload_position))
          {
            payloads.push_back(entity);
          }
        }
        return true;
      });
      */
    return payloads;
  }

  void initialize_components(EntityComponentManager& ecm)
  {
    ecm.Each<components::Lift, components::Name>([&](const Entity& entity, const components::Lift* lift_comp, const components::Name* name_comp) -> bool
        {
          const auto& name = name_comp->Data();
          const auto& lift = lift_comp->Data();
          auto cabin_joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
          create_joint_components(cabin_joint_entity, ecm);

          // Optimization: Read and store lift's pose and AABB whenever available, then
          // delete the AABB component once read. Not deleting it causes rtf to drop by
          // a 3-4x factor whenever the lift moves.
          const auto& aabb_component =
            ecm.Component<components::AxisAlignedBox>(entity);
          const auto& pose_component =
            ecm.Component<components::Pose>(entity);

          if (aabb_component && pose_component)
          {
            const double volume = aabb_component->Data().Volume();
            if (volume > 0 && volume != std::numeric_limits<double>::infinity())
            {
              // TODO(luca) this could be a component instead of a hash map
              _initial_aabbs[name] = aabb_component->Data();
              _initial_poses[name] = pose_component->Data();
              enableComponent<components::AxisAlignedBox>(ecm, entity, false);
            }
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

  std::string get_current_floor(const Entity& entity, EntityComponentManager& ecm, const LiftData& lift) const
  {
    // TODO update current_floor only when lift reaches its destination
    const auto joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
    if (joint_entity == kNullEntity)
    {
      ignwarn << "Cabin entity not found" << std::endl;
      return "";
    }

    const auto lift_pos = ecm.Component<components::JointPosition>(joint_entity)->Data()[0];

    double smallest_error = std::numeric_limits<double>::max();
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
  std::vector<Entity> get_floor_doors(EntityComponentManager& ecm, const LiftData& lift, const std::string& floor_name) const {
    std::vector<Entity> doors;
    for (const auto& door_pair : lift.floors.at(floor_name).doors)
    {
      const auto shaft_door = ecm.EntityByComponents(components::Name(door_pair.shaft_door));
      if (shaft_door != kNullEntity)
        doors.push_back(shaft_door);
      const auto cabin_door = ecm.EntityByComponents(components::Name(door_pair.cabin_door));
      if (cabin_door != kNullEntity)
        doors.push_back(cabin_door);
      else
        std::cout << "Cabin door " << door_pair.cabin_door << "not found" << std::endl;
    }
    return doors;
  }

  uint8_t get_door_state(const Entity& entity, EntityComponentManager& ecm, const LiftData& lift) const
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
        ignwarn << "Door state for lift not found" << std::endl;
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

  uint8_t get_motion_state(const Entity& entity, EntityComponentManager& ecm, const LiftData& lift, const std::string& destination_floor) const
  {
    const auto joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
    if (joint_entity == kNullEntity)
    {
      ignwarn << "Cabin entity not found" << std::endl;
      return LiftState::MOTION_STOPPED;
    }

    const auto lift_pos = ecm.Component<components::JointPosition>(joint_entity)->Data()[0];
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

  void command_doors(EntityComponentManager& ecm, const std::vector<Entity>& doors, DoorModeCmp door_state) const
  {
    for (const auto& entity : doors)
      ecm.CreateComponent<components::DoorCmd>(entity, components::DoorCmd(door_state));
  }

  bool all_doors_at_state(EntityComponentManager& ecm, const std::vector<Entity>& doors, DoorModeCmp cmd) const
  {
    for (const auto& entity : doors)
    {
      const auto* door_state_comp = ecm.Component<components::DoorStateComp>(entity);
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

  void command_lift(const Entity& entity, EntityComponentManager& ecm, const LiftData& lift, double dt, double target_elevation, double cur_elevation)
  {
    auto joint_entity = Model(entity).JointByName(ecm, lift.cabin_joint);
    if (joint_entity != kNullEntity)
    {
      auto target_vel = calculate_target_velocity(target_elevation, cur_elevation, _last_cmd_vel[joint_entity], dt, lift.params);
      ecm.CreateComponent<components::JointVelocityCmd>(joint_entity, components::JointVelocityCmd({target_vel}));
      _last_cmd_vel[joint_entity] = target_vel;
      std::cout << "Commanding lift to " << target_elevation << " from " << cur_elevation << " target vel is " << target_vel << std::endl;
    }
  }

public:

  void Configure(const Entity& /*entity*/,
    const std::shared_ptr<const sdf::Element>& /*sdf*/,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    ignerr << "Entering lift plugin" << std::endl;
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    std::string plugin_name("rmf_simulation_lift_manager");
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    // initialize pub & sub
    _lift_state_pub = _ros_node->create_publisher<LiftState>(
      "lift_states", rclcpp::SystemDefaultsQoS());

    _lift_request_sub = _ros_node->create_subscription<LiftRequest>(
      "lift_requests", rclcpp::SystemDefaultsQoS(),
      [&](LiftRequest::UniquePtr msg)
      {
        // Find entity with the name and create a DoorCmd component
        auto entity = ecm.EntityByComponents(components::Name(msg->lift_name));
        const auto* lift_comp = ecm.Component<components::Lift>(entity);
        if (entity != kNullEntity || lift_comp == nullptr)
        {
          const auto& available_floors = lift_comp->Data().floors;
          if (available_floors.find(msg->destination_floor) == available_floors.end())
          {
            ignwarn << "Received request for unavailable floor [" << msg->destination_floor << "]" << std::endl;
            return;
          }
          LiftCommand lift_command;
          lift_command.request_type = msg->request_type;
          lift_command.destination_floor = msg->destination_floor;
          lift_command.session_id = msg->request_type == msg->REQUEST_END_SESSION ?
            "" : msg->session_id;

          lift_command.door_state = msg->door_state == msg->DOOR_OPEN ?
            DoorModeCmp::OPEN : DoorModeCmp::CLOSE;

          // Trigger an error if a request, different from previous one, comes in
          const auto* cur_lift_cmd_comp = ecm.Component<components::LiftCmd>(entity);
          if (cur_lift_cmd_comp)
          {
            const auto& cur_lift_cmd = cur_lift_cmd_comp->Data();
            if (cur_lift_cmd.destination_floor != msg->destination_floor ||
                cur_lift_cmd.request_type != msg->request_type ||
                cur_lift_cmd.session_id != msg->session_id)
            {
              ignwarn << "Discarding request: [" << msg->lift_name <<"] is busy at the moment" << std::endl;
              return;
            }
          }
          else
          {
            // TODO consistency between rclcpp and gz logger
            RCLCPP_INFO(_ros_node->get_logger(),
              "Lift [%s] requested at level [%s]",
              msg->lift_name.c_str(), msg->destination_floor.c_str());
          }
          ecm.CreateComponent<components::LiftCmd>(entity, components::LiftCmd(lift_command));
        }
        else
        {
          ignwarn << "Request received for lift " << msg->lift_name <<
            " but it is not being simulated" << std::endl;
        }
      });

    RCLCPP_INFO(_ros_node->get_logger(),
      "Starting LiftManager");
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // Read from components that may not have been initialized in configure()
    if (_first_iteration)
    {
      initialize_components(ecm);
      // TODO(luca) restore TPE functionality
      //fill_physics_engine(_lift_entity, ecm);

      /*
      double lift_elevation = _lift_common->get_elevation();
      if (_phys_plugin == PhysEnginePlugin::DEFAULT)
      {
        create_joint_components(_cabin_joint, ecm);
        auto position_cmd = ecm.Component<components::JointPositionReset>(
          _cabin_joint);
        position_cmd->Data()[0] = lift_elevation;
      }
      else
      {
        create_entity_components(_lift_entity, ecm);
        auto position_cmd = ecm.Component<components::WorldPoseCmd>(
          _lift_entity);
        position_cmd->Data().Pos().Z() = lift_elevation;
      }

      */
      _first_iteration = false;
      return;
    }

    rclcpp::spin_some(_ros_node);

    // Don't update the pose if the simulation is paused
    if (info.paused)
      return;

    std::unordered_set<Entity> finished_cmds;

    // Update state
    double dt =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).
      count()) * 1e-9;
    ecm.Each<components::Lift, components::Pose>([&](const Entity& entity, const components::Lift* lift_comp, const components::Pose* pose_comp) -> bool
    {
      const auto& lift = lift_comp->Data();
      const auto& pose = pose_comp->Data();

      const auto* lift_cmd_comp = ecm.Component<components::LiftCmd>(entity);

      const auto destination_floor = lift_cmd_comp != nullptr ?
        lift_cmd_comp->Data().destination_floor : lift.initial_floor;
      const double target_elevation = lift.floors.at(destination_floor).elevation;
      const auto target_door_state = lift_cmd_comp != nullptr ?
        lift_cmd_comp->Data().door_state : DoorModeCmp::CLOSE;
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
          command_lift(entity, ecm, lift, dt, target_elevation, pose.Z());
        }
      }

      return true;
    });

    // Clear finished commands
    for (const auto& e : finished_cmds)
      enableComponent<components::LiftCmd>(ecm, e, false);

    // Publish state
    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;
    ecm.Each<components::Lift, components::Name>([&](const Entity& entity, const components::Lift* lift_comp, const components::Name* name_comp) -> bool
    {
      const auto& name = name_comp->Data();
      const auto& lift = lift_comp->Data();

      const auto* lift_cmd_comp = ecm.Component<components::LiftCmd>(entity);

      if (_last_state_pub.find(name) == _last_state_pub.end())
        _last_state_pub[name] = static_cast<double>(std::rand()) / RAND_MAX;
      if (t - _last_state_pub[name] >= STATE_PUB_DT)
      {
        LiftState msg;
        msg.lift_time.sec = t;
        msg.lift_time.nanosec = (t - static_cast<int>(t)) * 1e9;
        msg.lift_name = name;

        msg.available_floors = get_available_floors(lift);
        msg.current_floor = get_current_floor(entity, ecm, lift);
        msg.destination_floor = lift_cmd_comp != nullptr ?
          lift_cmd_comp->Data().destination_floor : msg.current_floor;

        msg.door_state = get_door_state(entity, ecm, lift);
        msg.motion_state = lift_cmd_comp != nullptr ?
          get_motion_state(entity, ecm, lift, lift_cmd_comp->Data().destination_floor) : msg.MOTION_STOPPED;
        msg.current_mode = msg.MODE_AGV;
        msg.session_id = lift_cmd_comp != nullptr ?
          lift_cmd_comp->Data().session_id : "";
        _lift_state_pub->publish(msg);
        _last_state_pub[name] = t;
      }

      return true;
    });
    /*
    // Read from either joint data or model data based on physics engine
    if (_phys_plugin == PhysEnginePlugin::DEFAULT)
    {
      position = ecm.Component<components::JointPosition>(
        _cabin_joint)->Data()[0];
      velocity = ecm.Component<components::JointVelocity>(
        _cabin_joint)->Data()[0];
    }
    else
    {
      position = ecm.Component<components::Pose>(
        _lift_entity)->Data().Pos().Z();
      auto lift_vel_cmd = ecm.Component<components::LinearVelocityCmd>(
        _lift_entity);
      velocity = lift_vel_cmd->Data()[2];
    }

    auto result = _lift_common->update(t, position, velocity);

    // Move either joint or lift cabin based on physics engine used
    if (_phys_plugin == PhysEnginePlugin::DEFAULT)
    {
      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(
        _cabin_joint);
      vel_cmd->Data()[0] = result.velocity;
    }
    else
    {
      auto lift_vel_cmd = ecm.Component<components::LinearVelocityCmd>(
        _lift_entity);
      lift_vel_cmd->Data()[2] = result.velocity;
    }

    // Move any payloads that need to be manually moved
    // (i.e. have a LinearVelocityCmd component that exists)
    if (_lift_common->motion_state_changed())
    {
      _payloads = get_payloads(ecm);
    }

    for (const Entity& payload : _payloads)
    {
      if (ecm.EntityHasComponentType(payload,
        components::LinearVelocityCmd().TypeId()))
      {
        auto lin_vel_cmd =
          ecm.Component<components::LinearVelocityCmd>(payload);
        lin_vel_cmd->Data()[2] = result.velocity;
      }
    }
    */
  }
};


IGNITION_ADD_PLUGIN(
  LiftPlugin,
  System,
  LiftPlugin::ISystemConfigure,
  LiftPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LiftPlugin, "lift")

} // namespace building_sim_ign
