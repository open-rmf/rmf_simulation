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
    enableComponent<components::JointPositionReset>(ecm, entity);
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

public:

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
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
        if (entity != kNullEntity)
        {
          LiftCommand lift_command;
          lift_command.request_type = msg->request_type;
          lift_command.destination_floor = msg->destination_floor;
          lift_command.session_id = msg->request_type == msg->REQUEST_END_SESSION ?
            "" : msg->session_id;

          lift_command.door_state = msg->door_state == msg->DOOR_OPEN ?
            DoorCommand::OPEN : DoorCommand::CLOSE;

          ecm.CreateComponent<components::LiftCmd>(entity, components::LiftCmd(lift_command));
        }
        else
        {
          ignwarn << "Request received for lift " << msg->lift_name <<
            " but it is not being simulated" << std::endl;
        }
        // TODO this checks on processing of the command
        /*
        if (_floor_name_to_elevation.find(
          msg->destination_floor) == _floor_name_to_elevation.end())
        {
          RCLCPP_INFO(logger(),
          "Received request for unavailable floor [%s]",
          msg->destination_floor.c_str());
          return;
        }

        // Trigger an error if a request, different from previous one, comes in
        // Noop if request is the same
        if (_lift_request)
        {
          if (_lift_request->destination_floor != msg->destination_floor ||
          _lift_request->request_type != msg->request_type ||
          _lift_request->session_id != msg->session_id)
          {
            RCLCPP_INFO(logger(),
            "Discarding request: [%s] is busy at the moment",
            _lift_name.c_str());
          }
          return;
        }

        _lift_request = std::move(msg);
        RCLCPP_INFO(logger(),
        "Lift [%s] requested at level [%s]",
        _lift_name.c_str(), _lift_request->destination_floor.c_str());
        */
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

    // Update state
    ecm.Each<components::Lift, components::Name>([&](const Entity& entity, const components::Lift* lift_comp, const components::Name* name_comp) -> bool
    {


      return true;
    });

    // Publish state
    ecm.Each<components::Lift, components::Name>([&](const Entity& entity, const components::Lift* lift_comp, const components::Name* name_comp) -> bool
    {
      const auto& name = name_comp->Data();
      const auto& lift = lift_comp->Data();

      double t =
        (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
        count()) * 1e-9;
      if (_last_state_pub.find(name) == _last_state_pub.end())
        _last_state_pub[name] = static_cast<double>(std::rand()) / RAND_MAX;
      if (t - _last_state_pub[name] >= STATE_PUB_DT)
      {
        LiftState msg;
        msg.lift_time.sec = t;
        msg.lift_time.nanosec = (t - static_cast<int>(t)) * 1e9;
        msg.lift_name = name;
        // TODO
        msg.door_state = msg.DOOR_CLOSED;
        // TODO
        msg.motion_state = msg.MOTION_STOPPED;
        msg.current_mode = msg.MODE_AGV;
        // TODO
        msg.session_id = "";
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
