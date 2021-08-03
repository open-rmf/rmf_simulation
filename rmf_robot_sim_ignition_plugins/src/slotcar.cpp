#include <unordered_map>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <rclcpp/rclcpp.hpp>

#include <rmf_robot_sim_common/utils.hpp>
#include <rmf_robot_sim_common/slotcar_common.hpp>

#include <rmf_fleet_msgs/msg/location.hpp>

using namespace ignition::gazebo;

enum class PhysEnginePlugin {DEFAULT, TPE};
std::unordered_map<std::string, PhysEnginePlugin> plugin_names {
  {"ignition-physics-tpe-plugin", PhysEnginePlugin::TPE}};

class IGNITION_GAZEBO_VISIBLE SlotcarPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& eventMgr) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<rmf_robot_sim_common::SlotcarCommon> dataPtr;
  ignition::transport::Node _ign_node;
  rclcpp::Node::SharedPtr _ros_node;

  Entity _entity;
  std::unordered_set<Entity> _payloads;
  std::unordered_set<Entity> _infrastructure;
  double _height = 0;

  PhysEnginePlugin phys_plugin = PhysEnginePlugin::DEFAULT;

  bool first_iteration = true; // Flag for checking if it is first PreUpdate() call
  bool _read_aabb_dimensions = true;
  bool _remove_world_pose_cmd = false;

  void charge_state_cb(const ignition::msgs::Selection& msg);

  void send_control_signals(EntityComponentManager& ecm,
    const std::pair<double, double>& velocities,
    const std::unordered_set<Entity> payloads,
    const double dt);
  void init_infrastructure(EntityComponentManager& ecm);
  void item_dispensed_cb(const ignition::msgs::UInt64_V& msg);
  void item_ingested_cb(const ignition::msgs::Entity& msg);
  bool get_slotcar_height(const ignition::msgs::Entity& req,
    ignition::msgs::Double& rep);
  std::vector<Eigen::Vector3d> get_obstacle_positions(
    EntityComponentManager& ecm);
};

SlotcarPlugin::SlotcarPlugin()
: dataPtr(std::make_unique<rmf_robot_sim_common::SlotcarCommon>())
{
  // Listen for messages that enable/disable charging
  if (!_ign_node.Subscribe("/charge_state", &SlotcarPlugin::charge_state_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/charge_state]" << std::endl;
  }
  // We do rest of initialization during ::Configure
}

SlotcarPlugin::~SlotcarPlugin()
{
}

void SlotcarPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm, EventManager&)
{
  _entity = entity;
  auto model = Model(entity);
  std::string model_name = model.Name(ecm);
  dataPtr->set_model_name(model_name);
  dataPtr->read_sdf(sdf);

  // TODO proper argc argv
  char const** argv = NULL;
  if (!rclcpp::ok())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + model_name);
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
  // TODO Check if executor is getting callbacks
  //executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  //executor->add_node(_ros_node);
  //executor->spin();
  dataPtr->init_ros_node(_ros_node);

  // Initialize Pose3d component
  if (!ecm.EntityHasComponentType(entity, components::Pose().TypeId()))
    ecm.CreateComponent(entity, components::Pose());
  // Initialize Bounding Box component
  if (!ecm.EntityHasComponentType(entity,
    components::AxisAlignedBox().TypeId()))
    ecm.CreateComponent(entity, components::AxisAlignedBox());
  // Initialize Linear/AngularVelocityCmd components to drive slotcar
  if (!ecm.EntityHasComponentType(_entity,
    components::LinearVelocityCmd().TypeId()))
    ecm.CreateComponent(_entity, components::LinearVelocityCmd());
  if (!ecm.EntityHasComponentType(_entity,
    components::AngularVelocityCmd().TypeId()))
    ecm.CreateComponent(_entity, components::AngularVelocityCmd());

  // Keep track of when a payload is dispensed onto/ingested from slotcar
  // Needed for TPE Plugin to know when to manually move payload via this plugin
  if (!_ign_node.Subscribe("/item_dispensed", &SlotcarPlugin::item_dispensed_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/item_dispensed]" << std::endl;
  }
  if (!_ign_node.Subscribe("/item_ingested", &SlotcarPlugin::item_ingested_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/item_ingested]" << std::endl;
  }
  // Respond to requests asking for height (e.g. for dispenser to dispense object)
  const std::string height_srv_name =
    "/slotcar_height_" + std::to_string(entity);
  if (!_ign_node.Advertise(height_srv_name, &SlotcarPlugin::get_slotcar_height,
    this))
  {
    std::cerr << "Error subscribing to topic [/slotcar_height]" << std::endl;
  }
  
}

void SlotcarPlugin::send_control_signals(EntityComponentManager& ecm,
  const std::pair<double, double>& velocities,
  const std::unordered_set<Entity> payloads,
  const double dt)
{
  auto lin_vel_cmd =
    ecm.Component<components::LinearVelocityCmd>(_entity);
  auto ang_vel_cmd =
    ecm.Component<components::AngularVelocityCmd>(_entity);

  double v_robot = lin_vel_cmd->Data()[0];
  double w_robot = ang_vel_cmd->Data()[2];
  std::array<double, 2> target_vels;
  target_vels = dataPtr->calculate_model_control_signals({v_robot, w_robot},
      velocities, dt);

  lin_vel_cmd->Data()[0] = target_vels[0];
  // lin_vel_cmd->Data()[0] = 0.25;
  ang_vel_cmd->Data()[2] = target_vels[1];
  // ang_vel_cmd->Data()[2] = 0.0;
  //ang_vel_cmd->Data()[2] = M_PI / 2.0;
  // ang_vel_cmd->Data()[2] = M_PI / 180.0 * 900.0;

  // lin_vel_cmd->Data()[0] = velocities.first;
  // lin_vel_cmd->Data()[2] = velocities.second;

  if (dataPtr->model_name() == "ambulance")
  {
    //lin_vel_cmd->Data()[0] = 1;
    // ang_vel_cmd->Data()[2] = -M_PI;
    // printf("v_robot %g velocities.first %g target_vels[0]: %g\n", v_robot, velocities.first, target_vels[0]);
    // printf("w_robot %g velocities.second %g target_vels[1]: %g\n", w_robot, velocities.second, target_vels[1]);
  }
  // printf("lin_vel_cmd->Data()[0]: %g\n", lin_vel_cmd->Data()[0]);

  if (phys_plugin == PhysEnginePlugin::TPE) // Need to manually move any payloads
  {
    for (const Entity& payload : payloads)
    {
      if (!ecm.EntityHasComponentType(payload,
        components::LinearVelocityCmd().TypeId()))
      {
        ecm.CreateComponent(payload,
          components::LinearVelocityCmd({0, 0, 0}));
      }
      if (!ecm.EntityHasComponentType(payload,
        components::AngularVelocityCmd().TypeId()))
      {
        ecm.CreateComponent(payload,
          components::AngularVelocityCmd({0, 0, 0}));
      }
      
      ecm.Component<components::LinearVelocityCmd>(payload)->Data() =
        lin_vel_cmd->Data();
      ecm.Component<components::AngularVelocityCmd>(payload)->Data() =
        ang_vel_cmd->Data();
    }
  }
}

void SlotcarPlugin::init_infrastructure(EntityComponentManager& ecm)
{
  // Cycle through all the static entities with Model and Name components
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name,
    const components::Pose*,
    const components::Static* is_static
    ) -> bool
    {
      if (is_static->Data() == false)
      {
        std::string n = name->Data();
        std::for_each(n.begin(), n.end(), [](char& c)
        {
          c = ::tolower(c);
        });
        if (n.find("door") != std::string::npos ||
        n.find("lift") != std::string::npos)
          _infrastructure.insert(entity);
      }
      return true;
    });
  // Also add itself
  _infrastructure.insert(_entity);
}

std::vector<Eigen::Vector3d> SlotcarPlugin::get_obstacle_positions(
  EntityComponentManager& ecm)
{
  std::vector<Eigen::Vector3d> obstacle_positions;
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name*,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      // Object should not be static
      // It should not be part of infrastructure (doors / lifts)
      // And it should be closer than the "stop" range (checked by common)
      const auto obstacle_position = pose->Data().Pos();
      if (is_static->Data() == false &&
      _infrastructure.find(entity) == _infrastructure.end())
      {
        obstacle_positions.push_back(rmf_plugins_utils::convert_vec(
          obstacle_position));
      }
      return true;
    });
  return obstacle_positions;
}

void SlotcarPlugin::charge_state_cb(const ignition::msgs::Selection& msg)
{
  dataPtr->charge_state_cb(msg.name(), msg.selected());
}

// First element of msg should be the slotcar, and the second should be the payload
void SlotcarPlugin::item_dispensed_cb(const ignition::msgs::UInt64_V& msg)
{
  if (msg.data_size() == 2 && msg.data(0) == _entity)
  {
    Entity new_payload = msg.data(1);
    this->_payloads.insert(new_payload);
  }
}

void SlotcarPlugin::item_ingested_cb(const ignition::msgs::Entity& msg)
{
  if (msg.IsInitialized())
  {
    const std::unordered_set<Entity>::iterator it = _payloads.find(msg.id());
    if (it != _payloads.end())
    {
      _payloads.erase(it);
    }
  }
}

bool SlotcarPlugin::get_slotcar_height(const ignition::msgs::Entity& req,
  ignition::msgs::Double& rep)
{
  if (req.id() == _entity)
  {
    rep.set_data(_height);
    return true;
  }
  return false;
}

void SlotcarPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  // Read from components that may not have been initialized in configure()
  if (first_iteration)
  {
    Entity parent = _entity;
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
        phys_plugin = it->second;
      }
    }
    first_iteration = false;
  }

  // Optimization: Read and store slotcar's dimensions whenever available, then
  // delete the AABB component once read. Not deleting it causes rtf to drop by
  // a 3-4x factor whenever the slotcar moves.
  if (_read_aabb_dimensions)
  {
    const auto& aabb_component =
      ecm.Component<components::AxisAlignedBox>(_entity);
    if (aabb_component)
    {
      const double volume = aabb_component->Data().Volume();
      if (volume > 0 && volume != std::numeric_limits<double>::infinity())
      {
        _height = aabb_component->Data().ZLength();
        ecm.RemoveComponent(_entity, components::AxisAlignedBox().TypeId());
        _read_aabb_dimensions = false;
      }
    }
  }

  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);
  if (_infrastructure.empty())
    init_infrastructure(ecm);

  double dt =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count()) *
    1e-9;
  double time =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count())
    * 1e-9;
#if 1
  
  //printf("name: %s\n", dataPtr->model_name().c_str());
  if (dataPtr->model_name() == "ambulance")
  {
    static bool once = false;
    //printf("do something!\n");
    if (!once)
    {
      std::vector<rmf_fleet_msgs::msg::Location> locations;
      locations.resize(5);

      locations[0].x = 16.998f;
      locations[0].y = -20.419f;
      // locations[0].x = 0.f;
      // locations[0].y = -2.f;
      locations[0].yaw = M_PI / 2.0f;

      locations[1].x = 16.998f;
      //locations[1].x = 17.998f;
      locations[1].y = -16.919f;
      // locations[1].x = 0.f;
      // locations[1].y = 0.f;

      // locations[2].x = 20.498f;
      // locations[2].y = -16.919f;
      locations[2].x = 11.85f;
      locations[2].y = -11.76f;

      //inverted u-turn
      locations[3].x = 5.66f;
      locations[3].y = -17.03f;

      locations[4].x = 5.21f;
      locations[4].y = -20.71f;

      //right u-turn
      // locations[3].x = 16.967f;
      // locations[3].y = -5.67f;

      // locations[4].x = 20.667f;
      // locations[4].y = -5.718f;
      
      using namespace rmf_robot_sim_common;
      dataPtr->nonholonomic_trajectory.clear();

      // add 1st trajectory
      if (locations.size() >= 2)
      {
        NonHolonomicTrajectory traj(
          Eigen::Vector2d(locations[0].x, locations[0].y),
          Eigen::Vector2d(locations[1].x, locations[1].y));

        dataPtr->nonholonomic_trajectory.push_back(traj);
      }

      auto convert_waypoints_to_spline_trajectories = [this](
        const std::vector<rmf_fleet_msgs::msg::Location>& locations)
      {
        for (uint i=2; i<locations.size(); ++i)
        {
          // every 3 waypoints that make a bend
          std::array<Eigen::Vector2d, 3> wp;
          wp[0] = Eigen::Vector2d(locations[i - 2].x, locations[i - 2].y);
          wp[1] = Eigen::Vector2d(locations[i - 1].x, locations[i - 1].y);
          wp[2] = Eigen::Vector2d(locations[i].x, locations[i].y);

          Eigen::Vector2d wp1_to_wp0 = (wp[0] - wp[1]);
          Eigen::Vector2d wp1_to_wp2 = (wp[2] - wp[1]);
          double wp1_to_wp0_len = wp1_to_wp0.norm();
          double wp1_to_wp2_len = wp1_to_wp2.norm();
          Eigen::Vector2d wp1_to_wp0_norm = wp1_to_wp0 / wp1_to_wp0_len;
          Eigen::Vector2d wp1_to_wp2_norm = wp1_to_wp2 / wp1_to_wp2_len;

          double cp = wp1_to_wp0.x() * wp1_to_wp2.y() - wp1_to_wp2.x() * wp1_to_wp0.y();
          cp /= (wp1_to_wp0_len * wp1_to_wp2_len);
          // std::cout << "1to0: " << wp1_to_wp0 << std::endl;
          // std::cout << "1to2: " << wp1_to_wp2 << std::endl;
          // printf("cp: %g\n", cp);

          double bend_delta = asin(cp);
          double half_bend_delta = bend_delta * 0.5;
          
          double half_turn_arc = M_PI / 2.0 - half_bend_delta; // right angle tri, 90 - half_turn_delta
          //use sin rule to obtain length of tangent

          // the computation of min_turning_radius goes deeper than i thought.
          // reference:
          // https://www.vboxautomotive.co.uk/downloads/Calculating%20Radius%20of%20Turn%20from%20Yaw.pdf
          //double min_turning_radius = 0.5;
          double min_turning_radius = (100 * 0.2777 / 0.8 * 0.0174);
          printf("min_turning_radius: %g\n", min_turning_radius);
          
          double target_radius = min_turning_radius;
          double tangent_length = std::abs(target_radius / sin(half_bend_delta) * sin(half_turn_arc));
          // printf("wp1_to_wp0_len: %g wp1_to_wp2_len: %g tangent_length: %g\n",
          //   wp1_to_wp0_len, wp1_to_wp2_len, tangent_length);

          bool has_runway = tangent_length < wp1_to_wp0_len && tangent_length < wp1_to_wp2_len;

          if (std::abs(cp) < 0.05 || !has_runway)
          {
            NonHolonomicTrajectory sp2(
              Eigen::Vector2d(wp[1].x(), wp[1].y()),
              Eigen::Vector2d(wp[2].x(), wp[2].y()));

            NonHolonomicTrajectory& last_traj = dataPtr->nonholonomic_trajectory.back();
            last_traj.v1 = sp2.v0;

            dataPtr->nonholonomic_trajectory.push_back(sp2);
          }
          else
          {
            NonHolonomicTrajectory& last_traj = dataPtr->nonholonomic_trajectory.back();
            
            // bend, build an intermediate spline using turn rate. 
            Eigen::Vector2d tangent0 = wp[1] + tangent_length * wp1_to_wp0_norm;
            Eigen::Vector2d tangent1 = wp[1] + tangent_length * wp1_to_wp2_norm;

            // shorten the last trajectory and set it's heading
            last_traj.x1 = Eigen::Vector2d(tangent0.x(), tangent0.y());
            last_traj.v1 = last_traj.v0;
            
            Eigen::Vector2d ref(1, 0);
            double turn_target_yaw = acos(wp1_to_wp2_norm.dot(ref));
            printf("turn_target_yaw: %g\n", turn_target_yaw);

            NonHolonomicTrajectory turn_traj(
              Eigen::Vector2d(tangent0.x(), tangent0.y()), 
              Eigen::Vector2d(tangent1.x(), tangent1.y()),
              Eigen::Vector2d(0,0),
              true);
            turn_traj.v0 = -wp1_to_wp0_norm;
            turn_traj.v1 = wp1_to_wp2_norm;

            turn_traj.turning_radius = target_radius;
            turn_traj.turn_arc_radians = half_turn_arc * 2.0;
            turn_traj.turn_arclength = 
              (turn_traj.turn_arc_radians / (2.0 * M_PI)) * 2.0 * target_radius * M_PI;

            Eigen::Vector2d wp0_to_wp1_norm = -wp1_to_wp0_norm;
            Eigen::Vector2d perp_wp1_wp2(wp0_to_wp1_norm.y(), -wp0_to_wp1_norm.x());
            if (cp < 0)
              perp_wp1_wp2 = -perp_wp1_wp2;
            turn_traj.turn_circle_center = tangent0 + target_radius * perp_wp1_wp2;

            // std::cout << "tangent0: " << tangent0 << std::endl;
            // std::cout << "tangent1: " << tangent1 << std::endl;

            // printf("turn_arc_radians: %g\n", sp2.turn_arc_radians);
            // printf("sp2.turn_arclength: %g\n", sp2.turn_arclength);
            // fflush(stdout);
            
            // std::cout << "turn_circle_center: " << sp2.turn_circle_center << std::endl;
            // std::cout << "r0:" << (sp2.turn_circle_center - tangent0).norm() << std::endl;
            // std::cout << "r1:" << (sp2.turn_circle_center - tangent1).norm() << std::endl;

            NonHolonomicTrajectory end_traj(
                Eigen::Vector2d(tangent1.x(), tangent1.y()),
                Eigen::Vector2d(wp[2].x(), wp[2].y()));
            end_traj.v0 = wp1_to_wp2_norm;
            end_traj.v1 = wp1_to_wp2_norm;

            dataPtr->nonholonomic_trajectory.push_back(turn_traj);
            dataPtr->nonholonomic_trajectory.push_back(end_traj);
          }
        }

        NonHolonomicTrajectory& last_traj = dataPtr->nonholonomic_trajectory.back();
        last_traj.v1 = last_traj.v0;
      };
      convert_waypoints_to_spline_trajectories(locations);
      once = !once;
    }

    if (_remove_world_pose_cmd)
    {
      ecm.RemoveComponent<components::WorldPoseCmd>(_entity);
      _remove_world_pose_cmd = false;
    }
    
    auto& pose = ecm.Component<components::Pose>(_entity)->Data();
    std::vector<Eigen::Vector3d> obstacle_positions;

    bool snap_world_pose = false;
    auto isometry_pose = rmf_plugins_utils::convert_pose(pose);
    auto velocities = dataPtr->update_nonholonomic(isometry_pose, dt, snap_world_pose);

    //convert back to account for flips
    pose = rmf_plugins_utils::convert_to_pose<ignition::math::Pose3d>(isometry_pose);
    
    if (snap_world_pose)
    {
      if (!ecm.EntityHasComponentType(_entity,
        components::WorldPoseCmd().TypeId()))
      {
        ecm.CreateComponent(_entity, components::WorldPoseCmd());
      }
      auto world_pose_cmd =
        ecm.Component<components::WorldPoseCmd>(_entity);
      world_pose_cmd->Data() = pose;

      // remove world pose command next frame
      _remove_world_pose_cmd = true;
    }
    else
    {
      send_control_signals(ecm, velocities, _payloads, dt);
      // auto lin_vel_cmd =
      //   ecm.Component<components::LinearVelocityCmd>(_entity);
      // auto ang_vel_cmd =
      //   ecm.Component<components::AngularVelocityCmd>(_entity);
      // lin_vel_cmd->Data()[0] = velocities.first;
      // ang_vel_cmd->Data()[2] = velocities.second;
    }
    return;
  }
#endif
  auto pose = ecm.Component<components::Pose>(_entity)->Data();
  auto obstacle_positions = get_obstacle_positions(ecm);
  
  auto p = pose.Pos();
  
  //printf("%s: %g %g!\n", dataPtr->model_name().c_str(), p.X(), p.Y());

  auto velocities =
    dataPtr->update(rmf_plugins_utils::convert_pose(pose),
      obstacle_positions, time);

  send_control_signals(ecm, velocities, _payloads, dt);
}

IGNITION_ADD_PLUGIN(
  SlotcarPlugin,
  System,
  SlotcarPlugin::ISystemConfigure,
  SlotcarPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SlotcarPlugin, "slotcar")
