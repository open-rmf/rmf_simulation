#include <unordered_map>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>

#include <ignition/math/eigen3.hh>
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
  std::unordered_set<Entity> _obstacle_exclusions;
  double _height = 0;

  PhysEnginePlugin phys_plugin = PhysEnginePlugin::DEFAULT;

  bool first_iteration = true; // Flag for checking if it is first PreUpdate() call
  bool _read_aabb_dimensions = true;
  bool _remove_world_pose_cmd = false;

  // Previous velocities, used to do open loop velocity control
  double _prev_v_command = 0.0;
  double _prev_w_command = 0.0;

  void charge_state_cb(const ignition::msgs::Selection& msg);

  void send_control_signals(EntityComponentManager& ecm,
    const std::pair<double, double>& displacements,
    const std::unordered_set<Entity> payloads,
    const double dt,
    const double target_linear_speed_now,
    const double target_linear_speed_destination,
    const std::optional<double>& max_linear_velocity);
  void init_obstacle_exclusions(EntityComponentManager& ecm);
  void item_dispensed_cb(const ignition::msgs::UInt64_V& msg);
  void item_ingested_cb(const ignition::msgs::Entity& msg);
  bool get_slotcar_height(const ignition::msgs::Entity& req,
    ignition::msgs::Double& rep);
  std::vector<Eigen::Vector3d> get_obstacle_positions(
    EntityComponentManager& ecm);

  void path_request_marker_update(
    const rmf_fleet_msgs::msg::PathRequest::SharedPtr);

  void draw_lookahead_marker();

  ignition::msgs::Marker_V _trajectory_marker_msg;
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
  enableComponent<components::Pose>(ecm, entity);
  // Initialize Bounding Box component
  enableComponent<components::AxisAlignedBox>(ecm, entity);
  // Initialize Linear/AngularVelocityCmd components to drive slotcar
  enableComponent<components::LinearVelocityCmd>(ecm, entity);
  enableComponent<components::AngularVelocityCmd>(ecm, entity);

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

  if (dataPtr->display_markers)
  {
    dataPtr->set_path_request_callback(std::bind(&SlotcarPlugin::
      path_request_marker_update,
      this, std::placeholders::_1));
  }
}

void SlotcarPlugin::send_control_signals(EntityComponentManager& ecm,
  const std::pair<double, double>& displacements,
  const std::unordered_set<Entity> payloads,
  const double dt,
  const double target_linear_speed_now,
  const double target_linear_speed_destination,
  const std::optional<double>& max_linear_velocity)
{
  auto lin_vel_cmd =
    ecm.Component<components::LinearVelocityCmd>(_entity);
  auto ang_vel_cmd =
    ecm.Component<components::AngularVelocityCmd>(_entity);

  // Open loop control
  double v_robot = _prev_v_command;
  double w_robot = _prev_w_command;
  std::array<double, 2> target_vels;
  target_vels = dataPtr->calculate_control_signals({v_robot, w_robot},
      displacements, dt, target_linear_speed_now,
      target_linear_speed_destination, max_linear_velocity);

  lin_vel_cmd->Data()[0] = target_vels[0];
  ang_vel_cmd->Data()[2] = target_vels[1];

  // Update previous velocities
  _prev_v_command = target_vels[0];
  _prev_w_command = target_vels[1];

  if (phys_plugin == PhysEnginePlugin::TPE) // Need to manually move any payloads
  {
    for (const Entity& payload : payloads)
    {
      enableComponent<components::LinearVelocityCmd>(ecm, payload);
      enableComponent<components::AngularVelocityCmd>(ecm, payload);

      ecm.Component<components::LinearVelocityCmd>(payload)->Data() =
        lin_vel_cmd->Data();
      ecm.Component<components::AngularVelocityCmd>(payload)->Data() =
        ang_vel_cmd->Data();
    }
  }
}

void SlotcarPlugin::init_obstacle_exclusions(EntityComponentManager& ecm)
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
        n.find("lift") != std::string::npos ||
        n.find("dispensable") != std::string::npos)
          _obstacle_exclusions.insert(entity);
      }
      return true;
    });
  // Also add itself
  _obstacle_exclusions.insert(_entity);
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
      // It should not be part of obstacle exclusions (doors/lifts/dispensables)
      // And it should be closer than the "stop" range (checked by common)
      const auto obstacle_position = pose->Data().Pos();
      if (is_static->Data() == false &&
      _obstacle_exclusions.find(entity) == _obstacle_exclusions.end())
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

void SlotcarPlugin::path_request_marker_update(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  ignition::msgs::Boolean res;
  bool result;
  for (int i = 0; i < _trajectory_marker_msg.marker_size(); ++i)
  {
    auto marker = _trajectory_marker_msg.mutable_marker(i);
    marker->set_action(ignition::msgs::Marker::DELETE_ALL);
  }
  _ign_node.Request(
    "/marker_array", _trajectory_marker_msg, 5000, res, result);
  _trajectory_marker_msg.clear_marker();
  auto line_marker = _trajectory_marker_msg.add_marker();
  line_marker->set_ns(dataPtr->model_name() + "_line");
  line_marker->set_id(1);
  line_marker->set_action(ignition::msgs::Marker::ADD_MODIFY);
  line_marker->set_type(ignition::msgs::Marker::LINE_STRIP);
  line_marker->set_visibility(ignition::msgs::Marker::GUI);
  ignition::msgs::Set(
    line_marker->mutable_material()->mutable_ambient(),
    ignition::math::Color(1, 0, 0, 1));
  ignition::msgs::Set(
    line_marker->mutable_material()->mutable_diffuse(),
    ignition::math::Color(1, 0, 0, 1));

  auto marker_headings = _trajectory_marker_msg.add_marker();
  marker_headings->set_id(1);
  marker_headings->set_ns(dataPtr->model_name() + "_waypoint_headings");
  marker_headings->set_action(ignition::msgs::Marker::ADD_MODIFY);
  marker_headings->set_type(ignition::msgs::Marker::LINE_LIST);
  marker_headings->set_visibility(ignition::msgs::Marker::GUI);
  ignition::msgs::Set(marker_headings->mutable_material()->mutable_ambient(),
    ignition::math::Color(0, 1, 0, 1));
  ignition::msgs::Set(marker_headings->mutable_material()->mutable_diffuse(),
    ignition::math::Color(0, 1, 0, 1));

  auto& locations = msg->path;
  double elevation = 0.5;
  for (size_t i = 0; i < locations.size(); ++i)
  {
    auto loc = locations[i];

    // Add points to the trajectory line, slightly elevated for visibility.
    ignition::msgs::Set(line_marker->add_point(),
      ignition::math::Vector3d(loc.x, loc.y, elevation)
    );

    // Draw waypoints
    ignition::math::Color waypoint_color(0.0, 1.0, 0.0, 1.0);
    auto waypoint_marker = _trajectory_marker_msg.add_marker();
    waypoint_marker->set_ns(dataPtr->model_name() + "_waypoints");
    waypoint_marker->set_id(i+1);
    waypoint_marker->set_action(ignition::msgs::Marker::ADD_MODIFY);
    waypoint_marker->set_type(ignition::msgs::Marker::SPHERE);
    waypoint_marker->set_visibility(ignition::msgs::Marker::GUI);
    ignition::msgs::Set(waypoint_marker->mutable_scale(),
      ignition::math::Vector3d(1.5, 1.5, 1.5));
    ignition::msgs::Set(
      waypoint_marker->mutable_material()->mutable_ambient(),
      waypoint_color);
    ignition::msgs::Set(
      waypoint_marker->mutable_material()->mutable_diffuse(),
      waypoint_color);
    ignition::msgs::Set(waypoint_marker->mutable_pose(),
      ignition::math::Pose3d(loc.x, loc.y, elevation, 0, 0, 0));

    Eigen::Vector2d dir(cos(loc.yaw), sin(loc.yaw));
    double length = 2.0;
    ignition::msgs::Set(marker_headings->add_point(),
      ignition::math::Vector3d(loc.x, loc.y, elevation));
    ignition::msgs::Set(marker_headings->add_point(),
      ignition::math::Vector3d(loc.x + length * dir.x(),
      loc.y + length * dir.y(),
      elevation+0.5));
  }

  _ign_node.Request(
    "/marker_array", _trajectory_marker_msg, 5000, res, result);
}

void SlotcarPlugin::draw_lookahead_marker()
{
  auto lookahead_point = dataPtr->get_lookahead_point();

  // Lookahead point
  ignition::msgs::Marker marker_msg;
  marker_msg.set_ns(dataPtr->model_name() + "_lookahead_point");
  marker_msg.set_id(1);
  marker_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  marker_msg.set_type(ignition::msgs::Marker::CYLINDER);
  marker_msg.set_visibility(ignition::msgs::Marker::GUI);

  ignition::msgs::Set(marker_msg.mutable_pose(),
    ignition::math::Pose3d(
      lookahead_point(0),
      lookahead_point(1),
      lookahead_point(2),
      0, 0, 0));
  const double scale = 1.5;
  ignition::msgs::Set(marker_msg.mutable_scale(),
    ignition::math::Vector3d(scale, scale, scale));
  ignition::msgs::Set(marker_msg.mutable_material()->mutable_ambient(),
    ignition::math::Color(0, 0, 1, 1));
  ignition::msgs::Set(marker_msg.mutable_material()->mutable_diffuse(),
    ignition::math::Color(0, 0, 1, 1));
  _ign_node.Request("/marker", marker_msg);
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
        enableComponent<components::AxisAlignedBox>(ecm, _entity, false);
        _read_aabb_dimensions = false;
      }
    }
  }

  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);

  // After initialization once, this set will have at least one exclusion, which
  // is the itself.
  if (_obstacle_exclusions.empty())
    init_obstacle_exclusions(ecm);

  // Don't update the pose if the simulation is paused
  if (info.paused)
    return;

  double dt =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count()) *
    1e-9;
  double time =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count())
    * 1e-9;

  auto pose = ecm.Component<components::Pose>(_entity)->Data();
  auto obstacle_positions = get_obstacle_positions(ecm);

  auto update_result =
    dataPtr->update(rmf_plugins_utils::convert_pose(pose),
      obstacle_positions, time);

  send_control_signals(ecm, {update_result.v, update_result.w}, _payloads, dt,
    update_result.target_linear_speed_now,
    update_result.target_linear_speed_destination,
    update_result.max_speed);

  if (dataPtr->display_markers)
  {
    draw_lookahead_marker();
  }
}


IGNITION_ADD_PLUGIN(
  SlotcarPlugin,
  System,
  SlotcarPlugin::ISystemConfigure,
  SlotcarPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(SlotcarPlugin, "slotcar")
