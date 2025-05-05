#include <unordered_map>

#include <gz/plugin/Register.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/PhysicsEnginePlugin.hh>

#include <gz/math/eigen3.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <rclcpp/rclcpp.hpp>

#include <rmf_robot_sim_common/utils.hpp>
#include <rmf_robot_sim_common/slotcar_common.hpp>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>
#include <rmf_building_sim_gz_plugins/components/Lift.hpp>

#include <rmf_fleet_msgs/msg/location.hpp>

using namespace gz::sim;

class GZ_SIM_VISIBLE SlotcarPlugin
  : public System,
  public ISystemConfigure,
  public ISystemConfigurePriority,
  public ISystemPreUpdate
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& eventMgr) override;
  int32_t ConfigurePriority() override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  // Distance to attach cart, if none is found attaching will fail
  static constexpr float MIN_ATTACHING_DIST = 1.0;
  std::unique_ptr<rmf_robot_sim_common::SlotcarCommon> dataPtr;
  gz::transport::Node _gz_node;
  rclcpp::Node::SharedPtr _ros_node;

  EntityComponentManager* _ecm;

  Entity _entity;
  Entity _joint_entity = kNullEntity;
  Eigen::Isometry3d _pose;
  std::unordered_set<Entity> _obstacle_exclusions;
  std::unordered_map<Entity, Eigen::Vector3d> _dispensable_positions;
  double _height = 0;

  bool _read_aabb_dimensions = true;

  // Previous velocities, used to do open loop velocity control
  double _prev_v_command = 0.0;
  double _prev_w_command = 0.0;

  void charge_state_cb(const gz::msgs::Selection& msg);

  void send_control_signals(EntityComponentManager& ecm,
    const std::pair<double, double>& displacements,
    const double dt,
    const double target_linear_speed_now,
    const double target_linear_speed_destination,
    const std::optional<double>& max_linear_velocity);
  void init_obstacle_exclusions(EntityComponentManager& ecm);
  bool get_slotcar_height(const gz::msgs::Entity& req,
    gz::msgs::Double& rep);
  std::pair<std::vector<Eigen::Vector3d>, std::unordered_map<Entity,
    Eigen::Vector3d>>
  get_obstacle_positions(EntityComponentManager& ecm);

  void path_request_marker_update(
    const rmf_fleet_msgs::msg::PathRequest::SharedPtr);

  bool attach_cart(bool attach);

  bool attach_entity(const Entity& entity);
  bool detach_entity();

  void draw_lookahead_marker();

  gz::msgs::Marker_V _trajectory_marker_msg;
};

SlotcarPlugin::SlotcarPlugin()
: dataPtr(std::make_unique<rmf_robot_sim_common::SlotcarCommon>())
{
  // Listen for messages that enable/disable charging
  if (!_gz_node.Subscribe("/charge_state", &SlotcarPlugin::charge_state_cb,
    this))
  {
    std::cerr << "Error subscribing to topic [/charge_state]" << std::endl;
  }
  // We do rest of initialization during ::Configure
}

SlotcarPlugin::~SlotcarPlugin()
{
}

bool SlotcarPlugin::attach_cart(bool attach)
{
  if (attach)
  {
    // Find _dispensable_position closest to _pose
    if (_dispensable_positions.size() == 0)
      return false;
    auto min_entity = _dispensable_positions.begin()->first;
    auto min_dist = (_pose.translation() -
      _dispensable_positions.begin()->second).norm();
    for (const auto& [entity, pose] : _dispensable_positions)
    {
      auto dist = (_pose.translation() - pose).norm();
      if (dist < min_dist)
      {
        min_dist = dist;
        min_entity = entity;
      }
    }
    if (min_dist > MIN_ATTACHING_DIST)
      return false;

    return attach_entity(min_entity);
  }
  else
  {
    return detach_entity();
  }
  return false;
}

bool SlotcarPlugin::detach_entity()
{
  _ecm->RequestRemoveEntity(_joint_entity);
  _joint_entity = kNullEntity;
  return true;
}

bool SlotcarPlugin::attach_entity(const Entity& entity)
{
  // A cart was already attached
  if (_joint_entity != kNullEntity)
    return true;

  _joint_entity = _ecm->CreateEntity();

  auto robot_link_entities = _ecm->ChildrenByComponents(_entity,
      components::Link());
  if (robot_link_entities.size() != 1)
  {
    gzwarn << "Robot should only have one link, using first" << std::endl;
  }
  auto robot_link_entity = robot_link_entities[0];

  auto cart_link_entities = _ecm->ChildrenByComponents(entity,
      components::Link());
  if (cart_link_entities.size() != 1)
  {
    gzwarn << "Cart should only have one link, using first" << std::endl;
  }
  auto cart_link_entity = cart_link_entities[0];

  _ecm->CreateComponent(
    _joint_entity,
    components::DetachableJoint({robot_link_entity,
      cart_link_entity, "fixed"}));
  return true;
}

void SlotcarPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm, EventManager&)
{
  _entity = entity;
  _ecm = &ecm;
  auto model = Model(entity);
  std::string model_name = model.Name(ecm);
  dataPtr->set_model_name(model_name);
  dataPtr->read_sdf(sdf);
  dataPtr->set_attach_cart_callback(std::bind(&SlotcarPlugin::
    attach_cart,
    this, std::placeholders::_1));

  // TODO proper argc argv
  char const** argv = NULL;
  if (!rclcpp::ok())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + model_name);
  rmf_plugins_utils::sanitize_node_name(plugin_name);
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

  // Respond to requests asking for height (e.g. for dispenser to dispense object)
  const std::string height_srv_name =
    "/slotcar_height_" + std::to_string(entity);
  if (!_gz_node.Advertise(height_srv_name, &SlotcarPlugin::get_slotcar_height,
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
  const double dt,
  const double target_linear_speed_now,
  const double target_linear_speed_destination,
  const std::optional<double>& max_linear_velocity)
{
  // Open loop control
  double v_robot = _prev_v_command;
  double w_robot = _prev_w_command;
  std::array<double, 2> target_vels;
  target_vels = dataPtr->calculate_control_signals({v_robot, w_robot},
      displacements, dt, target_linear_speed_now,
      target_linear_speed_destination, max_linear_velocity);

  gz::math::Vector3d lin_vel_cmd(0, 0, 0);
  lin_vel_cmd[0] = target_vels[0];
  gz::math::Vector3d ang_vel_cmd(0, 0, 0);
  ang_vel_cmd[2] = target_vels[1];

  ecm.SetComponentData<components::LinearVelocityCmd>(_entity, lin_vel_cmd);
  ecm.SetComponentData<components::AngularVelocityCmd>(_entity, ang_vel_cmd);

  // Update previous velocities
  _prev_v_command = target_vels[0];
  _prev_w_command = target_vels[1];
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
        if (ecm.Component<components::Door>(entity) != nullptr ||
        ecm.Component<components::Lift>(entity) != nullptr ||
        n.find("dispensable") != std::string::npos)
        {
          _obstacle_exclusions.insert(entity);
        }
      }
      return true;
    });
  // Also add itself
  _obstacle_exclusions.insert(_entity);
}

std::pair<std::vector<Eigen::Vector3d>, std::unordered_map<Entity,
  Eigen::Vector3d>>
SlotcarPlugin::get_obstacle_positions(EntityComponentManager& ecm)
{
  std::vector<Eigen::Vector3d> obstacle_positions;
  std::unordered_map<Entity, Eigen::Vector3d> dispensable_positions;
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      const auto& n = name->Data();
      // Object should not be static
      // It should not be part of obstacle exclusions (doors/lifts/dispensables)
      // And it should be closer than the "stop" range (checked by common)
      const auto object_position = pose->Data().Pos();
      if (is_static->Data() == false &&
      _obstacle_exclusions.find(entity) == _obstacle_exclusions.end())
      {
        obstacle_positions.push_back(rmf_plugins_utils::convert_vec(
          object_position));
      }
      // It is a dispensable object
      if (n.find("dispensable") != std::string::npos)
      {
        dispensable_positions.insert({entity, rmf_plugins_utils::convert_vec(
            object_position)});
      }
      return true;
    });
  return {obstacle_positions, dispensable_positions};
}

void SlotcarPlugin::charge_state_cb(const gz::msgs::Selection& msg)
{
  dataPtr->charge_state_cb(msg.name(), msg.selected());
}

bool SlotcarPlugin::get_slotcar_height(const gz::msgs::Entity& req,
  gz::msgs::Double& rep)
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
  gz::msgs::Boolean res;
  bool result;
  for (int i = 0; i < _trajectory_marker_msg.marker_size(); ++i)
  {
    auto marker = _trajectory_marker_msg.mutable_marker(i);
    marker->set_action(gz::msgs::Marker::DELETE_ALL);
  }
  _gz_node.Request(
    "/marker_array", _trajectory_marker_msg, 5000, res, result);
  _trajectory_marker_msg.clear_marker();
  auto line_marker = _trajectory_marker_msg.add_marker();
  line_marker->set_ns(dataPtr->model_name() + "_line");
  line_marker->set_id(1);
  line_marker->set_action(gz::msgs::Marker::ADD_MODIFY);
  line_marker->set_type(gz::msgs::Marker::LINE_STRIP);
  line_marker->set_visibility(gz::msgs::Marker::GUI);
  gz::msgs::Set(
    line_marker->mutable_material()->mutable_ambient(),
    gz::math::Color(1, 0, 0, 1));
  gz::msgs::Set(
    line_marker->mutable_material()->mutable_diffuse(),
    gz::math::Color(1, 0, 0, 1));

  auto marker_headings = _trajectory_marker_msg.add_marker();
  marker_headings->set_id(1);
  marker_headings->set_ns(dataPtr->model_name() + "_waypoint_headings");
  marker_headings->set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_headings->set_type(gz::msgs::Marker::LINE_LIST);
  marker_headings->set_visibility(gz::msgs::Marker::GUI);
  gz::msgs::Set(marker_headings->mutable_material()->mutable_ambient(),
    gz::math::Color(0, 1, 0, 1));
  gz::msgs::Set(marker_headings->mutable_material()->mutable_diffuse(),
    gz::math::Color(0, 1, 0, 1));

  auto& locations = msg->path;
  double elevation = 0.5;
  for (size_t i = 0; i < locations.size(); ++i)
  {
    auto loc = locations[i];

    // Add points to the trajectory line, slightly elevated for visibility.
    gz::msgs::Set(line_marker->add_point(),
      gz::math::Vector3d(loc.x, loc.y, elevation)
    );

    // Draw waypoints
    gz::math::Color waypoint_color(0.0, 1.0, 0.0, 1.0);
    auto waypoint_marker = _trajectory_marker_msg.add_marker();
    waypoint_marker->set_ns(dataPtr->model_name() + "_waypoints");
    waypoint_marker->set_id(i+1);
    waypoint_marker->set_action(gz::msgs::Marker::ADD_MODIFY);
    waypoint_marker->set_type(gz::msgs::Marker::SPHERE);
    waypoint_marker->set_visibility(gz::msgs::Marker::GUI);
    gz::msgs::Set(waypoint_marker->mutable_scale(),
      gz::math::Vector3d(1.5, 1.5, 1.5));
    gz::msgs::Set(
      waypoint_marker->mutable_material()->mutable_ambient(),
      waypoint_color);
    gz::msgs::Set(
      waypoint_marker->mutable_material()->mutable_diffuse(),
      waypoint_color);
    gz::msgs::Set(waypoint_marker->mutable_pose(),
      gz::math::Pose3d(loc.x, loc.y, elevation, 0, 0, 0));

    Eigen::Vector2d dir(cos(loc.yaw), sin(loc.yaw));
    double length = 2.0;
    gz::msgs::Set(marker_headings->add_point(),
      gz::math::Vector3d(loc.x, loc.y, elevation));
    gz::msgs::Set(marker_headings->add_point(),
      gz::math::Vector3d(loc.x + length * dir.x(),
      loc.y + length * dir.y(),
      elevation+0.5));
  }

  _gz_node.Request(
    "/marker_array", _trajectory_marker_msg, 5000, res, result);
}

void SlotcarPlugin::draw_lookahead_marker()
{
  auto lookahead_point = dataPtr->get_lookahead_point();

  // Lookahead point
  gz::msgs::Marker marker_msg;
  marker_msg.set_ns(dataPtr->model_name() + "_lookahead_point");
  marker_msg.set_id(1);
  marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_msg.set_type(gz::msgs::Marker::CYLINDER);
  marker_msg.set_visibility(gz::msgs::Marker::GUI);

  gz::msgs::Set(marker_msg.mutable_pose(),
    gz::math::Pose3d(
      lookahead_point(0),
      lookahead_point(1),
      lookahead_point(2),
      0, 0, 0));
  const double scale = 1.5;
  gz::msgs::Set(marker_msg.mutable_scale(),
    gz::math::Vector3d(scale, scale, scale));
  gz::msgs::Set(marker_msg.mutable_material()->mutable_ambient(),
    gz::math::Color(0, 0, 1, 1));
  gz::msgs::Set(marker_msg.mutable_material()->mutable_diffuse(),
    gz::math::Color(0, 0, 1, 1));
  _gz_node.Request("/marker", marker_msg);
}

int32_t SlotcarPlugin::ConfigurePriority()
{
  // Set the priority down by one so this runs before the infrastructure plugin
  // which should have a default priority of 0. This is important for ensuring
  // that the linear velocity command component is created before the
  // infrastructure plugin runs since the lift needs to adjust its z value, and
  // the physics plugin removes the component on every update.
  return -1;
}

void SlotcarPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
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
    else
    {
      enableComponent<components::AxisAlignedBox>(ecm, _entity);
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

  _pose = rmf_plugins_utils::convert_pose(
    ecm.Component<components::Pose>(_entity)->Data());
  auto [obstacle_positions,
    dispensable_positions] = get_obstacle_positions(ecm);
  _dispensable_positions = std::move(dispensable_positions);

  auto update_result = dataPtr->update(_pose, obstacle_positions, time);

  send_control_signals(ecm, {update_result.v, update_result.w}, dt,
    update_result.target_linear_speed_now,
    update_result.target_linear_speed_destination,
    update_result.max_speed);

  if (dataPtr->display_markers)
  {
    draw_lookahead_marker();
  }
}


GZ_ADD_PLUGIN(
  SlotcarPlugin,
  System,
  SlotcarPlugin::ISystemConfigure,
  SlotcarPlugin::ISystemConfigurePriority,
  SlotcarPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SlotcarPlugin, "slotcar")
