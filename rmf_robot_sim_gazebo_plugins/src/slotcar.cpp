#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rmf_robot_sim_common/utils.hpp>
#include <rmf_robot_sim_common/slotcar_common.hpp>

class SlotcarPlugin : public gazebo::ModelPlugin
{
public:
  SlotcarPlugin();
  ~SlotcarPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void OnUpdate();

private:
  std::unique_ptr<rmf_robot_sim_common::SlotcarCommon> dataPtr;

  gazebo::transport::NodePtr _gazebo_node;
  gazebo::transport::SubscriberPtr _charge_state_sub;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;

  std::array<gazebo::physics::JointPtr, 2> joints;

  std::unordered_set<gazebo::physics::Model*> obstacle_exclusions;

  // Book keeping
  double last_update_time = 0.0;

  void init_obstacle_exclusions();

  std::vector<Eigen::Vector3d> get_obstacle_positions(
    const gazebo::physics::WorldPtr& world);

  void charge_state_cb(ConstSelectionPtr& msg);

  void send_control_signals(const std::pair<double, double>& displacements,
    const double dt, const double target_linear_speed_now,
    const double target_linear_speed_destination,
    const std::optional<double>& max_linear_velocity)
  {
    std::array<double, 2> w_tire;
    for (std::size_t i = 0; i < 2; ++i)
      w_tire[i] = joints[i]->GetVelocity(0);
    auto joint_signals = dataPtr->calculate_joint_control_signals(w_tire,
        displacements, dt, target_linear_speed_now,
        target_linear_speed_destination,
        max_linear_velocity);
    for (std::size_t i = 0; i < 2; ++i)
    {
      joints[i]->SetParam("vel", 0, joint_signals[i]);
      joints[i]->SetParam("fmax", 0, 10000000.0); // TODO(MXG): Replace with realistic torque limit
    }
  }
};

SlotcarPlugin::SlotcarPlugin()
: dataPtr(std::make_unique<rmf_robot_sim_common::SlotcarCommon>())
{
  // Listen for messages that enable/disable charging
  _gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  _gazebo_node->Init();
  _charge_state_sub = _gazebo_node->Subscribe("/charge_state",
      &SlotcarPlugin::charge_state_cb, this);
  // We do rest of initialization during ::Load
}

SlotcarPlugin::~SlotcarPlugin()
{
}

void SlotcarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _model = model;
  dataPtr->set_model_name(_model->GetName());
  dataPtr->read_sdf(sdf);
  gazebo_ros::Node::SharedPtr _ros_node = gazebo_ros::Node::Get(sdf);
  dataPtr->init_ros_node(_ros_node);

  RCLCPP_INFO(
    dataPtr->logger(),
    "Initialising slotcar for %s",
    model->GetName().c_str());

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&SlotcarPlugin::OnUpdate, this));

  joints[0] = _model->GetJoint("joint_tire_left");
  if (!joints[0])
  {
    RCLCPP_ERROR(
      dataPtr->logger(),
      "Could not find tire for [joint_tire_left]");
  }

  joints[1] = _model->GetJoint("joint_tire_right");
  if (!joints[1])
  {
    RCLCPP_ERROR(
      dataPtr->logger(),
      "Could not find tire for [joint_tire_right]");
  }
}

void SlotcarPlugin::charge_state_cb(ConstSelectionPtr& msg)
{
  dataPtr->charge_state_cb(msg->name(), msg->selected());
}

void SlotcarPlugin::init_obstacle_exclusions()
{
  const auto& world = _model->GetWorld();
  obstacle_exclusions.insert(_model.get());
  const auto& all_models = world->Models();
  for (const auto& m : all_models)
  {
    // Object should not be static, be part of infrastructure, or dispensable
    if (!m->IsStatic())
    {
      std::string name = m->GetName();
      std::for_each(name.begin(), name.end(), [](char& c)
        {
          c = ::tolower(c);
        });
      if (name.find("door") != std::string::npos ||
        name.find("lift") != std::string::npos ||
        name.find("dispensable") != std::string::npos)
        obstacle_exclusions.insert(m.get());
    }
  }
}

std::vector<Eigen::Vector3d> SlotcarPlugin::get_obstacle_positions(
  const gazebo::physics::WorldPtr& world)
{
  std::vector<Eigen::Vector3d> obstacle_positions;

  for (const auto& m : world->Models())
  {
    // Object should not be static, not part of obstacle_exclusions,
    // and close than a threshold (checked by common function)
    const auto p_obstacle = m->WorldPose().Pos();
    if (m->IsStatic() == false &&
      obstacle_exclusions.find(m.get()) == obstacle_exclusions.end())
      obstacle_positions.push_back(rmf_plugins_utils::convert_vec(p_obstacle));
  }

  return obstacle_positions;
}

void SlotcarPlugin::OnUpdate()
{
  const auto& world = _model->GetWorld();

  // After initialization once, this set will have at least one exclusion, which
  // is the itself.
  if (obstacle_exclusions.empty())
    init_obstacle_exclusions();

  const double time = world->SimTime().Double();
  const double dt = time - last_update_time;
  last_update_time = time;

  auto pose = _model->WorldPose();
  auto obstacle_positions = get_obstacle_positions(world);

  auto update_result =
    dataPtr->update(rmf_plugins_utils::convert_pose(pose),
      obstacle_positions, time);

  send_control_signals({update_result.v, update_result.w}, dt,
    update_result.target_linear_speed_now,
    update_result.target_linear_speed_destination,
    update_result.max_speed);
}

GZ_REGISTER_MODEL_PLUGIN(SlotcarPlugin)
