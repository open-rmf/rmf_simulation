/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_BUILDING_SIM_COMMON__SLOTCAR_COMMON_HPP
#define RMF_BUILDING_SIM_COMMON__SLOTCAR_COMMON_HPP

#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/pause_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>

namespace rmf_robot_sim_common {

// TODO migrate ign-math-eigen conversions when upgrading to ign-math5

//3rd coordinate is yaw
struct NonHolonomicTrajectory
{
  NonHolonomicTrajectory(const Eigen::Vector2d& _x0, const Eigen::Vector2d& _x1,
    const Eigen::Vector2d& _v1 = Eigen::Vector2d(0, 0),
    bool _turning = false)
  : x0(_x0), x1(_x1),
    v0((x1 - x0).normalized()), v1(_v1),
    turning(_turning)
  {}
  // positions
  Eigen::Vector2d x0;
  Eigen::Vector2d x1;
  // headings
  Eigen::Vector2d v0;
  Eigen::Vector2d v1;

  bool turning = false;
};

// Edit reference of parameter for template type deduction
template<typename IgnQuatT>
inline void convert(const Eigen::Quaterniond& _q, IgnQuatT& quat)
{
  quat.W() = _q.w();
  quat.X() = _q.x();
  quat.Y() = _q.y();
  quat.Z() = _q.z();
}

template<typename IgnVec3T>
inline void convert(const Eigen::Vector3d& _v, IgnVec3T& vec)
{
  vec.X() = _v[0];
  vec.Y() = _v[1];
  vec.Z() = _v[2];
}

template<typename IgnVec3T>
inline Eigen::Vector3d convert_vec(const IgnVec3T& _v)
{
  return Eigen::Vector3d(_v[0], _v[1], _v[2]);
}

template<typename IgnQuatT>
inline Eigen::Quaterniond convert_quat(const IgnQuatT& _q)
{
  Eigen::Quaterniond quat;
  quat.w() = _q.W();
  quat.x() = _q.X();
  quat.y() = _q.Y();
  quat.z() = _q.Z();

  return quat;
}

template<typename IgnPoseT>
inline auto convert(const Eigen::Isometry3d& _tf)
{
  IgnPoseT pose;
  convert(Eigen::Vector3d(_tf.translation()), pose.Pos());
  convert(Eigen::Quaterniond(_tf.linear()), pose.Rot());

  return pose;
}

template<typename IgnPoseT>
inline Eigen::Isometry3d convert_pose(const IgnPoseT& _pose)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = convert_vec(_pose.Pos());
  tf.linear() = Eigen::Matrix3d(convert_quat(_pose.Rot()));

  return tf;
}

typedef struct TrajectoryPoint
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  TrajectoryPoint(const Eigen::Vector3d& _pos, const Eigen::Quaterniond& _quat)
  : pos(_pos), quat(_quat) {}
} TrajectoryPoint;

class SlotcarCommon
{
public:
  SlotcarCommon();

  rclcpp::Logger logger() const;

  template<typename SdfPtrT>
  void read_sdf(SdfPtrT& sdf);

  void set_model_name(const std::string& model_name);

  std::string model_name() const;

  void init_ros_node(const rclcpp::Node::SharedPtr node);

  std::pair<double, double> update(const Eigen::Isometry3d& pose,
    const std::vector<Eigen::Vector3d>& obstacle_positions,
    const double time);

  std::pair<double, double> update_nonholonomic(Eigen::Isometry3d& pose,
    double& target_linear_velocity);

  bool emergency_stop(const std::vector<Eigen::Vector3d>& obstacle_positions,
    const Eigen::Vector3d& current_heading);

  std::array<double, 2> calculate_control_signals(const std::array<double,
    2>& curr_velocities,
    const std::pair<double, double>& displacements,
    const double dt,
    const double target_linear_velocity = 0.0) const;

  std::array<double, 2> calculate_joint_control_signals(
    const std::array<double, 2>& w_tire,
    const std::pair<double, double>& displacements,
    const double dt) const;

  std::array<double, 2> calculate_model_control_signals(
    const std::array<double, 2>& curr_velocities,
    const std::pair<double, double>& displacements,
    const double dt,
    const double target_linear_velocity = 0.0) const;

  void charge_state_cb(const std::string& name, bool selected);

  void publish_robot_state(const double time);

  // steering type constants
  enum class STEERING_TYPE
  {
    DIFF_DRIVE,
    ACKERMANN
  };

  STEERING_TYPE get_steering_type() const;

  bool is_ackermann_steered() const;

private:
  // Paramters needed for power dissipation and charging calculations
  // Default values may be overriden using values from sdf file
  struct PowerParams
  {
    double nominal_voltage = 12; // V
    double nominal_capacity = 24; // Ah
    double charging_current = 2; // A
    double mass = 20;
    double inertia = 10;
    double friction_coefficient = 0.3;
    double nominal_power = 10;
  };

  struct ChargerWaypoint
  {
    double x;
    double y;
    ChargerWaypoint(double x, double y)
    : x(x), y(y)
    {
    }
  };

  // Constants for update rate of tf2 and robot_state topic
  static constexpr float TF2_RATE = 100.0;
  static constexpr float STATE_TOPIC_RATE = 2.0;

  // Initial distance threshold over which a fleet adapter error is reported
  static constexpr float INITIAL_DISTANCE_THRESHOLD = 1.0;

  rclcpp::Node::SharedPtr _ros_node;

  double _last_update_time = 0.0;
  double last_tf2_pub = 0.0;
  double last_topic_pub = 0.0;
  std::size_t _sequence = 0;

  std::vector<Eigen::Isometry3d> trajectory;
  std::size_t _traj_wp_idx = 0;
  std::vector<NonHolonomicTrajectory> nonholonomic_trajectory;
  std::size_t _nonholonomic_traj_idx = 0;

  rmf_fleet_msgs::msg::PauseRequest pause_request;

  std::vector<rclcpp::Time> _hold_times;

  std::mutex _mutex;
  std::mutex _ackmann_path_req_mutex;

  std::string _model_name;
  bool _emergency_stop = false;
  bool _adapter_error = false;

  bool _initialized_pose = false; // True if at least 1 call to update() has been made
  Eigen::Isometry3d _old_pose; // Pose at previous time step
  // Assumes robot is stationary upon initialization
  Eigen::Vector3d _old_lin_vel = Eigen::Vector3d::Zero(); // Linear velocity at previous time step
  double _old_ang_vel = 0.0; // Angular velocity at previous time step
  Eigen::Isometry3d _pose; // Pose at current time step
  int _rot_dir = 1; // Current direction of rotation

  std::unordered_map<std::string, double> _level_to_elevation;
  bool _initialized_levels = false;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf2_broadcaster;
  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_pub;

  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr _traj_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::PauseRequest>::SharedPtr _pause_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr _mode_sub;
  rclcpp::Subscription<rmf_building_map_msgs::msg::BuildingMap>::SharedPtr
    _building_map_sub;

  rmf_fleet_msgs::msg::RobotMode _current_mode;

  STEERING_TYPE _steering_type = STEERING_TYPE::DIFF_DRIVE;

  std::string _current_task_id;
  std::vector<rmf_fleet_msgs::msg::Location> _remaining_path;

  // Vehicle dynamic constants
  // TODO(MXG): Consider fetching these values from model data
  // Radius of a tire
  double _tire_radius = 0.1;
  // Distance of a tire from the origin
  double _base_width = 0.52;

  double _nominal_drive_speed = 0.5;         // nominal robot velocity (m/s)
  double _nominal_drive_acceleration = 0.05; // nominal robot forward acceleration (m/s^2)
  double _max_drive_acceleration = 0.1;      // maximum robot forward acceleration (m/s^2)

  double _nominal_turn_speed = M_PI / 8.0;         // nominal robot turning speed (half a rotation per 8 seconds)
  double _nominal_turn_acceleration = M_PI / 10.0; // nominal robot turning acceleration (rad/s^2)

  double _max_turn_acceleration = M_PI; // maximum robot turning acceleration (rad/s^2)

  double _stop_distance = 1.0;
  double _stop_radius = 1.0;

  double _min_turning_radius = -1.0; // minimum turning radius, will use a formula if negative
  double _turning_right_angle_mul_offset = 1.0; // if _min_turning_radius is computed, this value multiplies it

  bool _reversible = true; // true if the robot can drive backwards

  PowerParams _params;
  bool _enable_charge = true;
  bool _enable_instant_charge = false;
  bool _enable_drain = true;
  // Used for comparing with name argument of charge_state_cb to identify button selected
  const std::string _enable_charge_str = "_enable_charge";
  const std::string _enable_instant_charge_str = "_enable_instant_charge";
  const std::string _enable_drain_str = "_enable_drain";
  const double _soc_max = 1.0;
  double _soc = _soc_max;
  std::unordered_map<std::string, std::vector<ChargerWaypoint>>
  _charger_waypoints;
  // Straight line distance to charging waypoint within which charging can occur
  static constexpr double _charger_dist_thres = 0.3;

  bool _docking = false;

  std::string get_level_name(const double z) const;

  double compute_change_in_rotation(
    const Eigen::Vector3d& heading_vec,
    const Eigen::Vector3d& dpos,
    const Eigen::Vector3d* traj_vec = nullptr,
    double* const dir = nullptr) const;

  void publish_tf2(const rclcpp::Time& t);

  void publish_state_topic(const rclcpp::Time& t);

  bool path_request_valid(
    const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  void path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  void ackmann_path_request_cb(
    const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  void pause_request_cb(const rmf_fleet_msgs::msg::PauseRequest::SharedPtr msg);

  void mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);

  void map_cb(const rmf_building_map_msgs::msg::BuildingMap::SharedPtr msg);

  bool near_charger(const Eigen::Isometry3d& pose) const;

  double compute_charge(const double run_time) const;

  double compute_discharge(
    const Eigen::Vector3d lin_vel, const double ang_vel,
    const Eigen::Vector3d lin_acc, const double ang_acc,
    const double run_time) const;
};

template<typename SdfPtrT, typename valueT>
bool get_element_val_if_present(
  SdfPtrT& _sdf,
  const std::string& _element_name,
  valueT& _val)
{
  if (!_sdf->HasElement(_element_name))
  {
    return false;
  }
  _val = _sdf->template Get<valueT>(_element_name);
  return true;
}

template<typename SdfPtrT>
void SlotcarCommon::read_sdf(SdfPtrT& sdf)
{
  std::string steering_type;
  get_element_val_if_present<SdfPtrT, std::string>(sdf, "steering",
    steering_type);

  if (steering_type == "ackermann")
    _steering_type = STEERING_TYPE::ACKERMANN;
  else if (steering_type == "diff_drive")
    _steering_type = STEERING_TYPE::DIFF_DRIVE;

  RCLCPP_INFO(
    logger(),
    "Vehicle uses %s steering", steering_type.c_str());

  get_element_val_if_present<SdfPtrT, double>(sdf, "nominal_drive_speed",
    this->_nominal_drive_speed);
  RCLCPP_INFO(
    logger(),
    "Setting nominal drive speed to: %f",
    _nominal_drive_speed);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "nominal_drive_acceleration", this->_nominal_drive_acceleration);
  RCLCPP_INFO(
    logger(),
    "Setting nominal drive acceleration to: %f",
    _nominal_drive_acceleration);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "max_drive_acceleration", this->_max_drive_acceleration);
  RCLCPP_INFO(
    logger(),
    "Setting max drive acceleration to: %f",
    _max_drive_acceleration);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "nominal_turn_speed", this->_nominal_turn_speed);
  RCLCPP_INFO(
    logger(),
    "Setting nominal turn speed to: %f",
    _nominal_turn_speed);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "nominal_turn_acceleration", this->_nominal_turn_acceleration);
  RCLCPP_INFO(
    logger(),
    "Setting nominal turn acceleration to: %f",
    _nominal_turn_acceleration);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "max_turn_acceleration", this->_max_turn_acceleration);
  RCLCPP_INFO(
    logger(),
    "Setting max turn acceleration to: %f",
    _max_turn_acceleration);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "min_turning_radius", this->_min_turning_radius);
  RCLCPP_INFO(
    logger(),
    "Setting minimum turning radius to: %f",
    _min_turning_radius);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "turning_right_angle_mul_offset", this->_turning_right_angle_mul_offset);
  RCLCPP_INFO(
    logger(),
    "Setting turning right angle multiplier offset to: %f",
    _turning_right_angle_mul_offset);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "stop_distance", this->_stop_distance);
  RCLCPP_INFO(logger(), "Setting stop distance to: %f", _stop_distance);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "stop_radius", this->_stop_radius);
  RCLCPP_INFO(logger(), "Setting stop radius to: %f", _stop_radius);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "tire_radius", this->_tire_radius);
  RCLCPP_INFO(logger(), "Setting tire radius to: %f", _tire_radius);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "base_width", this->_base_width);
  RCLCPP_INFO(logger(), "Setting base width to: %f", _base_width);

  get_element_val_if_present<SdfPtrT, bool>(sdf,
    "reversible", this->_reversible);
  RCLCPP_INFO(logger(), "Setting reversible to: %d", _reversible);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "nominal_voltage", this->_params.nominal_voltage);
  RCLCPP_INFO(
    logger(),
    "Setting nominal voltage to: %f",
    _params.nominal_voltage);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "nominal_capacity", this->_params.nominal_capacity);
  RCLCPP_INFO(
    logger(),
    "Setting nominal capacity to: %f",
    _params.nominal_capacity);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "charging_current", this->_params.charging_current);
  RCLCPP_INFO(
    logger(),
    "Setting charging current to: %f",
    _params.charging_current);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "mass", this->_params.mass);
  RCLCPP_INFO(logger(), "Setting mass to: %f", _params.mass);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "inertia", this->_params.inertia);
  RCLCPP_INFO(
    logger(),
    "Setting inertia to: %f",
    _params.inertia);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "friction_coefficient", this->_params.friction_coefficient);
  RCLCPP_INFO(
    logger(),
    "Setting friction coefficient to: %f",
    _params.friction_coefficient);

  get_element_val_if_present<SdfPtrT, double>(sdf,
    "nominal_power", this->_params.nominal_power);
  RCLCPP_INFO(
    logger(),
    "Setting nominal power to: %f",
    _params.nominal_power);

  // Charger Waypoint coordinates are in child element of top level world element
  if (sdf->GetParent() && sdf->GetParent()->GetParent())
  {
    auto parent = sdf->GetParent()->GetParent();
    if (parent->HasElement("rmf_charger_waypoints"))
    {
      auto waypoints = parent->GetElement("rmf_charger_waypoints");
      if (waypoints->HasElement("rmf_vertex"))
      {
        auto waypoint = waypoints->GetElement("rmf_vertex");
        while (waypoint)
        {
          if (waypoint->HasAttribute("x") && waypoint->HasAttribute("y") &&
            waypoint->HasAttribute("level"))
          {
            std::string lvl_name;
            double x, y;
            waypoint->GetAttribute("x")->Get(x);
            waypoint->GetAttribute("y")->Get(y);
            waypoint->GetAttribute("level")->Get(lvl_name);
            _charger_waypoints[lvl_name].push_back(ChargerWaypoint(x, y));
          }
          waypoint = waypoint->GetNextElement("rmf_vertex");
        }
      }
    }
    else
    {
      RCLCPP_INFO(logger(), "No charger waypoints found.");
    }
  }

  RCLCPP_INFO(logger(), "Setting name to: %s", _model_name.c_str());
}
} // namespace rmf_robot_sim_common

#endif // RMF_BUILDING_SIM_COMMON__SLOTCAR_COMMON_HPP
