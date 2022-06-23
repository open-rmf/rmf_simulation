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

#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmf_fleet_msgs/msg/destination_request.hpp>
#include <rclcpp/logger.hpp>

#include <rmf_robot_sim_common/utils.hpp>
#include <rmf_robot_sim_common/slotcar_common.hpp>


static double compute_yaw(const Eigen::Isometry3d& pose)
{
  auto quat = Eigen::Quaterniond(pose.linear());
  // Taken from ignition math quaternion Euler()
  double yaw = std::atan2(2 * (quat.x()*quat.y() + quat.w()*quat.z()),
      (quat.w() * quat.w()) + (quat.x() * quat.x()) - (quat.y() * quat.y()) -
      (quat.z() * quat.z()));
  return yaw;
}

// Computes change in yaw angle from old_pose to pose
static double compute_yaw(const Eigen::Isometry3d& pose,
  const Eigen::Isometry3d& old_pose, int rot_dir)
{
  const double yaw = compute_yaw(pose);
  const double old_yaw = compute_yaw(old_pose);

  double disp = yaw - old_yaw;
  double eps = 0.01;
  // Account for edge cases where the robot rotates past -PI rad to PI rad, or vice versa
  if (rot_dir > 0 && yaw < (old_yaw - eps))
  {
    disp = (M_PI - old_yaw) + (yaw + M_PI);
  }
  else if (rot_dir < 0 && yaw > (old_yaw + eps))
  {
    disp = (M_PI - yaw) + (old_yaw + M_PI);
  }
  return disp;
}

static Eigen::Vector3d compute_heading(const Eigen::Isometry3d& pose)
{
  double yaw = compute_yaw(pose);
  return Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
}

inline static Eigen::Vector3d compute_dist(const Eigen::Isometry3d& old_pos,
  const Eigen::Isometry3d& new_pos)
{
  return new_pos.translation() - old_pos.translation();
}

inline static auto compute_dpos(const Eigen::Isometry3d& target,
  const Eigen::Isometry3d& actual)
{
  Eigen::Vector3d dpos(compute_dist(actual, target));
  dpos(2) = 0.0;
  return dpos;
}

double compute_friction_energy(
  const double f,
  const double m,
  const double v,
  const double dt)
{
  const double g = 9.81; // ms-1
  return f * m * g * v * dt;
}

// Given a line segment from point1 to point2, and a circle centred at (cx, cy),
// returns the points of intersection.
static std::vector<Eigen::Vector2d> line_circle_intersections(
  const Eigen::Vector2d point1,
  const Eigen::Vector2d point2,
  const double cx,
  const double cy,
  const double radius
)
{
  double dx, dy, A, B, C, det, t;
  std::vector<Eigen::Vector2d> intersections;

  dx = point2(0) - point1(0);
  dy = point2(1) - point1(1);

  // From the equations for the line segment from point1 to point2 with parameter t,
  // and the circle, form the quadratic equation for t.
  // Then A, B and C are the coefficients of the quadratic equation.
  A = dx * dx + dy * dy;
  B = 2 * (dx * (point1(0) - cx) + dy * (point1(1) - cy));
  C = (point1(0) - cx) * (point1(0) - cx) +
    (point1(1) - cy) * (point1(1) - cy) -
    radius * radius;

  det = B * B - 4 * A * C;

  if ((A <= 1e-7) || (det < 0))
  {
    // No real solutions.
    return intersections;
  }
  else if (det < 1e-7)
  {
    // One solution.
    t = -B / (2 * A);
    intersections.push_back(Eigen::Vector2d(point1(0) + t * dx,
      point1(1) + t * dy));
  }
  else
  {
    // Two solutions.
    t = (-B + std::sqrt(det)) / (2 * A);
    intersections.push_back(Eigen::Vector2d(point1(0) + t * dx,
      point1(1) + t * dy));
    t = (-B - std::sqrt(det)) / (2 * A);
    intersections.push_back(Eigen::Vector2d(point1(0) + t * dx,
      point1(1) + t * dy));
  }

  return intersections;
}

// Returns the point on the line segment from A to B that is
// closest to point P.
static Eigen::Vector2d get_closest_point_on_line_segment(
  Eigen::Vector2d A,
  Eigen::Vector2d B,
  Eigen::Vector2d P)
{
  Eigen::Vector2d AP = P - A;
  Eigen::Vector2d AB = B - A;

  double mag = AB.dot(AP) / AB.norm();  // Magnitude of projection of AP on AB
  if (mag < 0)
  {
    return A;
  }
  else if (mag > AB.norm())
  {
    return B;
  }
  else
  {
    return A + (AB.normalized() * mag);
  }
}

using SlotcarCommon = rmf_robot_sim_common::SlotcarCommon;

SlotcarCommon::SlotcarCommon()
{
  // Make sure we initialize this message to TYPE_RESUME, or else the robot
  // might just sit and wait around unintentionally.
  pause_request = rmf_fleet_msgs::build<rmf_fleet_msgs::msg::PauseRequest>()
    .fleet_name("")
    .robot_name("")
    .mode_request_id(0)
    .type(rmf_fleet_msgs::msg::PauseRequest::TYPE_RESUME)
    .at_checkpoint(0);
}

rclcpp::Logger SlotcarCommon::logger() const
{
  return rclcpp::get_logger("slotcar_" + _model_name);
}

void SlotcarCommon::set_model_name(const std::string& model_name)
{
  _model_name = model_name;
}

std::string SlotcarCommon::model_name() const
{
  return _model_name;
}

void SlotcarCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _ros_node = std::move(node);

  _tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(_ros_node);

  _robot_state_pub =
    _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
    "/robot_state", 10);

  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _building_map_sub =
    _ros_node->create_subscription<rmf_building_map_msgs::msg::BuildingMap>(
    "/map",
    qos_profile,
    std::bind(&SlotcarCommon::map_cb, this, std::placeholders::_1));

  _traj_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::PathRequest>(
    "/robot_path_requests",
    10,
    std::bind(&SlotcarCommon::path_request_cb, this, std::placeholders::_1));

  using PauseRequest = rmf_fleet_msgs::msg::PauseRequest;
  _pause_sub = _ros_node->create_subscription<PauseRequest>(
    "/robot_pause_requests",
    10,
    std::bind(&SlotcarCommon::pause_request_cb, this, std::placeholders::_1));

  _mode_sub = _ros_node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "/robot_mode_requests",
    10,
    std::bind(&SlotcarCommon::mode_request_cb, this, std::placeholders::_1));
}

bool SlotcarCommon::path_request_valid(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  // Request is for another robot
  if (msg->robot_name != _model_name)
    return false;

  // Repeated task request
  if (msg->task_id == _current_task_id)
  {
    RCLCPP_INFO(
      logger(), "%s already received task [%s] -- continuing as normal",
      _current_task_id.c_str(), _model_name.c_str());
    return false;
  }

  // Empty task request
  if (msg->path.size() == 0)
  {
    RCLCPP_WARN(logger(), "%s received a path with no waypoints",
      _model_name.c_str());
    return false;
  }
  return true;
}

void SlotcarCommon::path_request_cb(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  if (path_request_valid(msg) == false)
    return;
  std::lock_guard<std::mutex> lock(_mutex);

  const auto old_path = _remaining_path;

  RCLCPP_DEBUG(
    logger(),
    "%s received a path request with %d waypoints",
    _model_name.c_str(), (int)msg->path.size());

  // Reset this if we aren't at the final waypoint
  trajectory.resize(msg->path.size());
  _hold_times.resize(msg->path.size());
  for (size_t i = 0; i < msg->path.size(); ++i)
  {
    Eigen::Vector3d v3(
      msg->path[i].x,
      msg->path[i].y,
      0);

    Eigen::Vector3d yaw_euler(
      0,
      0,
      msg->path[i].yaw);

    Eigen::Quaterniond quat(
      Eigen::AngleAxisd(msg->path[i].yaw, Eigen::Vector3d::UnitZ()));
    trajectory.at(i).pose.translation() = v3;
    trajectory.at(i).pose.linear() = Eigen::Matrix3d(quat);
    if (msg->path[i].obey_approach_speed_limit &&
      msg->path[i].approach_speed_limit > 0.0)
    {
      trajectory.at(i).approach_speed_limit = msg->path[i].approach_speed_limit;
    }

    _hold_times.at(i) = msg->path[i].t;
  }
  _remaining_path = msg->path;
  _traj_wp_idx = 0;

  _current_task_id = msg->task_id;
  _adapter_error = false;

  const double initial_dist =
    compute_dpos(trajectory.front().pose, _pose).norm();

  if (this->_steering_type == SteeringType::DIFF_DRIVE &&
    initial_dist > INITIAL_DISTANCE_THRESHOLD)
  {
    trajectory.clear();
    trajectory.push_back(_pose);

    _hold_times.clear();
    _hold_times.push_back(rclcpp::Time((int64_t)0, RCL_ROS_TIME));

    // We'll stick with the old path when an adapter error happens so that the
    // fleet adapter knows where the robot currently is along its previous path.
    _remaining_path = old_path;

    _adapter_error = true;
  }
  else
  {
    trajectory.erase(trajectory.begin());
    _hold_times.erase(_hold_times.begin());
    _remaining_path.erase(_remaining_path.begin());
  }

  if (_path_request_callback)
    _path_request_callback(msg);
}

void SlotcarCommon::pause_request_cb(
  const rmf_fleet_msgs::msg::PauseRequest::SharedPtr msg)
{
  if (msg->robot_name != _model_name)
    return;

  std::lock_guard<std::mutex> lock(_mutex);
  pause_request = *msg;
}

std::array<double, 2> SlotcarCommon::calculate_control_signals(
  const std::array<double, 2>& curr_velocities,
  const std::pair<double, double>& displacements,
  const double dt,
  const double linear_speed_target_now,
  const double linear_speed_target_destination,
  const std::optional<double>& linear_speed_limit) const
{
  const double v_robot = curr_velocities[0];
  const double w_robot = curr_velocities[1];

  const double max_lin_vel = linear_speed_limit.has_value() ?
    linear_speed_limit.value() : _nominal_drive_speed;
  rmf_plugins_utils::MotionParams drive_params {
    max_lin_vel,
    _max_drive_acceleration,
    _nominal_drive_acceleration,
    0.01,
    10000000.0};
  const double v_target = rmf_plugins_utils::compute_desired_rate_of_change(
    displacements.first,
    v_robot,
    linear_speed_target_now,
    linear_speed_target_destination,
    drive_params,
    dt);

  rmf_plugins_utils::MotionParams turn_params {
    _nominal_turn_speed,
    _max_turn_acceleration,
    _nominal_turn_acceleration,
    0.01,
    10000000.0};
  const double w_target = rmf_plugins_utils::compute_desired_rate_of_change(
    displacements.second,
    w_robot,
    _nominal_turn_speed,
    0.0,
    turn_params,
    dt);

  return std::array<double, 2>{v_target, w_target};
}

std::array<double, 2> SlotcarCommon::calculate_joint_control_signals(
  const std::array<double, 2>& w_tire,
  const std::pair<double, double>& displacements,
  const double dt,
  const double linear_speed_target_now,
  const double linear_speed_target_destination,
  const std::optional<double>& linear_speed_limit) const
{
  std::array<double, 2> curr_velocities;
  curr_velocities[0] = (w_tire[0] + w_tire[1]) * _tire_radius / 2.0;
  curr_velocities[1] = (w_tire[1] - w_tire[0]) * _tire_radius / _base_width;

  std::array<double, 2> new_velocities = calculate_control_signals(
    curr_velocities, displacements, dt, linear_speed_target_now,
    linear_speed_target_destination,
    linear_speed_limit);

  std::array<double, 2> joint_signals;
  for (std::size_t i = 0; i < 2; ++i)
  {
    const double yaw_sign = i == 0 ? -1.0 : 1.0;
    joint_signals[i] = (new_velocities[0] / _tire_radius) + (yaw_sign *
      new_velocities[1] * _base_width / (2.0 * _tire_radius));
  }
  return joint_signals;
}

std::string to_str(uint32_t type)
{
  if (rmf_fleet_msgs::msg::PauseRequest::TYPE_RESUME == type)
    return "resume";
  else if (rmf_fleet_msgs::msg::PauseRequest::TYPE_PAUSE_IMMEDIATELY == type)
    return "pause immediately";
  else if (rmf_fleet_msgs::msg::PauseRequest::TYPE_PAUSE_AT_CHECKPOINT == type)
    return "pause at checkpoint";

  return "UNKNOWN: " + std::to_string(type) + "??";
}

SlotcarCommon::UpdateResult SlotcarCommon::update(const Eigen::Isometry3d& pose,
  const std::vector<Eigen::Vector3d>& obstacle_positions,
  const double time)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _pose = pose;
  publish_robot_state(time);

  switch (this->_steering_type)
  {
    case SteeringType::DIFF_DRIVE:
      return update_diff_drive(obstacle_positions, time);
    case SteeringType::ACKERMANN:
      // TODO(anyone) use obstacle_positions for emergency stop for ackermann
      return update_ackermann(obstacle_positions, time);
    default:
      return UpdateResult();
  }
}

// First value of par is x_target, second is yaw_target
SlotcarCommon::UpdateResult SlotcarCommon::update_diff_drive(
  const std::vector<Eigen::Vector3d>& obstacle_positions,
  const double time)
{
  UpdateResult result;
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};
  double dt = time - _last_update_time;
  _last_update_time = time;

  // Update battery state of charge
  if (_initialized_pose)
  {
    const Eigen::Vector3d dist = compute_dpos(_old_pose, _pose); // Ignore movement along z-axis
    const Eigen::Vector3d lin_vel = dist / dt;
    double ang_disp = compute_yaw(_pose, _old_pose, _rot_dir);
    const double ang_vel = ang_disp / dt;

    // Try charging battery
    double eps = 0.01;
    bool stationary = lin_vel.norm() < eps && std::abs(ang_vel) < eps;
    bool in_charger_vicinity = near_charger(_pose);
    if (stationary && in_charger_vicinity)
    {
      if (_enable_instant_charge)
      {
        _soc = _soc_max;
      }
      else if (_enable_charge)
      {
        _soc += compute_charge(dt);
        _soc = std::min(_soc_max, _soc);
      }
    }
    // Discharge battery
    if (_enable_drain && !in_charger_vicinity)
    {
      const Eigen::Vector3d lin_acc = (lin_vel - _old_lin_vel) / dt;
      const double ang_acc = (ang_vel - _old_ang_vel) / dt;
      _soc -= compute_discharge(lin_vel, ang_vel, lin_acc, ang_acc, dt);
      _soc = std::max(0.0, _soc);
    }
    // Update mode
    if (stationary && in_charger_vicinity &&
      (_enable_instant_charge || _enable_charge))
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING;
    }
    else if (_docking)
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_DOCKING;
    }
    else if (stationary)
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    }
    else
    {
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
    }
    _old_lin_vel = lin_vel;
    _old_ang_vel = ang_vel;
  }
  _old_pose = _pose;
  _initialized_pose = true;

  if (trajectory.empty())
    return result;

  Eigen::Vector3d current_heading = compute_heading(_pose);

  if (_traj_wp_idx < trajectory.size())
  {
    const auto& approach_speed_limit =
      trajectory.at(_traj_wp_idx).approach_speed_limit;
    if (approach_speed_limit.has_value())
      result.max_speed = approach_speed_limit.value();
    const Eigen::Vector3d dpos = compute_dpos(
      trajectory.at(_traj_wp_idx).pose, _pose);

    if (_hold_times.size() != trajectory.size())
    {
      throw std::runtime_error(
              "Mismatch between trajectory size ["
              + std::to_string(trajectory.size()) + "] and holding time size ["
              + std::to_string(_hold_times.size()) + "]");
    }

    auto dpos_mag = dpos.norm();
    // TODO(MXG): Some kind of crazy nonsense bug is somehow altering the
    // clock type value for the _hold_times. I don't know where this could
    // possibly be happening, but I suspect it must be caused by undefined
    // behavior. For now we deal with this by explicitly setting the clock type.
    const auto hold_time =
      rclcpp::Time(_hold_times.at(_traj_wp_idx), RCL_ROS_TIME);

    bool close_enough = (dpos_mag < 0.02);
    if (_was_rotating) // Accomodate slight drift when rotating on the spot
      close_enough = (dpos_mag < 0.04);

    const bool checkpoint_pause =
      pause_request.type == pause_request.TYPE_PAUSE_AT_CHECKPOINT
      && pause_request.at_checkpoint <= _remaining_path.front().index;
    const bool immediate_pause =
      pause_request.type == pause_request.TYPE_PAUSE_IMMEDIATELY;
    const bool pause = checkpoint_pause || immediate_pause;

    const bool hold = now < hold_time;

    const bool rotate_towards_next_target = close_enough && (hold || pause);
    _was_rotating = rotate_towards_next_target;

    if (rotate_towards_next_target)
    {
      if (_traj_wp_idx+1 < trajectory.size())
      {
        const auto dpos_next =
          compute_dpos(trajectory.at(_traj_wp_idx+1).pose, _pose);

        const auto goal_heading =
          compute_heading(trajectory.at(_traj_wp_idx+1).pose);

        double dir = 1.0;
        result.w = compute_change_in_rotation(
          current_heading, dpos_next, &goal_heading, &dir);

        if (dir < 0.0)
          current_heading *= -1.0;
      }
      else
      {
        const auto goal_heading =
          compute_heading(trajectory.at(_traj_wp_idx).pose);
        result.w = compute_change_in_rotation(
          current_heading, goal_heading);
      }
      result.target_linear_speed_now = 0.0;
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_PAUSED;
    }
    else if (close_enough)
    {
      _traj_wp_idx++;
      if (_remaining_path.empty())
        return result;

      _remaining_path.erase(_remaining_path.begin());
      RCLCPP_DEBUG(logger(),
        "%s reached waypoint %ld/%d",
        _model_name.c_str(),
        _traj_wp_idx,
        (int)trajectory.size());
      if (_traj_wp_idx == trajectory.size())
      {
        RCLCPP_DEBUG(
          logger(),
          "%s reached goal -- rotating to face target",
          _model_name.c_str());
      }
    }

    if (!rotate_towards_next_target && _traj_wp_idx < trajectory.size())
    {
      const double d_yaw_tolerance = 5.0 * M_PI / 180.0;
      auto goal_heading = compute_heading(trajectory.at(_traj_wp_idx).pose);
      double dir = 1.0;
      result.w =
        compute_change_in_rotation(current_heading, dpos, &goal_heading, &dir);
      if (dir < 0.0)
        current_heading *= -1.0;

      // If d_yaw is less than a certain tolerance (i.e. we don't need to spin
      // too much), then we'll include the forward velocity. Otherwise, we will
      // only spin in place until we are oriented in the desired direction.
      result.v = std::abs(result.w) <
        d_yaw_tolerance ? dir * dpos_mag : 0.0;
      if (result.v != 0.0)
      {
        result.target_linear_speed_now = _nominal_drive_speed;
      }
      result.target_linear_speed_destination = 0.0;
    }
  }
  else
  {
    const auto goal_heading = compute_heading(trajectory.back().pose);
    result.w = compute_change_in_rotation(
      current_heading,
      goal_heading);

    result.v = 0.0;
  }

  const bool immediate_pause =
    pause_request.type == pause_request.TYPE_PAUSE_IMMEDIATELY;

  const bool stop =
    immediate_pause || emergency_stop(obstacle_positions, current_heading);

  if (immediate_pause)
  {
    // If we are required to immediately pause, report that we are in paused
    // mode
    _current_mode.mode = _current_mode.MODE_PAUSED;
  }
  else if (stop)
  {
    // If we are not required to pause but we are being forced to emergency stop
    // because of an obstacle, report that we are in waiting mode.
    _current_mode.mode = _current_mode.MODE_WAITING;
  }

  if (stop)
  {
    // Allow spinning but not translating
    result.v = 0.0;
  }

  _rot_dir = result.w >= 0 ? 1 : -1;
  return result;
}

SlotcarCommon::UpdateResult SlotcarCommon::update_ackermann(
  const std::vector<Eigen::Vector3d>& /*obstacle_positions*/,
  const double time)
{
  UpdateResult result;
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};
  double dt = time - _last_update_time;
  _last_update_time = time;

  if (_initialized_pose)
  {
    const Eigen::Vector3d dist = compute_dpos(_old_pose, _pose); // Ignore movement along z-axis
    const Eigen::Vector3d lin_vel = dist / dt;
    double ang_disp = compute_yaw(_pose, _old_pose, _rot_dir);
    const double ang_vel = ang_disp / dt;

    const double eps = 0.01;
    bool stationary = lin_vel.norm() < eps && std::abs(ang_vel) < eps;

    if (stationary)
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    else
      _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;

    _old_lin_vel = lin_vel;
    _old_ang_vel = ang_vel;
  }
  _old_pose = _pose;
  _initialized_pose = true;

  if (trajectory.empty())
    return result;

  Eigen::Vector3d current_heading = compute_heading(_pose);

  if (_traj_wp_idx < trajectory.size())
  {
    const auto& limit = trajectory.at(_traj_wp_idx).approach_speed_limit;
    if (limit.has_value())
      result.max_speed = limit.value();
    else
      result.max_speed = _nominal_drive_speed;

    Eigen::Vector3d dpos = compute_dpos(
      trajectory.at(_traj_wp_idx).pose, _pose);

    auto dpos_mag = dpos.norm();

    // The lines between waypoints form the path.
    // Drive towards the nearest point on it that is
    // one lookahead distance away.
    double close_enough_threshold = _lookahead_distance;

    if (_traj_wp_idx == trajectory.size() - 1)
    {
      // At the last waypoint, stop more closely.
      close_enough_threshold = 1.0;
    }

    if (dpos_mag < close_enough_threshold)
    {
      _traj_wp_idx++;

      _remaining_path.erase(_remaining_path.begin());
      RCLCPP_DEBUG(logger(),
        "%s reached waypoint %ld/%d",
        _model_name.c_str(),
        _traj_wp_idx,
        (int)trajectory.size());
      if (_traj_wp_idx == trajectory.size())
      {
        RCLCPP_DEBUG(
          logger(),
          "%s reached goal",
          _model_name.c_str());
      }
      else
      {
        dpos = compute_dpos(trajectory.at(_traj_wp_idx).pose, _pose);
        dpos_mag = dpos.norm();
      }
    }

    if (_traj_wp_idx < trajectory.size())
    {
      Eigen::Vector2d target; // Target point to drive towards

      if ((_traj_wp_idx == trajectory.size() - 1) &&
        dpos_mag < _lookahead_distance)
      {
        // When near the last waypoint, directly head towards it.
        target = trajectory.at(_traj_wp_idx).pose.translation().head<2>();
      }
      else
      {
        // Otherwise, head towards the closest point on the path which
        // is at least one lookahead distance away from the vehicle.

        // points A and B represent the start and end of the current path segment.
        Eigen::Vector2d point_A;
        if (_traj_wp_idx == 0)
        {
          point_A = _pose.translation().head<2>();
        }
        else
        {
          point_A = trajectory.at(_traj_wp_idx-1).pose.translation().head<2>();
        }
        auto point_B = trajectory.at(_traj_wp_idx).pose.translation().head<2>();

        // Find the intersection of the circle around the vehicle
        // with the path segment.
        auto intersections = line_circle_intersections(
          point_A,
          point_B,
          _pose.translation()(0),
          _pose.translation()(1),
          _lookahead_distance
        );

        if (intersections.size() == 0)
        {
          // No intersections; head towards closest point on the path segment.
          auto point_P = _pose.translation().head<2>();
          target = get_closest_point_on_line_segment(point_A, point_B, point_P);
        }
        else if (intersections.size() == 1)
        {
          target = intersections.at(0);
        }
        else if (intersections.size() == 2)
        {
          // Select the intersection closer to B.
          auto distance_0 = (point_B.head<2>() - intersections.at(0)).norm();
          auto distance_1 = (point_B.head<2>() - intersections.at(1)).norm();
          target = intersections.at(distance_0 < distance_1 ? 0 : 1);
        }
      }

      _lookahead_point = Eigen::Vector3d(target(0), target(1),
          trajectory.at(_traj_wp_idx).pose.translation()(2));

      Eigen::Vector3d d_target = _lookahead_point - _pose.translation();

      result.v = d_target.norm();

      auto goal_heading = compute_heading(trajectory.at(_traj_wp_idx).pose);
      double dir = 1.0;
      result.w = compute_change_in_rotation(
        current_heading, d_target, &goal_heading, &dir);

      // As turning yaw increases, slow down more.
      double turning = fabs(result.w) / M_PI;
      double slowdown = std::min(0.8, turning);   // Minimum speed 20%
      result.target_linear_speed_now =
        std::min(result.max_speed.value(), _nominal_drive_speed) *
        (1 - slowdown);
      result.target_linear_speed_destination = result.target_linear_speed_now;

      if (_traj_wp_idx == trajectory.size() - 1 &&
        dpos_mag < _lookahead_distance)
      {
        // if near the last waypoint, slow to a stop nicely
        result.target_linear_speed_destination = 0;
      }
    }
  }
  else
  {
    _current_task_id = "";
    result.w = 0.0;
    result.v = 0.0;
  }

  _rot_dir = result.w >= 0 ? 1 : -1;
  return result;
}

bool SlotcarCommon::emergency_stop(
  const std::vector<Eigen::Vector3d>& obstacle_positions,
  const Eigen::Vector3d& current_heading)
{
  const Eigen::Vector3d stop_zone =
    _pose.translation() + _stop_distance * current_heading;

  bool need_to_stop = false;
  for (const auto& obstacle_pos : obstacle_positions)
  {
    const double dist = (obstacle_pos - stop_zone).norm();
    if (dist < _stop_radius)
    {
      need_to_stop = true;
      break;
    }
  }

  if (need_to_stop != _emergency_stop)
  {
    _emergency_stop = need_to_stop;
    // TODO flush logger here
    // TODO get collision object name
    if (need_to_stop)
      RCLCPP_INFO_STREAM(logger(), "Stopping [" << _model_name <<
          "] to avoid a collision");
    else
      RCLCPP_INFO_STREAM(logger(), "No more obstacles; resuming course for [" <<
          _model_name << "]");
  }

  return _emergency_stop;
}

std::string SlotcarCommon::get_level_name(const double z)
{
  if (!_initialized_levels)
    return "";
  for (auto it = _level_to_elevation.begin(); it != _level_to_elevation.end();
    ++it)
  {
    const double disp = std::abs(it->second - z);
    if (disp < 0.1)
    {
      _last_known_level = it->first;
      return _last_known_level;
    }
  }
  // Robot is transitioning between levels so return last known level
  return _last_known_level;
}

double SlotcarCommon::compute_change_in_rotation(
  const Eigen::Vector3d& heading_vec,
  const Eigen::Vector3d& dpos,
  const Eigen::Vector3d* traj_vec,
  double* const dir) const
{
  if (dpos.norm() < 1e-3)
  {
    // We're right next to the waypoint, so we don't really need any heading
    // to reach it.
    return 0.0;
  }

  Eigen::Vector3d target = dpos;
  // If a traj_vec is provided and slotcar is reversible, of the two possible
  // headings (dpos/-dpos), choose the one closest to traj_vec
  if (traj_vec && _reversible)
  {
    const double dot = traj_vec->dot(dpos);
    target = dot < 0 ? -dpos : dpos;
    // dir is negative if slotcar will need to reverse to go towards target
    if (dir)
    {
      *dir = dot < 0 ? -1.0 : 1.0;
    }
  }

  const auto cross = heading_vec.cross(target);
  const double direction = cross(2) < 0.0 ? -1.0 : 1.0;
  const double denom = heading_vec.norm() * target.norm();
  const double d_yaw = direction * std::asin(cross.norm() / denom);

  return d_yaw;
}

void SlotcarCommon::publish_robot_state(const double time)
{
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
    static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
  const rclcpp::Time ros_time{t_sec, t_nsec, RCL_ROS_TIME};
  if ((time - last_tf2_pub) > (1.0 / TF2_RATE))
  {
    // Publish tf2
    publish_tf2(ros_time);
    last_tf2_pub = time;
  }
  if ((time - last_topic_pub) > (1.0 / STATE_TOPIC_RATE))
  {
    // Publish state topic
    publish_state_topic(ros_time);
    last_topic_pub = time;
  }
}

void SlotcarCommon::publish_tf2(const rclcpp::Time& t)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  Eigen::Quaterniond quat(_pose.linear());
  tf_stamped.header.stamp = t;
  tf_stamped.header.frame_id = "world";
  tf_stamped.child_frame_id = _model_name + "/base_link";
  tf_stamped.transform.translation.x = _pose.translation()[0];
  tf_stamped.transform.translation.y = _pose.translation()[1];
  tf_stamped.transform.translation.z = _pose.translation()[2];
  tf_stamped.transform.rotation.x = quat.x();
  tf_stamped.transform.rotation.y = quat.y();
  tf_stamped.transform.rotation.z = quat.z();
  tf_stamped.transform.rotation.w = quat.w();
  _tf2_broadcaster->sendTransform(tf_stamped);
}

void SlotcarCommon::publish_state_topic(const rclcpp::Time& t)
{
  rmf_fleet_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.name = _model_name;
  robot_state_msg.battery_percent = std::ceil(100.0 * _soc);

  robot_state_msg.location.x = _pose.translation()[0];
  robot_state_msg.location.y = _pose.translation()[1];
  robot_state_msg.location.yaw = compute_yaw(_pose);
  robot_state_msg.location.t = t;
  robot_state_msg.location.level_name = get_level_name(_pose.translation()[2]);

  if (robot_state_msg.location.level_name.empty())
  {
    RCLCPP_ERROR(
      logger(),
      "Unable to determine the current level_name for robot [%s]. Kindly "
      "ensure the building_map_server is running. The RobotState message for"
      "this robot will not be published.",
      _model_name.c_str());

    return;
  }

  robot_state_msg.task_id = _current_task_id;
  robot_state_msg.path = _remaining_path;
  robot_state_msg.mode = _current_mode;
  robot_state_msg.mode.mode_request_id = pause_request.mode_request_id;

  if (_adapter_error)
  {
    robot_state_msg.mode.mode =
      rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
  }

  robot_state_msg.seq = ++_sequence;
  _robot_state_pub->publish(robot_state_msg);
}

void SlotcarCommon::mode_request_cb(
  const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
{
  // Request is for another robot
  if (msg->robot_name != _model_name)
    return;

  _current_mode = msg->mode;
  if (msg->mode.mode == msg->mode.MODE_DOCKING)
    _docking = true;
  else
    _docking = false;
}

void SlotcarCommon::map_cb(
  const rmf_building_map_msgs::msg::BuildingMap::SharedPtr msg)
{
  if (msg->levels.empty())
  {
    RCLCPP_ERROR(logger(), "Received empty building map");
    return;
  }

  for (const auto& level : msg->levels)
  {
    _level_to_elevation.insert({level.name, level.elevation});
  }
  _initialized_levels = true;

}

// Enables/disables charge when called from Ignition/Gazebo plugin
void SlotcarCommon::charge_state_cb(
  const std::string& name, bool selected)
{
  if (name == _enable_charge_str)
  {
    _enable_charge = selected;
  }
  else if (name == _enable_instant_charge_str)
  {
    _enable_instant_charge = selected;
  }
  else if (name == _enable_drain_str)
  {
    _enable_drain = selected;
  }
  else
  {
    std::cerr << "Invalid button selected. " << std::endl;
  }
}

bool SlotcarCommon::near_charger(const Eigen::Isometry3d& pose) const
{
  std::string lvl_name = _last_known_level;
  auto waypoints_it = _charger_waypoints.find(lvl_name);
  if (waypoints_it != _charger_waypoints.end())
  {
    const std::vector<ChargerWaypoint>& waypoints = waypoints_it->second;
    for (const ChargerWaypoint& waypoint : waypoints)
    {
      // Assumes it is on the same Z-plane
      double dist =
        sqrt(pow(waypoint.x - pose.translation()[0], 2)
          + pow(waypoint.y - pose.translation()[1], 2));
      if (dist < _charger_dist_thres)
      {
        return true;
      }
    }
  }
  return false;
}

double SlotcarCommon::compute_charge(const double run_time) const
{
  const double dQ = _params.charging_current * run_time; // Coulombs
  const double dSOC = dQ / (_params.nominal_capacity * 3600.0);
  return dSOC;
}

double SlotcarCommon::compute_discharge(
  const Eigen::Vector3d lin_vel, const double ang_vel,
  const Eigen::Vector3d lin_acc, const double ang_acc,
  const double run_time) const
{
  const double v = std::min(lin_vel.norm(), _nominal_drive_speed);
  const double w = std::min(std::abs(ang_vel), _nominal_turn_speed);
  const double a = std::min(lin_acc.norm(), _max_drive_acceleration);
  const double alpha = std::min(std::abs(ang_acc), _max_turn_acceleration);

  // Loss through acceleration
  const double EA = ((_params.mass * a * v) +
    (_params.inertia * alpha * w)) * run_time;

  // Loss through friction. Ignores friction due to rotation in place
  const double EF = compute_friction_energy(_params.friction_coefficient,
      _params.mass, v, run_time);

  // Change in energy as a result of motion
  const double dEM = EA + EF;
  // Change in energy due to any onboard device over the time period `run_time`
  const double dED = _params.nominal_power * run_time;

  // The charge consumed
  const double dQ = (dEM + dED) / _params.nominal_voltage;
  // The depleted state of charge as a fraction in range [0,1]
  double dSOC = dQ / (_params.nominal_capacity * 3600.0);

  return dSOC;
}

Eigen::Vector3d SlotcarCommon::get_lookahead_point() const
{
  return _lookahead_point;
}
