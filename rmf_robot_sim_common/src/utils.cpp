#include <cmath>
#include <algorithm>

#include <rmf_robot_sim_common/utils.hpp>

namespace rmf_plugins_utils {

//==============================================================================

double compute_desired_rate_of_change(
  double _s_target,
  double _v_actual,
  double _speed_target_now,
  double _speed_target_dest,
  const MotionParams& _motion_params,
  const double _dt)
{
  using std::copysign;
  using std::min;
  using std::signbit;

  bool v_opposite_s = abs(_v_actual) > 0 &&
    (signbit(_v_actual) != signbit(_s_target));

  if (abs(_s_target) == 0.0 || v_opposite_s)
  {
    // Use maximum accleration to come to a stop
    double v_difference = copysign(
      min(abs(_v_actual), _motion_params.a_max * _dt),
      _v_actual);
    return _v_actual - v_difference;
  }

  // This is what our acceleration should be if we want to begin a constant
  // acceleration from now until we reach the goal with _speed_target_dest
  double required_accel =
    (pow(_speed_target_dest, 2) - pow(_v_actual, 2)) / (2.0 * _s_target);
  // Test acceleration limit
  required_accel = copysign(
    min(abs(required_accel), _motion_params.a_max),
    required_accel);
  if (abs(required_accel) >= _motion_params.a_nom)
  {
    // If the smallest constant acceleration for reaching the goal is
    // greater than the nominal acceleration, then we should begin
    // accelerating right away so that we can smoothly reach the goal while
    // accelerating as close to the nominal acceleration as possible.
    double v_next = required_accel * _dt + _v_actual;
    return v_next;
  }

  // Test target speed limit
  double v_target_now = copysign(
    min(abs(_speed_target_now), _motion_params.v_max), _s_target);

  // Test acceleration limit
  double v_change = v_target_now - _v_actual;
  v_change = copysign(
    min(abs(v_change), _motion_params.a_nom * _dt), v_change);
  double v_next = _v_actual + v_change;
  return v_next;
}

//================================================================================
rclcpp::Time simulation_now(double t)
{
  const int32_t t_sec = static_cast<int32_t>(t);
  const uint32_t t_nsec =
    static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
  return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
}

} // namespace rmf_plugins_utils
