#include <cmath>
#include <algorithm>

#include <rmf_robot_sim_common/utils.hpp>

namespace rmf_plugins_utils {

//==============================================================================
double compute_ds(
  double s_target,
  double v_actual,
  const double v_max,
  const double accel_nom,
  const double accel_max,
  const double dt,
  const double v_target)
{
  double sign = 1.0;
  if (s_target < 0.0)
  {
    // Limits get confusing when we need to go backwards, so we'll flip signs
    // here so that we pretend the target is forwards
    s_target *= -1.0;
    v_actual *= -1.0;
    sign = -1.0;
  }

  // We should try not to shoot past the target
  double next_v = s_target / dt;

  // Test velocity limit
  next_v = std::min(next_v, v_max);

  // Test acceleration limit
  next_v = std::min(next_v, accel_nom * dt + v_actual);

  if (v_actual > 0.0 && s_target > 0.0)
  {
    // This is what our deceleration should be if we want to begin a constant
    // deceleration from now until we reach the goal
    double deceleration = pow(v_actual - v_target, 2) / s_target;
    deceleration = std::min(deceleration, accel_max);

    if (accel_nom <= deceleration)
    {
      // If the smallest constant deceleration for reaching the goal is
      // greater than the nominal acceleration, then we should begin
      // decelerating right away so that we can smoothly reach the goal while
      // decelerating as close to the nominal acceleration as possible.
      next_v = -deceleration * dt + v_actual;
    }
  }

  // if you have a target velocity and a distance shorter than that, set to
  // the target velocity otherwise it'll hard-stop
  if (s_target <= v_target)
    next_v = v_target;

  // Flip the sign the to correct direction before returning the value
  return sign * next_v;
}

//==============================================================================
double compute_desired_rate_of_change(
  double _s_target,
  double _v_actual,
  const MotionParams& _motion_params,
  const double _dt)
{
  double sign = 1.0;
  if (_s_target < 0.0)
  {
    // Limits get confusing when we need to go backwards, so we'll flip signs
    // here so that we pretend the target is forwards
    _s_target *= -1.0;
    _v_actual *= -1.0;
    sign = -1.0;
  }

  // We should try not to shoot past the target
  double v_next = _s_target / _dt;

  // Test velocity limit
  v_next = std::min(v_next, _motion_params.v_max);

  // Test acceleration limit
  v_next = std::min(v_next, _motion_params.a_nom * _dt + _v_actual);

  if (_v_actual > 0.0 && _s_target > 0.0)
  {
    // This is what our deceleration should be if we want to begin a constant
    // deceleration from now until we reach the goal
    double deceleration = pow(_v_actual, 2) / (2.0 * _s_target);
    deceleration = std::min(deceleration, _motion_params.a_max);

    if (_motion_params.a_nom <= deceleration)
    {
      // If the smallest constant deceleration for reaching the goal is
      // greater than the nominal acceleration, then we should begin
      // decelerating right away so that we can smoothly reach the goal while
      // decelerating as close to the nominal acceleration as possible.
      v_next = -deceleration * _dt + _v_actual;
    }
  }

  // Flip the sign the to correct direction before returning the value
  return sign * v_next;
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
