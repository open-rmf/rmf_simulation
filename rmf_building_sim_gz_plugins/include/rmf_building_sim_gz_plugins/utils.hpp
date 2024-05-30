#ifndef RMF_BUILDING_SIM_GZ_PLUGINS_UTILS_HPP
#define RMF_BUILDING_SIM_GZ_PLUGINS_UTILS_HPP

#include <chrono>

namespace rmf_building_sim_gz_plugins {

struct MotionParams
{
  double v_max = 0.2;
  double a_max = 0.1;
  double a_nom = 0.08;
  double dx_min = 0.01;
  double f_max = 10000000.0;
};

// TODO(luca) move this to a cpp file
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

  // Flip the sign to the correct direction before returning the value
  return sign * v_next;
}

/// Convert a duration into a double value representing seconds.
inline double to_seconds(const std::chrono::nanoseconds delta_t)
{
  using Sec64 = std::chrono::duration<double>;
  return std::chrono::duration_cast<Sec64>(delta_t).count();
}

}
#endif
