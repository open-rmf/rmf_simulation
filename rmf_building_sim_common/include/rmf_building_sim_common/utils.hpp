#ifndef SRC__RMF_BUILDING_SIM_COMMON__UTILS_HPP
#define SRC__RMF_BUILDING_SIM_COMMON__UTILS_HPP

#include <cmath>
#include <iostream>

namespace rmf_building_sim_common {

struct MotionParams
{
  double v_max = 0.2;
  double a_max = 0.1;
  double a_nom = 0.08;
  double dx_min = 0.01;
  double f_max = 10000000.0;
};

//==============================================================================
// TODO(MXG): Refactor the use of this function to replace it with
// compute_desired_rate_of_change()
double compute_ds(
  double s_target,
  double v_actual,
  const double v_max,
  const double accel_nom,
  const double accel_max,
  const double dt);

//==============================================================================
double compute_desired_rate_of_change(
  double _s_target,
  double _v_actual,
  const MotionParams& _motion_params,
  const double _dt);

//==============================================================================
template<typename SdfPtrT, typename SdfElementPtrT>
bool get_element_required(
  SdfPtrT& _sdf,
  const std::string& _element_name,
  SdfElementPtrT& _element)
{
  if (!_sdf->HasElement(_element_name))
  {
    std::cerr << "Element [" << _element_name << "] not found" << std::endl;
    return false;
  }
  // using GetElementImpl() because for sdf::v9 GetElement() is not const
  _element = _sdf->GetElementImpl(_element_name);
  return true;
}

//==============================================================================
template<typename T, typename SdfPtrT>
bool get_sdf_attribute_required(SdfPtrT& sdf, const std::string& attribute_name,
  T& value)
{
  if (sdf->HasAttribute(attribute_name))
  {
    if (sdf->GetAttribute(attribute_name)->Get(value))
    {
      std::cout << "Using specified attribute value [" << value
                << "] for property [" << attribute_name << "]"
                << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf attribute for [" << attribute_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cerr << "Attribute [" << attribute_name << "] not found" << std::endl;
  }

  return false;
}

//==============================================================================
template<typename T, typename SdfPtrT>
bool get_sdf_param_required(SdfPtrT& sdf, const std::string& parameter_name,
  T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
      return true;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name << "]"
                <<std::endl;
    }
  }
  else
  {
    std::cerr << "Property [" << parameter_name << "] not found" << std::endl;
  }

  return false;
}

//==============================================================================
template<typename T, typename SdfPtrT>
void get_sdf_param_if_available(SdfPtrT& sdf, const std::string& parameter_name,
  T& value)
{
  if (sdf->HasElement(parameter_name))
  {
    if (sdf->GetElement(parameter_name)->GetValue()->Get(value))
    {
      std::cout << "Using specified value [" << value << "] for property ["
                << parameter_name << "]" << std::endl;
    }
    else
    {
      std::cerr << "Failed to parse sdf value for [" << parameter_name
                << "]" << std::endl;
    }
  }
  else
  {
    std::cout << "Using default value [" << value << "] for property ["
              << parameter_name << "]" << std::endl;
  }
}

} // namespace rmf_building_sim_common

#endif // SRC__RMF_BUILDING_SIM_COMMON__UTILS_HPP
