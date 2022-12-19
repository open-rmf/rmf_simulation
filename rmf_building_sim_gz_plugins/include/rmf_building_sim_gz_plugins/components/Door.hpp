/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef RMF_BUILDING_SIM_GZ_PLUGINS_COMPONENTS_DOOR_HPP
#define RMF_BUILDING_SIM_GZ_PLUGINS_COMPONENTS_DOOR_HPP

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
namespace components
{
  struct DoorData {
    double v_max;
    double a_max;
    double a_nom;
    double dx_min;
    double f_max;
    std::string left_joint_name;
    std::string right_joint_name;
  };

  /// \brief A component used to turn off a model's joint's movement.
  using Door = Component<DoorData, class DoorTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Door",
      Door)
}
}
}
#endif

