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

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

#include <rmf_building_sim_gz_plugins/utils.hpp>

namespace gz {
namespace sim {
struct DoorJoint
{
  std::string name;
  double closed_position;
  double open_position;
};

struct DoorData
{
  rmf_building_sim_gz_plugins::MotionParams params;
  std::vector<DoorJoint> joints;
  bool ros_interface; // Whether it's managed by RMF, false for lift doors
};

enum class DoorModeCmp
{
  CLOSE = 0,
  MOVING = 1,
  OPEN = 2
};

namespace components {
/// \brief A component used to describe an RMF door.
using Door = Component<DoorData, class DoorTag>;
GZ_SIM_REGISTER_COMPONENT("rmf_components.Door", Door)

/// \brief A component used to command an RMF door to open / close.
using DoorCmd = Component<DoorModeCmp, class DoorCmdTag>;
GZ_SIM_REGISTER_COMPONENT("rmf_components.DoorCmd", DoorCmd)

/// \brief A component used to show the state of an RMF door.
using DoorStateComp = Component<DoorModeCmp, class DoorStateTag>;
GZ_SIM_REGISTER_COMPONENT("rmf_components.DoorStateComp", DoorStateComp)
}
}
}
#endif
