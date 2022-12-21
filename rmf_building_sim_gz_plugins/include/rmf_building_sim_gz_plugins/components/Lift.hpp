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

#ifndef RMF_BUILDING_SIM_GZ_PLUGINS_COMPONENTS_LIFT_HPP
#define RMF_BUILDING_SIM_GZ_PLUGINS_COMPONENTS_LIFT_HPP

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

#include <rmf_building_sim_common/utils.hpp>

#include <rmf_building_sim_gz_plugins/components/Door.hpp>

namespace ignition
{
namespace gazebo
{
  struct FloorDoorPair {
    std::string cabin_door;
    std::string shaft_door;
  };

  struct Floor {
    double elevation;
    std::vector<FloorDoorPair> doors;
  };

  struct LiftData {
    std::string name;
    std::unordered_map<std::string, Floor> floors; // Maps name to floor
    std::string initial_floor;
    rmf_building_sim_common::MotionParams params;
    std::string cabin_joint;
  };

  struct LiftCommand {
    uint8_t request_type;
    std::string destination_floor;
    std::string session_id;
    DoorCommand door_state;
  };

  namespace components
  {
    /// \brief A component used to describe an RMF lift.
    using Lift = Component<LiftData, class LiftTag>;
    IGN_GAZEBO_REGISTER_COMPONENT("rmf_components.Lift", Lift)

    /// \brief A component used to command an RMF lift to open / close.
    using LiftCmd = Component<LiftCommand, class LiftCmdTag>;
    IGN_GAZEBO_REGISTER_COMPONENT("rmf_components.LiftCmd", LiftCmd)
  }
}
}
#endif
