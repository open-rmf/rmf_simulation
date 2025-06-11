/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#ifndef RMF_BUILDING_SIM_GZ_PLUGINS_COMPONENTS_CHARGER_HPP
#define RMF_BUILDING_SIM_GZ_PLUGINS_COMPONENTS_CHARGER_HPP

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz {
namespace sim {
// Marker component for chargers, data such as name and pose is contained
// in builtin Name and Pose gz components
struct ChargerData {};

namespace components {
/// \brief A component used to describe an RMF robot charger.
using Charger = Component<ChargerData, class DoorTag>;
GZ_SIM_REGISTER_COMPONENT("rmf_components.Charger", Charger)
}
}
}
#endif

