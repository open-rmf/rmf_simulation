/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/node.hpp>

#include <rmf_robot_sim_common/readonly_blockade_common.hpp>

#include <rmf_robot_sim_common/utils.hpp>

namespace rmf_robot_sim_gazebo_plugins {

class ReadOnlyBlockade : public gazebo::ModelPlugin
{
public:

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) final
  {
    _model = model;
    _common.init(_model->GetName(), gazebo_ros::Node::Get(sdf), sdf);

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        [this](const auto&){ this->on_update(); });
  }

  void on_update()
  {
    const auto& world = _model->GetWorld();
    const auto pose = rmf_plugins_utils::convert_pose(_model->WorldPose());
    const auto sim_time = world->SimTime().Double();
    _common.on_update(pose, sim_time);
  }

private:
  rmf_robot_sim_common::ReadOnlyBlockadeCommon _common;
  gazebo::physics::ModelPtr _model;
  gazebo::event::ConnectionPtr _update_connection;
};

} // namespace rmf_robot_sim_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(rmf_robot_sim_gazebo_plugins::ReadOnlyBlockade)
