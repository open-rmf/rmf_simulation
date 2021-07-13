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
#include <rmf_building_sim_common/charger_common.hpp>

#include <gazebo_ros/node.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace charger_simulator
{
  class ChargerSimulator: public gazebo::WorldPlugin
  {
  public:
    ChargerSimulator():
      _charger_common(
        std::make_unique<rmf_building_sim_common::ChargerCommon>())
    {
      _gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
      _gazebo_node->Init();
      _update_sub =
        _gazebo_node->Subscribe(
          "/charger_update/", &ChargerSimulator::on_update_charger, this);
      _complete_sub =
        _gazebo_node->Subscribe(
          "/charger_complete/", &ChargerSimulator::on_complete_charger, this);
    }

    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
    {
      gazebo_ros::Node::SharedPtr _ros_node = gazebo_ros::Node::Get(sdf);
      _charger_common->init_ros_node(_ros_node);

      auto waypoints = sdf->GetElement("rmf_charger_waypoints");
      if(sdf->HasElement("rmf_charger_waypoints"))
      {
        if (waypoints->HasElement("rmf_vertex"))
        {
          auto waypoint = waypoints->GetElement("rmf_vertex");
          while (waypoint)
          {
            if (waypoint->HasAttribute("x") && waypoint->HasAttribute("y") &&
              waypoint->HasAttribute("level") && waypoint->HasAttribute("name"))
            {
              std::string lvl_name, waypoint_name;
              double x, y;
              waypoint->GetAttribute("x")->Get(x);
              waypoint->GetAttribute("y")->Get(y);
              waypoint->GetAttribute("level")->Get(lvl_name);
              waypoint->GetAttribute("name")->Get(waypoint_name);
              _charger_common->
                add_charger(waypoint_name, x, y, lvl_name);
            }
            waypoint = waypoint->GetNextElement("rmf_vertex");
          }
        }
      }
    }

    void on_update_charger(ConstHeaderPtr &msg)
    {
      rclcpp::Duration duration{msg->stamp().sec(), msg->stamp().nsec()};
      _charger_common->update_charger_charge(msg->str_id(), duration);
    }

    void on_complete_charger(ConstGzStringPtr &msg)
    {
      _charger_common->idle(msg->data());
    }
  private:
    std::unique_ptr<rmf_building_sim_common::ChargerCommon> _charger_common;
    gazebo::transport::NodePtr _gazebo_node;
    gazebo::transport::SubscriberPtr _update_sub;
    gazebo::transport::SubscriberPtr _complete_sub;
  };
}

GZ_REGISTER_WORLD_PLUGIN(charger_simulator::ChargerSimulator)
