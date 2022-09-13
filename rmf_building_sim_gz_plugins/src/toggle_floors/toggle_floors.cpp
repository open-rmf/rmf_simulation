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

#include <string>
#include <iostream>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>

using FleetState = rmf_fleet_msgs::msg::FleetState;
using RobotState = rmf_fleet_msgs::msg::RobotState;

class toggle_floors
  :
  public ignition::gui::Plugin
{
  Q_OBJECT

public:
  toggle_floors();
  ~toggle_floors();
  void LoadConfig(const tinyxml2::XMLElement* _pluginElem) override;

private:

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  tinyxml2::XMLDocument _config_xml;

  bool eventFilter(QObject* _obj, QEvent* _event) override;
  void PerformRenderingOperations();
  void FindScene();
  ignition::rendering::ScenePtr scene{nullptr};
  void get_plugin_config();

  QStringList qfloors;
  std::unordered_map<std::string, std::atomic<bool>> _floor_changed_flags;
  std::unordered_map<std::string, std::atomic<bool>> _floor_visibility;
  std::unordered_map<std::string, std::string> _robot_floor;
  std::unordered_map<std::string, std::vector<std::string>> _floor_models;

  bool _floor_checked_flag{false};


protected slots:
  void OnFloorChecked(const QString, bool);

};

toggle_floors::toggle_floors()
{
  char const** argv = NULL;
  if (!rclcpp::ok())
  {
    rclcpp::init(0, argv);
  }
  _ros_node = std::make_shared<rclcpp::Node>("toggle_floors");
}

toggle_floors::~toggle_floors()
{
  ignition::gui::App()->findChild<
    ignition::gui::MainWindow*>()->removeEventFilter(this);
}

// Get plugin information from ignition transport
void toggle_floors::get_plugin_config()
{
  ignition::transport::Node node;
  bool result{false};
  unsigned int timeout{2000};
  const std::string WORLD_NAME = "sim_world";

  ignition::msgs::GUI res;
  std::string service = ignition::transport::TopicUtils::AsValidTopic("/world/" + WORLD_NAME +
      "/gui/info");

  node.Request(service, timeout, res, result);

  for (int p = 0; p < res.plugin_size(); ++p)
  {
    const auto& plugin = res.plugin(p);
    const auto& fileName = plugin.filename();
    if (fileName == "toggle_floors")
    {
      _config_xml.Parse(plugin.innerxml().c_str());
      break;
    }
  }
  return;
}

void toggle_floors::LoadConfig(const tinyxml2::XMLElement* _pluginElem)
{
  if (this->title.empty())
    this->title = "Toggle Floors";

  const tinyxml2::XMLElement* plugin_config = [&]
    {
      if (_pluginElem->FirstChildElement("floor"))
      {
        return _pluginElem->FirstChildElement("floor");
      }
      else
      {
        get_plugin_config();
        const tinyxml2::XMLElement* p = _config_xml.RootElement();
        return p;
      }
    } ();

  if (plugin_config)
  {
    for (auto floor_ele = plugin_config->ToElement();
      floor_ele;
      floor_ele = floor_ele->NextSiblingElement("floor"))
    {
      std::vector<std::string> models;

      auto floor_name = std::string(floor_ele->Attribute("name"));
      auto building = std::string(floor_ele->Attribute("model_name"));
      qfloors.append(QString::fromStdString(floor_name));
      models.push_back(building);

      for (auto model_ele = floor_ele->FirstChildElement("model");
        model_ele;
        model_ele = model_ele->NextSiblingElement("model"))
      {
        auto model_name = std::string(model_ele->Attribute("name"));
        models.push_back(model_name);
        printf(
          "toggle_floors found a floor element: [%s]->[%s]\n",
          floor_name.c_str(),
          model_name.c_str());
      }
      _floor_visibility[floor_name] = true;
      _floor_changed_flags[floor_name] = true;
      _floor_models[floor_name] = models;
    }
  }

  auto qos_profile = rclcpp::QoS(10);
  _fleet_state_sub = _ros_node->create_subscription<FleetState>(
    "/fleet_states", qos_profile,
    [this](FleetState::UniquePtr msg)
    {
      for (const RobotState& robot : msg->robots)
      {
        _robot_floor[robot.name] = robot.location.level_name;
      }
    });

  this->Context()->setContextProperty("qfloors", qfloors);
  ignition::gui::App()->findChild<
    ignition::gui::MainWindow*>()->installEventFilter(this);
}

void toggle_floors::OnFloorChecked(const QString floor, bool checked)
{
  auto floor_str = floor.toStdString();
  _floor_visibility[floor_str] = checked;
  _floor_changed_flags[floor_str] = true;
}

void toggle_floors::PerformRenderingOperations()
{
  if (nullptr == this->scene)
  {
    this->FindScene();
  }
  if (nullptr == this->scene)
  {
    return;
  }

  rclcpp::spin_some(_ros_node);

  for (auto& flags_itr : _floor_changed_flags)
  {
    if (flags_itr.second)
    {
      auto floor_name = flags_itr.first;
      auto models = _floor_models[floor_name];
      for (const std::string& model : models)
      {
        auto target_node = this->scene->NodeByName(model);
        if (target_node == NULL)
        {
          ignwarn << "Node for " << model << "was not found" <<std::endl;
          continue;
        }
        auto target_vis =
          std::dynamic_pointer_cast<ignition::rendering::Visual>(target_node);
        target_vis->SetVisible(_floor_visibility[floor_name]);
      }
      flags_itr.second = false;
    }
  }

  for (auto& robots_itr : _robot_floor)
  {
    auto target_node = this->scene->NodeByName(robots_itr.first);
    if (target_node == NULL)
    {
      ignwarn << "Node for " << robots_itr.first << "was not found" <<std::endl;
      continue;
    }
    auto target_vis = std::dynamic_pointer_cast<ignition::rendering::Visual>(
      target_node);
    target_vis->SetVisible(_floor_visibility[robots_itr.second]);
  }
}

void toggle_floors::FindScene()
{
  auto loadedEngNames = ignition::rendering::loadedEngines();
  auto engineName = loadedEngNames[0];
  auto engine = ignition::rendering::engine(engineName);

  auto scenePtr = engine->SceneByIndex(0);
  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }
  this->scene = scenePtr;
}

bool toggle_floors::eventFilter(QObject* _obj, QEvent* _event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->PerformRenderingOperations();
  }
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(toggle_floors,
  ignition::gui::Plugin
)


#include "toggle_floors.moc"