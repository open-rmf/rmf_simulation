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

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <sdf/Light.hh>
#include <gz/rendering.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>

// Data model representing a list of lights. Provides data for delegates to display
// in QML via a series of roles which the delegates bind to.
class LightsModel : public QAbstractListModel
{
  Q_OBJECT
  Q_ENUMS(Roles)

public:
  enum Roles
  {
    NameRole = Qt::UserRole + 1,
    PoseRole,
    IndexRole,
    DiffuseRole,
    SpecularRole,
    AttenuationRangeRole,
    AttenuationConstantRole,
    AttenuationLinearRole,
    AttenuationQuadraticRole,
    DirectionRole,
    SpotInnerAngleRole,
    SpotOuterAngleRole,
    SpotFalloffRole
  };

  QHash<int, QByteArray> roleNames() const override;
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index,
    int role = Qt::DisplayRole) const override;
  // Note: We do not need to override and define the setData() function since
  // we only update our model from the GUI in `LightTuning::OnCreateLight`

  // Inserts a default light with the name specified in `name_qstr`,
  // if it does not exist
  void add_new_light(const QString& name_qstr);
  // Deletes the light from LightsModel at index `idx` if it exists
  void remove_light(int idx);

  // Returns a reference to the light at index `idx` in `_lights`.
  // Assumes `idx` is valid.
  sdf::Light& get_light(int idx);
  // Returns a reference to the light with name `name` in `_lights`.
  // Assumes `name` is a valid name belonging to some light in `_lights`.
  sdf::Light& get_light(const std::string& name);
  // Returns a const reference to a QVector of all lights in the LightsModel
  // object
  const QVector<sdf::Light>& get_lights() const;

  // Fill _existing_names with names from the ecm
  void populate_names(gz::sim::EntityComponentManager& ecm);

private:
  // Collection of lights, each with a unique name (enforced by
  // add_new_light)
  QVector<sdf::Light> _lights;

  // Set of existing names in the simulation. Used to ensure no name collisions
  // when creating new lights
  std::unordered_set<std::string> _pre_existing_names;
};
// Class that handles all GUI interactions and their associated
// light creations/deletions
class LightTuning : public gz::sim::GuiSystem
{
  Q_OBJECT

public:
  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;

  void Update(const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm) override;

signals:
  void poseChanged(QString nm, QString new_pose);
  void markerSelected(QString nm);

public slots:
  void OnLightTypeSelect(sdf::Light& light, const QString& type);
  void OnCreateLightBtnPress(
    int idx, bool cast_shadow, const QString& type,
    const QString& name, const QString& pose_str,
    const QString& diffuse_str, const QString& specular_str,
    const QString& attentuation_range_str,
    const QString& attentuation_constant_str,
    const QString& attentuation_linear_str,
    const QString& attentuation_quadratic_str,
    const QString& direction_str,
    const QString& spot_inner_angle_str,
    const QString& spot_outer_angle_str,
    const QString& spot_falloff_str);
  void OnRemoveLightBtnPress(int idx, const QString& name);
  void OnAddLightFormBtnPress(const QString& name);
  void OnSaveLightsBtnPress(const QString& url, bool save_all = true,
    int idx = -1);

protected: bool eventFilter(QObject* _obj, QEvent* _event) override;

private:
  bool first_update_call = true;
  const std::string sdf_open_tag = "<sdf version=\"1.7\"> \n";
  const std::string sdf_close_tag = "</sdf>";

  std::string _world_name;
  gz::transport::Node _node;
  LightsModel _model;
  gz::rendering::ScenePtr scene_ptr;

  // Contains an Entity that serves as a physical representation of
  // the light on the screen, so that a user can move it around to set the
  // light pose
  struct LightMarker
  {
    std::string name; // Name of the LightMarker
    gz::sim::Entity en;
    gz::math::Pose3d last_set_pose;
  };
  // List of pairs of light name and corresponding marker name being spawned
  std::vector<std::pair<std::string, std::string>> _markers_spawn_pipeline;
  // Map from light name to its corresponding marker
  std::unordered_map<std::string, LightMarker> _markers;

  enum class Action {REMOVE, CREATE};
  // Map from a light name to the latest update/creation/removal request
  // for that corresponding light
  std::unordered_map<std::string, Action> actions;

  // Returns a string representation of the specified light in the
  // SDF v1.7 format
  std::string light_to_sdf_string(const sdf::Light&);

  // Sends a service request to render the LightMarker corresponding to the light
  // with name `light_name` using Ignition transport
  void create_marker_service(
    const std::string& light_name, const gz::math::Pose3d& pose);
  // Sends a service request to remove the LightMarker corresponding to the light
  // with name `light_name` using Ignition transport
  void remove_marker_service(const std::string& light_name);

  // Gets and stores a pointer to scene from the Rendering singleton
  void load_scene();
  // Handles light creation/update/deletion requests stored in `actions`
  void render_lights();
  // Displays the light in `_model` that has the name `light_name`
  void create_light_rendering(const std::string& light_name);
  // Removes from the scene the light with name `light_name`
  void remove_light_rendering(const std::string& light_name);
};
