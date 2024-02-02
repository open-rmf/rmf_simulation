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

#include <string>
#include <iostream>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/common/Image.hh>
#include <QQuickImageProvider>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>


class ImageProvider : public QQuickImageProvider
{
  public: ImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}

  public: QImage requestImage(const QString &, QSize *, const QSize &) override
    {
      if (!this->img.isNull())
      {
        // Must return a copy
        QImage copy(this->img);
        return copy;
      }

      // Placeholder in case we have no image yet
      QImage i(400, 400, QImage::Format_RGB888);
      i.fill(QColor(128, 128, 128, 100));
      return i;
    }

  public: void SetImage(const QImage &_image)
  {
    this->img = _image;
  }

  private: QImage img;
};

struct CameraView
{
  ignition::msgs::Image _image_msg;
  ImageProvider *provider{nullptr};
  QString provider_name;
  QImage image;
};

class multiview : public ignition::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(
    QStringList topicList
    READ TopicList
    NOTIFY TopicListChanged
  )

private:
  ignition::transport::Node _node;
  std::unordered_map<std::string, CameraView> _camera_views;
  QStringList topicList;

public:
  multiview();
  ~multiview();

  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;

public: Q_INVOKABLE QStringList TopicList() const;

protected:
  void receive_image_cb(const ignition::msgs::Image& _msg,
                        const ignition::transport::MessageInfo &_info);
signals:
  void TopicListChanged();
  void newImage(QString selected_topic);
};

multiview::multiview() {}

multiview::~multiview()
{
  for (auto cam : _camera_views)
  {
    ignition::gui::App()->Engine()->removeImageProvider(cam.second.provider_name);
  }
}

void multiview::LoadConfig(const tinyxml2::XMLElement* _pluginElem)
{
  if (!_pluginElem)
    return;

  if (this->title.empty())
    this->title = "Multiview Rendering";

  for (auto e = _pluginElem->FirstChildElement("topic"); e != NULL; e = e->NextSiblingElement("topic"))
  {
    std::string topic_name = e->GetText();

    if (!_node.Subscribe(topic_name, &multiview::receive_image_cb, this))
      std::cerr << "Error subscribing to topic [/" << topic_name << "]" << std::endl;

    CameraView new_view = {
      .provider = new ImageProvider(),
      .provider_name = QString::fromUtf8(topic_name.c_str())
    };
    ignition::gui::App()->Engine()->addImageProvider(new_view.provider_name, new_view.provider);
    _camera_views.insert({topic_name, new_view});

    topicList.push_back(QString::fromUtf8(topic_name.c_str()));
  }

  this->TopicListChanged();
}

void multiview::receive_image_cb(const ignition::msgs::Image& _msg,
                                 const ignition::transport::MessageInfo &_info)
{
  std::string topic_name = _info.Topic();
  topic_name.erase(0,1);

  auto cam = _camera_views.find(topic_name);
  if (cam == _camera_views.end())
  {
    std::cerr << "Could not find camera view in world file" << std::endl;
    return;
  }

  cam->second._image_msg = _msg;

  unsigned int height = cam->second._image_msg.height();
  unsigned int width = cam->second._image_msg.width();
  QImage::Format qFormat = QImage::Format_RGB888;

  QImage image = QImage(width, height, qFormat);

  // ignition::common::Image output;
  switch (_msg.pixel_format_type())
  {
    case ignition::msgs::PixelFormatType::RGB_INT8:
      image = QImage(reinterpret_cast<const uchar *>(cam->second._image_msg.data().c_str()), width, height, qFormat);
      // std::cout << ">> format is RGB_INT8" << std::endl;
      break;
    case ignition::msgs::PixelFormatType::R_FLOAT32:
      // std::cout << ">> format is R_FLOAT32" << std::endl;
      break;
    case ignition::msgs::PixelFormatType::L_INT16:
      // std::cout << ">> format is L_INT16" << std::endl;
      break;
    case ignition::msgs::PixelFormatType::L_INT8:
      // std::cout << ">> format is L_INT8" << std::endl;
      break;
    default:
    {
      std::cerr << "Received an image of unsupported format type." << std::endl;
      return;
    }
  }

  cam->second.provider->SetImage(image);
  QString qstring_topic_name = QString::fromUtf8(topic_name.c_str());
  this->newImage(qstring_topic_name);

}


QStringList multiview::TopicList() const
{
  return topicList;
}


// Register this plugin
IGNITION_ADD_PLUGIN(multiview,
  ignition::gui::Plugin)


#include "multiview.moc"