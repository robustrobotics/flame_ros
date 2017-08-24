/**
 * This file is part of flame_ros.
 * Copyright (C) 2017 W. Nicholas Greene (wng@csail.mit.edu)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * @file textured_mesh_display.cc
 * @author W. Nicholas Greene
 * @date 2017-02-21 15:52:08 (Tue)
 */

#include "flame_rviz_plugins/textured_mesh_display.h"

#include <vector>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pluginlib/class_loader.h>

#include <image_transport/subscriber_plugin.h>

#include "flame_rviz_plugins/textured_mesh_visual.h"
#include "flame_rviz_plugins/surface_normals_visual.h"

namespace flame_rviz_plugins {

TexturedMeshDisplay::TexturedMeshDisplay() :
    rviz::Display(),
    mesh_filter_(nullptr),
    num_meshes_received_(0),
    tex_it_(nullptr),
    tex_filter_(nullptr),
    mesh_queue_(),
    tex_queue_(),
    visual_(nullptr),
    normals_(nullptr),
    mesh_topic_prop_("PolygonMesh Topic", "", "", "", this, SLOT(updateTopic())),
    tex_topic_prop_("Texture topic", "", "", "", this, SLOT(updateTopic())),
    tex_transport_prop_(new rviz::EnumProperty("Texture Transport Hint", "raw",
                                               "Preferred method of sending images.",
                                               this, SLOT(updateTopic()))),
    polygon_mode_prop_("Polygon Mode", "", "Polygon rendering mode.",
                       this, SLOT(updatePolygonMode())),
    shader_program_prop_("Shader Program", "", "Shader program for mesh.",
                   this, SLOT(updateShaderProgram())),
    phong_shading_prop_("Phong Shading", true, "Use Phong Shading.", this,
                        SLOT(updatePhongShading())),
    scene_color_scale_prop_("Scene Color Scale", 1.0f, "Color scale for shaders.",
                            this, SLOT(updateSceneColorScale())),
    show_normals_prop_("Show Normals", false, "Show normal vectors.", this,
                       SLOT(updateShowNormals())),
    normal_size_prop_("Normal Size", 0.05f, "Size of normal vector lines.",
                      this, SLOT(updateNormalSize())),
    queue_size_prop_("Queue Size", 25,
                     "Advanced: set the size of the incoming message queue.  Increasing this "
                     "is useful if your incoming TF data is delayed significantly from your"
                     " image data, but it can greatly increase memory usage if the messages are big.",
                     this, SLOT(updateQueueSize())),
    transport_plugin_types_(),
    mtx_() {
  /* Add polygon mode enums. */
  polygon_mode_prop_.addOptionStd("POINTS",
                                  static_cast<int>(Ogre::PM_POINTS));
  polygon_mode_prop_.addOptionStd("WIREFRAME",
                                  static_cast<int>(Ogre::PM_WIREFRAME));
  polygon_mode_prop_.addOptionStd("SOLID",
                                  static_cast<int>(Ogre::PM_SOLID));

  // Add shader enums.
  shader_program_prop_.addOptionStd("TEXTURE",
                              static_cast<int>(TexturedMeshVisual::ShaderProgram::TEXTURE));
  shader_program_prop_.addOptionStd("INVERSE_DEPTH",
                              static_cast<int>(TexturedMeshVisual::ShaderProgram::INVERSE_DEPTH));
  shader_program_prop_.addOptionStd("JET",
                                    static_cast<int>(TexturedMeshVisual::ShaderProgram::JET));
  shader_program_prop_.addOptionStd("SURFACE_NORMAL",
                              static_cast<int>(TexturedMeshVisual::ShaderProgram::SURFACE_NORMAL));

  // Scene color scale property.
  scene_color_scale_prop_.setMin(0.0f);

  QString mesh_msg_type = QString::fromStdString(
      ros::message_traits::datatype<pcl_msgs::PolygonMesh>());
  mesh_topic_prop_.setMessageType(mesh_msg_type);
  mesh_topic_prop_.setDescription(mesh_msg_type + " topic to subscribe to.");

  QString tex_msg_type = QString::fromStdString(
      ros::message_traits::datatype<sensor_msgs::Image>());
  tex_topic_prop_.setMessageType(tex_msg_type);
  tex_topic_prop_.setDescription(tex_msg_type + " topic to subscribe to.");

  // Texture transport property.
  connect(tex_transport_prop_.get(), SIGNAL(requestOptions(EnumProperty*)),
          this, SLOT(fillTransportOptionList(EnumProperty*)));
  tex_transport_prop_->setStdString("raw");

  // Queue size property
  queue_size_prop_.setMin(1);
  queue_size_ = queue_size_prop_.getInt();

  return;
}

void TexturedMeshDisplay::onInitialize() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  tex_it_.reset(new image_transport::ImageTransport(update_nh_));

  // Scan for available transport plugins
  scanForTransportSubscriberPlugins();

  return;
}

TexturedMeshDisplay::~TexturedMeshDisplay() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  unsubscribe();
  return;
}

void TexturedMeshDisplay::reset() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  Display::reset();
  visual_.reset();
  normals_.reset();

  return;
}

void TexturedMeshDisplay::updateTopic() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
  return;
}

void TexturedMeshDisplay::updateQueueSize() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  queue_size_ = queue_size_prop_.getInt();
  return;
}

void TexturedMeshDisplay::updatePolygonMode() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    Ogre::PolygonMode pm =
        static_cast<Ogre::PolygonMode>(polygon_mode_prop_.getOptionInt());
    visual_->setPolygonMode(pm);
  }
  return;
}

void TexturedMeshDisplay::updateShaderProgram() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    TexturedMeshVisual::ShaderProgram sp =
        static_cast<TexturedMeshVisual::ShaderProgram>(shader_program_prop_.getOptionInt());
    visual_->setShaderProgram(sp);
  }
  return;
}

void TexturedMeshDisplay::updatePhongShading() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    visual_->setPhongShading(phong_shading_prop_.getBool());
  }
  return;
}

void TexturedMeshDisplay::updateSceneColorScale() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    visual_->setSceneColorScale(scene_color_scale_prop_.getFloat());
  }
  return;
}

void TexturedMeshDisplay::updateShowNormals() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (normals_ != nullptr) {
    normals_->setVisible(show_normals_prop_.getBool());
  }
  return;
}

void TexturedMeshDisplay::updateNormalSize() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (normals_ != nullptr) {
    normals_->setNormalSize(normal_size_prop_.getFloat());
  }
  return;
}

void TexturedMeshDisplay::fillTransportOptionList(EnumProperty* property) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  property->clearOptions();

  std::vector<std::string> choices;

  choices.push_back("raw");

  // Loop over all current ROS topic names
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it) {
    // If the beginning of this topic name is the same as topic_,
    // and the whole string is not the same,
    // and the next character is /
    // and there are no further slashes from there to the end,
    // then consider this a possible transport topic.
    const ros::master::TopicInfo& ti = *it;
    const std::string& topic_name = ti.name;
    const std::string& topic = tex_topic_prop_.getStdString();

    // cppcheck-suppress stlIfStrFind
    if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/'
        && topic_name.find('/', topic.size() + 1) == std::string::npos) {
      std::string transport_type = topic_name.substr(topic.size() + 1);

      // If the transport type string found above is in the set of
      // supported transport type plugins, add it to the list.
      if (transport_plugin_types_.find(transport_type) !=
          transport_plugin_types_.end()) {
        choices.push_back(transport_type);
      }
    }
  }

  for (size_t ii = 0; ii < choices.size(); ii++) {
    property->addOptionStd(choices[ii]);
  }

  return;
}

void TexturedMeshDisplay::scanForTransportSubscriberPlugins() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  pluginlib::ClassLoader<image_transport::SubscriberPlugin>
      sub_loader("image_transport", "image_transport::SubscriberPlugin");

  BOOST_FOREACH(const std::string& lookup_name, sub_loader.getDeclaredClasses()) {
    // lookup_name is formatted as "pkg/transport_sub", for instance
    // "image_transport/compressed_sub" for the "compressed"
    // transport.  This code removes the "_sub" from the tail and
    // everything up to and including the "/" from the head, leaving
    // "compressed" (for example) in transport_name.
    std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
    transport_name = transport_name.substr(lookup_name.find('/') + 1);

    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try {
      boost::shared_ptr<image_transport::SubscriberPlugin> sub =
          sub_loader.createInstance(lookup_name);
      transport_plugin_types_.insert(transport_name);
    } catch (const pluginlib::LibraryLoadException& e) {}
    catch (const pluginlib::CreateClassException& e) {}
  }

  return;
}

void TexturedMeshDisplay::subscribe() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  if (!isEnabled()) {
    return;
  }

  try {
    mesh_filter_.reset(new message_filters::Subscriber<pcl_msgs::PolygonMesh>());
    tex_filter_.reset(new image_transport::SubscriberFilter());

    std::string mesh_topic = mesh_topic_prop_.getTopicStd();
    std::string tex_topic = tex_topic_prop_.getTopicStd();

    std::string tex_transport = tex_transport_prop_->getStdString();

    if (!mesh_topic.empty()) {
      // Subscribe to the mesh topic.
      mesh_filter_->subscribe(update_nh_, mesh_topic, queue_size_);
      mesh_filter_->registerCallback(
          boost::bind(&TexturedMeshDisplay::processPolygonMeshMessage, this, _1));

      if (!tex_topic.empty() && !tex_transport.empty()) {
        // Subscribe to texture topic.
        tex_filter_->subscribe(*tex_it_, tex_topic, queue_size_,
                               image_transport::TransportHints(tex_transport));
        tex_filter_->registerCallback(
            boost::bind(&TexturedMeshDisplay::processTextureMessage, this, _1));
      }
    }

    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  } catch(ros::Exception& e) {
    ROS_ERROR("Error subscribing: %s", e.what());
    setStatus(rviz::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  } catch (image_transport::TransportLoadException& e) {
    ROS_ERROR("Error subscribing: %s", e.what());
    setStatus(rviz::StatusProperty::Error, "Message",
              QString("Error subscribing: ") + e.what());
  } catch (...) {
    ROS_ERROR("Caught unknown exception!");
  }

  return;
}

void TexturedMeshDisplay::unsubscribe() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  try {
    mesh_filter_.reset();
    tex_filter_.reset();
  } catch (ros::Exception& e) {
    setStatus(rviz::StatusProperty::Error, "Message",
              QString("Error unsubscribing: ") + e.what());
  }

  return;
}

void TexturedMeshDisplay::onEnable() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  subscribe();
  return;
}

void TexturedMeshDisplay::onDisable() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  unsubscribe();
  reset();
  return;
}

void TexturedMeshDisplay::fixedFrameChanged() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // reset();

  return;
}

void TexturedMeshDisplay::
processTextureMessage(const sensor_msgs::Image::ConstPtr& tex_msg) {
  ROS_DEBUG("Got a texture message at %f!\n", tex_msg->header.stamp.toSec());
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  if (!tex_msg) {
    return;
  }

  // Push message onto queue.
  tex_queue_.push(tex_msg);

  // Enforce queue size.
  while (tex_queue_.size() > queue_size_) {
    tex_queue_.pop();
  }

  return;
}

void TexturedMeshDisplay::
processPolygonMeshMessage(const pcl_msgs::PolygonMesh::ConstPtr& msg) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  ROS_DEBUG("Got a mesh message at %f!\n", msg->header.stamp.toSec());

  if (!msg) {
    return;
  }

  // Push message onto queue.
  mesh_queue_.push(msg);

  // Enforce queue size.
  while (mesh_queue_.size() > queue_size_) {
    mesh_queue_.pop();
  }

  // Synchronize and process the texture and mesh messages.
  pcl_msgs::PolygonMesh::ConstPtr mesh_msg;
  sensor_msgs::Image::ConstPtr tex_msg;
  double tol = 5e-3; // 5 ms tolerance.
  while ((tex_queue_.size() > 0) && (mesh_queue_.size() > 0)) {
    double tex_time = tex_queue_.front()->header.stamp.toSec();
    double mesh_time = mesh_queue_.front()->header.stamp.toSec();

    if (std::fabs(tex_time - mesh_time) <= tol) {
      mesh_msg = mesh_queue_.front();
      tex_msg = tex_queue_.front();
      mesh_queue_.pop();
      tex_queue_.pop();
      break;
    } else {
      if (tex_time < mesh_time) {
        tex_queue_.pop();
      } else {
        mesh_queue_.pop();
      }
    }
  }

  if ((mesh_msg != nullptr) && (tex_msg != nullptr)) {
    ROS_DEBUG("Found a match!");
    processTexturedMeshMessages(mesh_msg, tex_msg);
  }

  ROS_DEBUG("Processed a mesh message!\n");

  return;
}

void TexturedMeshDisplay::
processTexturedMeshMessages(const pcl_msgs::PolygonMesh::ConstPtr& mesh_msg,
                            const sensor_msgs::Image::ConstPtr& tex_msg) {
  ROS_DEBUG("Got mesh and texture messages!\n");
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  if (!mesh_msg || !tex_msg) {
    return;
  }

  updateFixedFrameTransform(mesh_msg->header);

  num_meshes_received_++;
  setStatus(rviz::StatusProperty::Ok, "Topic",
            QString::number(num_meshes_received_) + " messages received");

  if (visual_ == nullptr) {
    Ogre::PolygonMode pm =
        static_cast<Ogre::PolygonMode>(polygon_mode_prop_.getOptionInt());
    TexturedMeshVisual::ShaderProgram sp =
        static_cast<TexturedMeshVisual::ShaderProgram>(shader_program_prop_.getOptionInt());

    visual_ = std::make_shared<TexturedMeshVisual>(scene_manager_,
                                                   getSceneNode(), pm, sp);
    visual_->setSceneColorScale(scene_color_scale_prop_.getFloat());
  }

  visual_->setFromMessage(mesh_msg, tex_msg);

  if (normals_ == nullptr) {
    normals_ = std::make_shared<SurfaceNormalsVisual>(scene_manager_,
                                                      getSceneNode(),
                                                      Ogre::ColourValue::Red,
                                                      normal_size_prop_.getFloat());
  }

  normals_->setFromMessage(mesh_msg);

  return;
}

void TexturedMeshDisplay::
updateFixedFrameTransform(const std_msgs::Header& header) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this message.
  // The fixed frame is the frame being displayed in RVIZ (e.g. FLU world).
  // The frame in the msg header should be the camera RDF world frame.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(header.frame_id,
                                                 header.stamp,
                                                 position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  getSceneNode()->setPosition(position);
  getSceneNode()->setOrientation(orientation);

  return;
}

}  // namespace flame_rviz_plugins

// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(flame_rviz_plugins::TexturedMeshDisplay, rviz::Display)
