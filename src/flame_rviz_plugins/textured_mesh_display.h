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
 * @file textured_mesh_display.h
 * @author W. Nicholas Greene
 * @date 2017-02-21 15:49:11 (Tue)
 */

#pragma once

#ifndef Q_MOC_RUN
#include <string>
#include <set>
#include <memory>
#include <mutex>
#include <queue>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>

#include <rviz/display.h>

#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/Image.h>

#endif

namespace flame_rviz_plugins {

using rviz::EnumProperty; // Have to do this so that Qt can find the slot.

class TexturedMeshVisual;
class SurfaceNormalsVisual;

/**
 * \brief Class that displays a textured mesh.
 *
 * Draws a triangle mesh defined in a pcl_msgs/PolygonMesh message. An image
 * topic can also be provided to be used as a texture.
 *
 * The PointCloud2 inside the PolygonMesh should have xyz fields defined as
 * floats and normal_xyz fields (also as floats). Texture coordinates are can be
 * supplied with uv fields (also as floats).
 *
 * The mesh can be shaded with different fragment shader programs (with and
 * without Phong shading) depending on your preferences.
 */
class TexturedMeshDisplay: public rviz::Display {
 Q_OBJECT // NOLINT(whitespace/indent)

 public:
  TexturedMeshDisplay();

  void onInitialize();

  virtual ~TexturedMeshDisplay();

  void reset();

 private Q_SLOTS: // NOLINT(whitespace/indent)
  void updateTopic();
  void updateQueueSize();
  void updatePolygonMode();
  void updateShaderProgram();
  void updatePhongShading();
  void updateSceneColorScale();
  void updateShowNormals();
  void updateNormalSize();

  /**
   * @brief Fill list of available and working transport options.
   * Copied from rviz::DepthCloudDisplay.
   */
  void fillTransportOptionList(EnumProperty* property);

 protected:
  // Scans for available transport plugins. Copied from rviz::DepthCloudDisplay.
  void scanForTransportSubscriberPlugins();

  void subscribe();
  void unsubscribe();

  void onEnable();
  void onDisable();

  void fixedFrameChanged();
  void updateFixedFrameTransform(const std_msgs::Header& header);

  void processTextureMessage(const sensor_msgs::Image::ConstPtr& tex_msg);
  void processPolygonMeshMessage(const pcl_msgs::PolygonMesh::ConstPtr& msg);
  void processTexturedMeshMessages(const pcl_msgs::PolygonMesh::ConstPtr& mesh_msg,
                                   const sensor_msgs::Image::ConstPtr& img_msg);

  // Subscribes 'n stuff.
  std::shared_ptr<message_filters::Subscriber<pcl_msgs::PolygonMesh> > mesh_filter_;
  uint32_t num_meshes_received_;

  std::shared_ptr<image_transport::ImageTransport> tex_it_;
  std::shared_ptr<image_transport::SubscriberFilter> tex_filter_;

  // Internal message queues used to synchronize meshes and textures.
  std::queue<pcl_msgs::PolygonMesh::ConstPtr> mesh_queue_;
  std::queue<sensor_msgs::Image::ConstPtr> tex_queue_;

  // Main visual object that draws stuff.
  std::shared_ptr<TexturedMeshVisual> visual_;

  // Surface normals visual.
  std::shared_ptr<SurfaceNormalsVisual> normals_;

  // RViz properties 'n stuff.
  rviz::RosTopicProperty mesh_topic_prop_;
  rviz::RosTopicProperty tex_topic_prop_;
  std::shared_ptr<rviz::EnumProperty> tex_transport_prop_;
  rviz::EnumProperty polygon_mode_prop_;
  rviz::EnumProperty shader_program_prop_;
  rviz::BoolProperty phong_shading_prop_;
  rviz::FloatProperty scene_color_scale_prop_;
  rviz::BoolProperty show_normals_prop_;
  rviz::FloatProperty normal_size_prop_;
  rviz::IntProperty queue_size_prop_;

  int queue_size_;
  std::set<std::string> transport_plugin_types_;

  std::recursive_mutex mtx_;
};

}  // end namespace flame_rviz_plugins
