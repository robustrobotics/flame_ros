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
 * @file surface_normals_visual.cc
 * @author W. Nicholas Greene
 * @date 2017-03-25 19:22:59 (Sat)
 */

#include "flame_rviz_plugins/surface_normals_visual.h"

#include <string>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
// #include <OGRE/OgreRenderOperation.h>

namespace flame_rviz_plugins {

SurfaceNormalsVisual::SurfaceNormalsVisual(Ogre::SceneManager* scene_manager,
                                           Ogre::SceneNode* parent_node,
                                           Ogre::ColourValue color,
                                           float normal_size) :
    Visual(scene_manager, parent_node),
    mobject_(scene_manager->createManualObject()),
    color_(color),
    normal_size_(normal_size) {
  scene_node->attachObject(mobject_.get());

  mobject_->setVisible(false);
  mobject_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  mobject_->position(0, 0, 0);
  mobject_->colour(Ogre::ColourValue::Red);
  mobject_->position(5, 0, 0);
  mobject_->colour(Ogre::ColourValue::Red);
  mobject_->end();

  return;
}

void SurfaceNormalsVisual::setFromMessage(const pcl_msgs::PolygonMesh::ConstPtr& msg) {
  setFromMessage(*msg);
  return;
}

void SurfaceNormalsVisual::setFromMessage(const pcl_msgs::PolygonMesh& msg) {
  std::lock_guard<std::mutex> lock(*Visual::getMutex());

  // Grab offset of position and normals in buffer.
  int pos_offset = 0;
  int normal_offset = 0;
  for (int ii = 0; ii < msg.cloud.fields.size(); ++ii) {
    std::string name = msg.cloud.fields[ii].name;
    if (name == "x") {
      // Assume next two fields are yz.
      pos_offset = msg.cloud.fields[ii].offset;
    } else if (name == "normal_x") {
      // Assume nexto two fields are normal_y and normal_z.
      normal_offset = msg.cloud.fields[ii].offset;
    }
  }

  // Walk over vertices and draw.
  mobject_->beginUpdate(0);

  for (int ii = 0; ii < msg.cloud.height; ++ii) {
    for (int jj = 0; jj < msg.cloud.width; ++jj) {
      // Grab position.
      float xyz[3];
      int poffset = ii*msg.cloud.row_step + jj*msg.cloud.point_step + pos_offset;
      memcpy(&xyz, &(msg.cloud.data[poffset]), sizeof(float) * 3);

      // Grab normal.
      float nxyz[3];
      int noffset = ii*msg.cloud.row_step + jj*msg.cloud.point_step + normal_offset;
      memcpy(&nxyz, &(msg.cloud.data[noffset]), sizeof(float) * 3);

      // Draw.
      mobject_->position(xyz[0], xyz[1], xyz[2]);
      mobject_->colour(color_);

      mobject_->position(normal_size_*nxyz[0] + xyz[0],
                         normal_size_*nxyz[1] + xyz[1],
                         normal_size_*nxyz[2] + xyz[2]);
    }
  }

  mobject_->end();

  return;
}

}  // namespace flame_rviz_plugins
