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
 * @file surface_normals_visual.h
 * @author W. Nicholas Greene
 * @date 2017-03-25 19:18:48 (Sat)
 */

#pragma once

#include <memory>
#include <vector>

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreManualObject.h>

#include <pcl_msgs/PolygonMesh.h>

#include <flame_rviz_plugins/visual.h>

namespace Ogre {
class SceneNode;
class SceneManager;
}  // namespace Ogre

namespace flame_rviz_plugins {

/**
 * \brief Draws surface normals as lines.
 */
class SurfaceNormalsVisual : public Visual {
 public:
/**
 * \brief Constructor.
 *
 * @param scene_manager[in] The current scene manager.
 * @param parent_node[in] The parent node. Only used to create a child node for this visual.
 * @param color[in] Color of the frustum.
 */
  SurfaceNormalsVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                       Ogre::ColourValue color = Ogre::ColourValue::Red,
                       float normal_size = 0.05f);

  virtual ~SurfaceNormalsVisual() = default;

  /**
   * \brief Set the frustum from a message.
   *
   * @param msg[in] View message.
   */
  void setFromMessage(const pcl_msgs::PolygonMesh::ConstPtr& msg);
  void setFromMessage(const pcl_msgs::PolygonMesh& msg);

  /**
   * @brief Set the size of the normal vectors.
   */
  void setNormalSize(float size) {
    std::lock_guard<std::mutex> lock(*Visual::getMutex());
    normal_size_ = size;
    return;
  }

  /**
   * @brief Set visibility of normals.
   */
  void setVisible(bool visible) {
    std::lock_guard<std::mutex> lock(*Visual::getMutex());
    mobject_->setVisible(visible);
    return;
  }

 private:
  std::shared_ptr<Ogre::ManualObject> mobject_;
  Ogre::ColourValue color_;
  float normal_size_;
};

}  // namespace flame_rviz_plugins
