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
 * @file visual.h
 * @author W. Nicholas Greene
 * @date 2017-08-21 21:47:01 (Mon)
 */

#pragma once

#include <mutex>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

namespace flame_rviz_plugins {

/**
 * \brief Base class for RVIZ visuals.
 */
class Visual {
 public:
  Visual(Ogre::SceneManager* scene_manager,
         Ogre::SceneNode* parent_node) :
    scene_manager(scene_manager),
    scene_node(parent_node->createChildSceneNode()) {}

  virtual ~Visual() {}

  inline Ogre::SceneManager* getSceneManager() {
    return scene_manager;
  }

  inline const Ogre::SceneManager* getSceneManager() const {
    return scene_manager;
  }

  inline Ogre::SceneNode* getSceneNode() {
    return scene_node;
  }

  inline const Ogre::SceneNode* getSceneNode() const {
    return scene_node;
  }

  inline std::mutex* getMutex() {
    return &mtx;
  }

 protected:
  Ogre::SceneManager* scene_manager;
  Ogre::SceneNode* scene_node;
  std::mutex mtx;
};


}  // namespace flame_rviz_plugins
