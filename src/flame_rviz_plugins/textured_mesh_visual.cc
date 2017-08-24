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
 * @file textured_mesh_visual.cc
 * @author W. Nicholas Greene
 * @date 2017-02-21 20:53:05 (Tue)
 */

#include "flame_rviz_plugins/textured_mesh_visual.h"

#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreEntity.h>

#include <pcl_conversions/pcl_conversions.h>

namespace flame_rviz_plugins {

TexturedMeshVisual::TexturedMeshVisual(Ogre::SceneManager* scene_manager,
                                       Ogre::SceneNode* parent_node,
                                       Ogre::PolygonMode poly_mode,
                                       ShaderProgram shader_program) :
  Visual(scene_manager, parent_node),
  mesh_(),
  mesh_name_("textured_mesh_visual_mesh"),
  entity_(nullptr),
  entity_name_("textured_mesh_visual_entity"),
  mesh_material_(),
  material_name_("textured_mesh_visual_mesh_material"),
  tex_img_(),
  tex_name_("textured_mesh_mesh_texture"),
  mode_(poly_mode),
  mesh_visibility_(true),
  shader_program_(shader_program),
  scene_color_scale_(1.0f),
  phong_shading_(true),
  vtx_shader_(),
  texture_shader_(),
  idepth_shader_(),
  jet_shader_(),
  normal_shader_() {
  // Set ambient light.
  scene_manager->setAmbientLight(Ogre::ColourValue(1.0, 1.0, 1.0));

  // Create material.
  Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();
  Ogre::String resource_group =
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

  mesh_material_ = material_manager.create(material_name_, resource_group);

  Ogre::Technique* tech = mesh_material_->getTechnique(0);
  Ogre::Pass* pass = tech->getPass(0);
  pass->setLightingEnabled(false);
  pass->setPolygonMode(mode_);

  // pass->setAmbient(0.5, 0.5, 0.5);
  // pass->setDiffuse(0.5, 0.5, 0.5, 1.0);
  // // pass->setSpecular(1.0, 0.0, 0.0, 10.0);
  // pass->setShadingMode(Ogre::ShadeOptions::SO_PHONG);

  // Vertex shader.
  vtx_shader_ = Ogre::HighLevelGpuProgramManager::getSingletonPtr()->
      createProgram("vertex_shader",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    "glsl", Ogre::GpuProgramType::GPT_VERTEX_PROGRAM);
  vtx_shader_->setSource(vtx_shader_src_);
  vtx_shader_->load();
  pass->setVertexProgram(vtx_shader_->getName());

  auto vparams = pass->getVertexProgramParameters();
  vparams->setNamedAutoConstant("MVP",
    Ogre::GpuProgramParameters::AutoConstantType::ACT_WORLDVIEWPROJ_MATRIX);
  vparams->setNamedAutoConstant("MV",
    Ogre::GpuProgramParameters::AutoConstantType::ACT_WORLDVIEW_MATRIX);
  vparams->setNamedAutoConstant("MVinvT",
    Ogre::GpuProgramParameters::AutoConstantType::ACT_INVERSE_TRANSPOSE_WORLDVIEW_MATRIX);

  // Texture fragment shader.
  texture_shader_ = Ogre::HighLevelGpuProgramManager::getSingletonPtr()->
      createProgram("texture_shader",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    "glsl", Ogre::GpuProgramType::GPT_FRAGMENT_PROGRAM);
  texture_shader_->setSource(texture_shader_src_);
  texture_shader_->load();

  // Inverse depth fragment shader.
  idepth_shader_ = Ogre::HighLevelGpuProgramManager::getSingletonPtr()->
      createProgram("idepth_shader",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    "glsl", Ogre::GpuProgramType::GPT_FRAGMENT_PROGRAM);
  idepth_shader_->setSource(idepth_shader_src_);
  idepth_shader_->load();

  // Jet fragment shader.
  jet_shader_ = Ogre::HighLevelGpuProgramManager::getSingletonPtr()->
      createProgram("jet_shader",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    "glsl", Ogre::GpuProgramType::GPT_FRAGMENT_PROGRAM);
  jet_shader_->setSource(jet_shader_src_);
  jet_shader_->load();

  // Surface normal fragment shader.
  normal_shader_ = Ogre::HighLevelGpuProgramManager::getSingletonPtr()->
      createProgram("normal_shader",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    "glsl", Ogre::GpuProgramType::GPT_FRAGMENT_PROGRAM);
  normal_shader_->setSource(normal_shader_src_);
  normal_shader_->load();

  // Set default to texture shader.
  pass->setFragmentProgram(texture_shader_->getName());
  auto fparams = pass->getFragmentProgramParameters();
  fparams->setNamedAutoConstant("ambient_color",
                                Ogre::GpuProgramParameters::AutoConstantType::ACT_AMBIENT_LIGHT_COLOUR);
  fparams->setNamedAutoConstant("diffuse_color",
                                Ogre::GpuProgramParameters::AutoConstantType::ACT_LIGHT_DIFFUSE_COLOUR);
  fparams->setNamedAutoConstant("specular_color",
                                Ogre::GpuProgramParameters::AutoConstantType::ACT_LIGHT_SPECULAR_COLOUR);
  fparams->setNamedAutoConstant("light_dir",
                                Ogre::GpuProgramParameters::AutoConstantType::ACT_LIGHT_DIRECTION_VIEW_SPACE);
  fparams->setNamedConstant("scene_color_scale", scene_color_scale_);
  fparams->setNamedConstant("phong_shading", phong_shading_ ? 1 : 0);

  return;
}

TexturedMeshVisual::~TexturedMeshVisual() {
  std::lock_guard<std::mutex> lock(*getMutex());

  if (entity_ != nullptr) {
    getSceneNode()->detachObject(entity_);
    getSceneManager()->destroyEntity(entity_);
  }

  if (!mesh_material_.isNull()) {
    mesh_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    Ogre::MaterialManager::getSingleton().remove(material_name_);
  }

  if (!mesh_.isNull()) {
    Ogre::MeshManager::getSingleton().remove(mesh_->getHandle());
    mesh_.setNull();
  }

  return;
}

void TexturedMeshVisual::setShaderProgram(ShaderProgram shader_program) {
  std::lock_guard<std::mutex> lock(*getMutex());
  shader_program_ = shader_program;

  if (mesh_material_.isNull()) {
    return;
  }

  Ogre::Pass* pass  = mesh_material_->getTechnique(0)->getPass(0);

  if (shader_program_ == ShaderProgram::TEXTURE) {
    pass->setFragmentProgram(texture_shader_->getName());
    auto fparams = pass->getFragmentProgramParameters();
    fparams->setNamedConstant("scene_color_scale", scene_color_scale_);
    fparams->setNamedConstant("phong_shading", phong_shading_ ? 1 : 0);
    updateTexture(mesh_material_, tex_img_);
  } else if (shader_program_ == ShaderProgram::INVERSE_DEPTH) {
    pass->setFragmentProgram(idepth_shader_->getName());
    pass->removeAllTextureUnitStates();
    auto fparams = pass->getFragmentProgramParameters();
    fparams->setNamedConstant("scene_color_scale", scene_color_scale_);
    fparams->setNamedConstant("phong_shading", phong_shading_ ? 1 : 0);
  } else if (shader_program_ == ShaderProgram::JET) {
    pass->setFragmentProgram(jet_shader_->getName());
    pass->removeAllTextureUnitStates();
    auto fparams = pass->getFragmentProgramParameters();
    fparams->setNamedConstant("scene_color_scale", scene_color_scale_);
    fparams->setNamedConstant("phong_shading", phong_shading_ ? 1 : 0);
  } else if (shader_program_ == ShaderProgram::SURFACE_NORMAL) {
    pass->setFragmentProgram(normal_shader_->getName());
    pass->removeAllTextureUnitStates();
    auto fparams = pass->getFragmentProgramParameters();
    fparams->setNamedConstant("phong_shading", phong_shading_ ? 1 : 0);
  } else {
    ROS_WARN("Unrecognized ShaderProgram!");
    return;
  }

  return;
}

// Adapted from:
// https://grahamedgecombe.com/blog/custom-meshes-in-ogre3d
// http://www.ogre3d.org/tikiwiki/Generating+A+Mesh
void TexturedMeshVisual::
setFromMessage(const pcl_msgs::PolygonMesh::ConstPtr& mesh_msg,
               const sensor_msgs::Image::ConstPtr& tex_msg) {
  std::lock_guard<std::mutex> lock(*getMutex());

  ROS_DEBUG("Updating mesh!");

  if (mesh_msg->cloud.row_step !=
      mesh_msg->cloud.point_step * mesh_msg->cloud.width) {
    ROS_WARN("Row padding not supported!\n");
    return;
  }

  /*==================== Update mesh geometry. ====================*/
  if (mesh_.isNull()) {
    // Create mesh and submesh.
    createMesh(mesh_msg->cloud.fields);
  }

  updateVertexBuffer(mesh_, mesh_msg->cloud);
  updateIndexBuffer(mesh_, mesh_msg->polygons);

  mesh_->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100, 100, 100, 100));
  mesh_->load();

  /*==================== Update mesh texture. ====================*/
  if ((shader_program_ == ShaderProgram::TEXTURE) && (tex_msg != nullptr)) {
    // Decompress image.
    tex_img_ = cv_bridge::toCvCopy(tex_msg, "rgb8")->image;
    updateTexture(mesh_material_, tex_img_);
  } else if ((shader_program_ == ShaderProgram::TEXTURE) && (tex_msg == nullptr)) {
    ROS_ERROR("ShaderProgram set to TEXTURE, but texture message is NULL!");
    return;
  }

  /*==================== Create instance and attach. ====================*/
  if (entity_ == nullptr) {
    Ogre::String resource_group =
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

    entity_ = getSceneManager()->createEntity(entity_name_, mesh_name_,
                                              resource_group);
    entity_->setMaterialName(material_name_, resource_group);
    getSceneNode()->attachObject(entity_);
  }

  getSceneNode()->setVisible(mesh_visibility_);

  ROS_DEBUG("Updated mesh!");

  return;
}

void TexturedMeshVisual::createMesh(const std::vector<sensor_msgs::PointField>& fields) {
  Ogre::String resource_group =
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

  mesh_ = Ogre::MeshManager::getSingleton().createManual(mesh_name_,
                                                         resource_group);
  mesh_->createSubMesh();
  mesh_->sharedVertexData = new Ogre::VertexData(); // Destroyed by mesh.

  // Declare vertex data.
  Ogre::VertexDeclaration* vtx_dec = mesh_->sharedVertexData->vertexDeclaration;

  // Loop through point fields and add vertex elements.
  int field_idx = 0;
  while (field_idx < fields.size()) {
    std::string name = fields[field_idx].name;

    if (name == "x") {
      // Assume next two fields are yz.
      vtx_dec->addElement(0, fields[field_idx].offset,
                          Ogre::VET_FLOAT3, Ogre::VES_POSITION);
      field_idx += 3;
    } else if (name == "normal_x") {
      // Assume nexto two fields are normal_y and normal_z.
      vtx_dec->addElement(0, fields[field_idx].offset,
                          Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
      field_idx += 3;
    } else if (name == "u") {
      // Assume next field is v.
      vtx_dec->addElement(0, fields[field_idx].offset,
                          Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);
      field_idx += 2;
    } else {
      field_idx++;
    }
  }

  return;
}

void TexturedMeshVisual::
updateVertexBuffer(const Ogre::MeshPtr& mesh,
                   const sensor_msgs::PointCloud2& cloud) {
  Ogre::HardwareVertexBufferSharedPtr vtx_buffer;

  if (mesh->sharedVertexData->vertexBufferBinding->getBufferCount() > 0) {
    vtx_buffer = mesh->sharedVertexData->vertexBufferBinding->getBuffer(0);
  }

  if ((vtx_buffer.isNull()) || (vtx_buffer->getSizeInBytes() < cloud.data.size())) {
    // Create a new vertex buffer.
    Ogre::HardwareBufferManager& hw_manager =
        Ogre::HardwareBufferManager::getSingleton();
    vtx_buffer = hw_manager.createVertexBuffer(cloud.point_step,
                                               cloud.height * cloud.width,
                                               Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vtx_buffer);
  }

  // Copy data.
  vtx_buffer->writeData(0, cloud.data.size(), cloud.data.data(), true);

  // Update listed number of vertices.
  mesh->sharedVertexData->vertexCount = cloud.height * cloud.width;

  return;
}

void TexturedMeshVisual::updateIndexBuffer(const Ogre::MeshPtr& mesh,
                                           const std::vector<pcl_msgs::Vertices>& indices) {
  Ogre::SubMesh* sub_mesh = mesh->getSubMesh(0);
  sub_mesh->useSharedVertices = true;
  sub_mesh->indexData->indexCount = indices.size() * 3;
  sub_mesh->indexData->indexStart = 0;

  auto idx_buffer = sub_mesh->indexData->indexBuffer;

  if (idx_buffer.isNull() ||
      (idx_buffer->getNumIndexes() < sub_mesh->indexData->indexCount)) {
    // Create new index buffer.
    Ogre::HardwareBufferManager& hw_manager =
        Ogre::HardwareBufferManager::getSingleton();
    idx_buffer =
        hw_manager.createIndexBuffer(Ogre::HardwareIndexBuffer::IT_32BIT,
                                     indices.size() * 3,
                                     Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    sub_mesh->indexData->indexBuffer = idx_buffer;
  }

  // Lock buffer and grab pointer to data.
  uint32_t* idx_data =
    static_cast<uint32_t*>(idx_buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL));

  // Copy index data.
  for (int ii = 0; ii < indices.size(); ++ii) {
    idx_data[3*ii] = indices[ii].vertices[0];
    idx_data[3*ii + 1] = indices[ii].vertices[1];
    idx_data[3*ii + 2] = indices[ii].vertices[2];
  }

  // Unlock.
  idx_buffer->unlock();

  return;
}

void TexturedMeshVisual::updateTexture(const Ogre::MaterialPtr& material,
                                       const cv::Mat3b& tex_img) {
  Ogre::Image ogre_img;
  ogre_img.loadDynamicImage(tex_img.data, tex_img.cols, tex_img.rows,
                            Ogre::PixelFormat::PF_B8G8R8);

  Ogre::String resource_group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::TextureManager* tex_man = Ogre::TextureManager::getSingletonPtr();

  Ogre::TexturePtr tex = tex_man->getByName(tex_name_, resource_group);
  if (!tex.isNull()) {
    // Delete old texture.
    tex_man->remove(tex_name_);
  }

  tex = tex_man->loadImage(tex_name_, resource_group, ogre_img);

  Ogre::Pass* pass  = material->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = nullptr;
  if (pass->getNumTextureUnitStates() == 0) {
    // Create texture unit state.
    tex_unit = pass->createTextureUnitState();
  } else {
    tex_unit = pass->getTextureUnitState(0);
  }

  tex_unit->setTexture(tex);

  return;
}

}  // namespace flame_rviz_plugins
