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
 * @file textured_mesh_visual.h
 * @author W. Nicholas Greene
 * @date 2017-02-21 20:57:52 (Tue)
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <OGRE/OgreHighLevelGpuProgramManager.h>

#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/Image.h>

#include <flame_rviz_plugins/visual.h>

namespace Ogre {
class SceneNode;
class SceneManager;
class SubMesh;
class TextureUnitState;
}  // namespace Ogre

namespace flame_rviz_plugins {

/**
 * @brief Class that draws a triangle mesh.
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
class TexturedMeshVisual final : public Visual {
 public:
  /**
   * @Brief Shader program for the mesh.
   */
  enum ShaderProgram {
    TEXTURE, // Use texture.
    INVERSE_DEPTH, // Color by inverse depth.
    JET, // Color by inverse depth using jet colormap.
    SURFACE_NORMAL // Color by surface normal.
  };

  /**
   * @brief Constructor.
   *
   * @param[in] scene_manager The Ogre scene manager.
   * @param[in] parent_node The parent scene node in the Ogre scene graph.
   * @param[in] poly_mode The default polygon mode to draw the mesh.
   * @param[in] shader_program The default fragment shader program to draw the mesh.
   */
  TexturedMeshVisual(Ogre::SceneManager* scene_manager,
                     Ogre::SceneNode* parent_node,
                     Ogre::PolygonMode poly_mode = Ogre::PM_WIREFRAME,
                     ShaderProgram shader_program = ShaderProgram::INVERSE_DEPTH);
  ~TexturedMeshVisual();

  TexturedMeshVisual(const TexturedMeshVisual& rhs) = delete;
  TexturedMeshVisual& operator=(const TexturedMeshVisual& rhs) = delete;

  TexturedMeshVisual(const TexturedMeshVisual&& rhs) = delete;
  TexturedMeshVisual& operator=(const TexturedMeshVisual&& rhs) = delete;

  /**
   * @brief Draw mesh defined by PolygonMesh.
   *
   * @param msg[in] Incoming message.
   */
  void setFromMessage(const pcl_msgs::PolygonMesh::ConstPtr& msg,
                      const sensor_msgs::Image::ConstPtr& tex_msg = nullptr);

  /**
   * @brief Set mesh visibility.
   *
   * @param new_vis[in] Desired visibility.
   */
  void setMeshVisibility(const bool new_vis) {
    std::lock_guard<std::mutex> lock(*getMutex());
    mesh_visibility_ = new_vis;
    getSceneNode()->setVisible(new_vis);
    return;
  }

  /**
   * @brief Set the polygon drawing mode.
   *
   * Either points, wireframe, or solid mesh.
   *
   * @param mode[in] New mode.
   */
  void setPolygonMode(Ogre::PolygonMode mode) {
    std::lock_guard<std::mutex> lock(*getMutex());
    this->mode_ = mode;
    if (mesh_material_.get() != nullptr) {
      mesh_material_->getTechnique(0)->getPass(0)->setPolygonMode(mode_);
    }
    return;
  }

  /**
   * @brief Set the scene color scale for jet shader.
   */
  void setSceneColorScale(float scale) {
    std::lock_guard<std::mutex> lock(*getMutex());
    scene_color_scale_ = scale;

    if (mesh_material_.isNull()) {
      return;
    }

    Ogre::Pass* pass  = mesh_material_->getTechnique(0)->getPass(0);
    auto fparams = pass->getFragmentProgramParameters();
    fparams->setNamedConstant("scene_color_scale", scene_color_scale_);

    return;
  }

  /**
   * @brief Enable or disable Phong Shading.
   */
  void setPhongShading(bool phong_shading) {
    std::lock_guard<std::mutex> lock(*getMutex());
    phong_shading_ = phong_shading;

    if (mesh_material_.isNull()) {
      return;
    }

    Ogre::Pass* pass  = mesh_material_->getTechnique(0)->getPass(0);
    auto fparams = pass->getFragmentProgramParameters();
    fparams->setNamedConstant("phong_shading", phong_shading_ ? 1 : 0);

    return;
  }

  /**
   * @brief Set the shader for the mesh.
   */
  void setShaderProgram(ShaderProgram shader_program);

 private:
  // Basic vertex shader.
  // This Converts each vertex into OpenGL clip coordinates (multiplication with
  // the MVP matrix) and forwards other information to the fragment shader
  // including the position of each fragment in "view coordinates"
  // (multiplication with the MV matrix), the outward surface normal for each
  // fragment in view coordinates (multiplication with the transpose of the
  // inverse of the MV matrix), and the depth of each fragment. Constants to the
  // shader are defined as uniforms, while inputs and outputs are defined as ins
  // and outs (per vertex and per fragment).
  // See here for tutorial: https://learnopengl.com/#!Lighting/Basic-Lighting
  const std::string vtx_shader_src_ =
      "#version 330 core\n"
      "uniform mat4 MVP;\n"
      "uniform mat4 MV;\n"
      "uniform mat4 MVinvT;\n"
      "in vec4 vertex;\n"
      "in vec3 normal;\n"
      "in vec2 uv0;\n"
      "out vec4 frag_pos;\n"
      "out vec3 frag_normal;\n"
      "out float frag_depth;\n"
      "out vec2 out_texcoord;\n"
      "void main() {\n"
      "  frag_pos = MV * vertex;\n"
      "  frag_normal = vec3(MVinvT * vec4(normal, 0.0));\n"
      "  frag_depth = vertex.z;\n"
      "  gl_Position = MVP * vertex;\n"
      "  out_texcoord = uv0;\n"
      "}";

  // Fragment shader using image texture.
  // Optionally performs phong shading.
  // See here for tutorial: https://learnopengl.com/#!Lighting/Basic-Lighting
  const std::string texture_shader_src_ =
      "#version 330 core\n"
      "uniform float scene_color_scale;\n"
      "uniform int phong_shading;\n"
      "uniform vec3 ambient_color;\n"
      "uniform vec3 diffuse_color;\n"
      "uniform vec3 specular_color;\n"
      "uniform vec3 light_dir;\n"
      "uniform sampler2D tex_sampler;\n"
      "in vec4 frag_pos;\n"
      "in vec3 frag_normal;\n"
      "in float frag_depth;\n"
      "in vec2 out_texcoord;\n"
      "out vec4 frag_color;\n"
      ""
      "void main() {\n"
      "  // Color from colormap.\n"
      "  vec3 rgb = texture2D(tex_sampler, out_texcoord).xyz;\n"
      ""
      "  // Ambient color.\n"
      "  vec3 acolor = ambient_color;\n"
      ""
      "  // Diffuse color.\n"
      "  float dfactor = max(dot(frag_normal, -light_dir), 0.0);\n"
      "  vec3 dcolor = dfactor * diffuse_color;\n"
      ""
      "  // Specular color.\n"
      "  vec3 view_dir = normalize(-frag_pos.xyz);\n"
      "  vec3 reflect_dir = reflect(light_dir, frag_normal);\n"
      "  int shininess = 32;"
      "  float sfactor = pow(max(dot(view_dir, reflect_dir), 0.0), shininess);\n"
      "  vec3 scolor = sfactor * specular_color;\n"
      ""
      "  if (phong_shading > 0) {\n"
      "    frag_color = vec4((0.7*acolor + 0.2*dcolor + 0.1*scolor) * rgb, 1.0); \n"
      "  } else {\n"
      "    frag_color = vec4(rgb, 1.0);\n"
      "  }\n"
      "}";

  // Fragment shader using grayscale inverse depth color scheme.
  // Optionally performs phong shading.
  // See here for tutorial: https://learnopengl.com/#!Lighting/Basic-Lighting
  const std::string idepth_shader_src_ =
      "#version 330 core\n"
      "uniform float scene_color_scale;\n"
      "uniform int phong_shading;\n"
      "uniform vec3 ambient_color;\n"
      "uniform vec3 diffuse_color;\n"
      "uniform vec3 specular_color;\n"
      "uniform vec3 light_dir;\n"
      "in vec4 frag_pos;\n"
      "in vec3 frag_normal;\n"
      "in float frag_depth;\n"
      "out vec4 frag_color;\n"
      ""
      "void main() {\n"
      "  // Color from colormap.\n"
      "  vec3 cmap = scene_color_scale * vec3(1.0/frag_depth, 1.0/frag_depth, 1.0/frag_depth);\n"
      ""
      "  // Ambient color.\n"
      "  vec3 acolor = ambient_color;\n"
      ""
      "  // Diffuse color.\n"
      "  float dfactor = max(dot(frag_normal, -light_dir), 0.0);\n"
      "  vec3 dcolor = dfactor * diffuse_color;\n"
      ""
      "  // Specular color.\n"
      "  vec3 view_dir = normalize(-frag_pos.xyz);\n"
      "  vec3 reflect_dir = reflect(light_dir, frag_normal);\n"
      "  int shininess = 32;"
      "  float sfactor = pow(max(dot(view_dir, reflect_dir), 0.0), shininess);\n"
      "  vec3 scolor = sfactor * specular_color;\n"
      ""
      "  if (phong_shading > 0) {\n"
      "    frag_color = vec4((0.7*acolor + 0.2*dcolor + 0.1*scolor) * cmap, 1.0);\n"
      "  } else {\n"
      "    frag_color = vec4(cmap, 1.0);\n"
      "  }\n"
      "}";

  // Fragment shader using jet colormap applied to inverse depth.
  // Optionally performs phong shading.
  // See here for tutorial: https://learnopengl.com/#!Lighting/Basic-Lighting
  const std::string jet_shader_src_ =
      "#version 330 core\n"
      "uniform float scene_color_scale;\n"
      "uniform int phong_shading;\n"
      "uniform vec3 ambient_color;\n"
      "uniform vec3 diffuse_color;\n"
      "uniform vec3 specular_color;\n"
      "uniform vec3 light_dir;\n"
      "in vec4 frag_pos;\n"
      "in vec3 frag_normal;\n"
      "in float frag_depth;\n"
      "out vec4 frag_color;\n"
      ""
      "vec3 jet(float v, float vmin, float vmax) {\n"
      "  vec3 c = vec3(255, 255, 255);  // white\n"
      "  float dv;\n"
      ""
      "  if (v < vmin)\n"
      "    v = vmin;\n"
      "  if (v > vmax)\n"
      "    v = vmax;\n"
      "  dv = vmax - vmin;\n"
      ""
      "  if (v < (vmin + 0.25 * dv)) {\n"
      "    c[0] = 0;\n"
      "    c[1] = 255 * (4 * (v - vmin) / dv);\n"
      "  } else if (v < (vmin + 0.5 * dv)) {\n"
      "    c[0] = 0;\n"
      "    c[2] = 255 * (1 + 4 * (vmin + 0.25 * dv - v) / dv);\n"
      "  } else if (v < (vmin + 0.75 * dv)) {\n"
      "    c[0] = 255 * (4 * (v - vmin - 0.5 * dv) / dv);\n"
      "    c[2] = 0;\n"
      "  } else {\n"
      "    c[1] = 255 * (1 + 4 * (vmin + 0.75 * dv - v) / dv);\n"
      "    c[2] = 0;\n"
      "  }\n"
      "  return c;\n"
      "}\n"
      ""
      "void main() {\n"
      "  // Color from colormap.\n"
      "  vec3 cmap = jet(scene_color_scale * 1.0/frag_depth, 0.0, 2.0)/255;\n"
      ""
      "  // Ambient color.\n"
      "  vec3 acolor = ambient_color;\n"
      ""
      "  // Diffuse color.\n"
      "  float dfactor = max(dot(frag_normal, -light_dir), 0.0);\n"
      "  vec3 dcolor = dfactor * diffuse_color;\n"
      ""
      "  // Specular color.\n"
      "  vec3 view_dir = normalize(-frag_pos.xyz);\n"
      "  vec3 reflect_dir = reflect(light_dir, frag_normal);\n"
      "  int shininess = 32;"
      "  float sfactor = pow(max(dot(view_dir, reflect_dir), 0.0), shininess);\n"
      "  vec3 scolor = sfactor * specular_color;\n"
      ""
      "  if (phong_shading > 0) {\n"
      "    frag_color = vec4((0.7*acolor + 0.2*dcolor + 0.1*scolor) * cmap, 1.0);\n"
      "  } else {\n"
      "    frag_color = vec4(cmap, 1.0);\n"
      "  }\n"
      "}";

  // Fragment shader using normal vector color scheme.
  // Optionally performs phong shading.
  // See here for tutorial: https://learnopengl.com/#!Lighting/Basic-Lighting
  const std::string normal_shader_src_ =
      "#version 330 core\n"
      "uniform int phong_shading;\n"
      "uniform vec3 ambient_color;\n"
      "uniform vec3 diffuse_color;\n"
      "uniform vec3 specular_color;\n"
      "uniform vec3 light_dir;\n"
      "in vec4 frag_pos;\n"
      "in vec3 frag_normal;\n"
      "in float frag_depth;\n"
      "out vec4 frag_color;\n"
      "void main() {\n"
      "  // Color from colormap.\n"
      "  vec3 cmap = vec3((frag_normal.x + 1)/2, (frag_normal.y + 1)/2, (127 * frag_normal.z + 127)/255);\n"
      ""
      "  // Ambient color.\n"
      "  vec3 acolor = ambient_color;\n"
      ""
      "  // Diffuse color.\n"
      "  float dfactor = max(dot(frag_normal, -light_dir), 0.0);\n"
      "  vec3 dcolor = dfactor * diffuse_color;\n"
      ""
      "  // Specular color.\n"
      "  vec3 view_dir = normalize(-frag_pos.xyz);\n"
      "  vec3 reflect_dir = reflect(light_dir, frag_normal);\n"
      "  int shininess = 32;"
      "  float sfactor = pow(max(dot(view_dir, reflect_dir), 0.0), shininess);\n"
      "  vec3 scolor = sfactor * specular_color;\n"
      ""
      "  if (phong_shading > 0) {\n"
      "    frag_color = vec4((0.7*acolor + 0.2*dcolor + 0.1*scolor) * cmap, 1.0);\n"
      "  } else {\n"
      "    frag_color = vec4(cmap, 1.0);\n"
      "  }\n"
      "}";

  // Methods to create and update the mesh.
  void createMesh(const std::vector<sensor_msgs::PointField>& fields);
  void updateVertexBuffer(const Ogre::MeshPtr& mesh,
                          const sensor_msgs::PointCloud2& cloud);
  void updateIndexBuffer(const Ogre::MeshPtr& mesh,
                         const std::vector<pcl_msgs::Vertices>& indices);
  void updateTexture(const Ogre::MaterialPtr& material, const cv::Mat3b& tex_img);

  Ogre::MeshPtr mesh_; // Main mesh object.
  std::string mesh_name_; // Ogre string for this mesh.

  Ogre::Entity* entity_; // An instance of a mesh in the scene is called an entity.
  std::string entity_name_; // Ogre string for this entity.

  Ogre::MaterialPtr mesh_material_; // Material for the mesh (how it's textured/lit/shaded/etc).
  std::string material_name_; // Ogre string for this material.

  cv::Mat3b tex_img_; // Texture image.
  std::string tex_name_; // Texture name.

  Ogre::PolygonMode mode_; // Whether to draw as points, wireframe, or solid.

  bool mesh_visibility_; // True if mesh should be visiable.

  float scene_color_scale_; // Parameter for color scale.
  bool phong_shading_; // True if phong shading should be applied.

  ShaderProgram shader_program_; // Controls which fragment shader to use.

  Ogre::HighLevelGpuProgramPtr vtx_shader_; // Main vertex shader.
  Ogre::HighLevelGpuProgramPtr texture_shader_; // Image texture fragment shader.
  Ogre::HighLevelGpuProgramPtr idepth_shader_; // IDepth fragment shader.
  Ogre::HighLevelGpuProgramPtr jet_shader_; // Jet fragment shader.
  Ogre::HighLevelGpuProgramPtr normal_shader_; // Normal vector fragment shader.
};

}  // namespace flame_rviz_plugins
