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
 * @file utils.h
 * @author W. Nicholas Greene
 * @date 2016-12-13 15:28:02 (Tue)
 */

#include <string>
#include <vector>
#include <unordered_map>
#include <limits>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include <ros/ros.h>

#include <image_transport/camera_publisher.h>

#include <flame/utils/triangulator.h>

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

namespace flame_ros {

/**
 * @breif Struct to hold mesh vertex data.
 */
struct PointNormalUV {
  PCL_ADD_POINT4D
  PCL_ADD_NORMAL4D
  float u; // Texture coordinates.
  float v;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

/**
 * @brief Find a parameter or fail.
 *
 * Copied from fsw/fla_utils/param_utils.h.
 */
template <typename T>
void getParamOrFail(const ros::NodeHandle& nh, const std::string& name, T* val) {
  if (!nh.getParam(name, *val)) {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(name, true).c_str());
    exit(1);
  }
  return;
}

/**
 * @brief Publish stats message for FlameNodelet.
 */
void publishFlameNodeletStats(const ros::Publisher& pub,
                              int img_id, double time,
                              const std::unordered_map<std::string, double>& stats,
                              const std::unordered_map<std::string, double>& timings);

/**
 * @brief Publish stats message for Flame.
 */
void publishFlameStats(const ros::Publisher& pub,
                       int img_id, double time,
                       const std::unordered_map<std::string, double>& stats,
                       const std::unordered_map<std::string, double>& timings);

/**
 * @brief Publish mesh.
 */
void publishDepthMesh(const ros::Publisher& mesh_pub,
                      const std::string& frame_id,
                      double time,
                      const Eigen::Matrix3f& Kinv,
                      const std::vector<cv::Point2f>& vertices,
                      const std::vector<float>& idepths,
                      const std::vector<Eigen::Vector3f>& normals,
                      const std::vector<flame::Triangle>& triangles,
                      const std::vector<bool>& tri_validity,
                      const cv::Mat3b& rgb);

/**
 * @brief Publish depthmap.
 */
void publishDepthMap(const image_transport::CameraPublisher& pub,
                     const std::string& frame_id,
                     double time, const Eigen::Matrix3f& K,
                     const cv::Mat1f& depth_est);

/**
 * @brief Publish point cloud.
 */
void publishPointCloud(const ros::Publisher& pub,
                       const std::string& frame_id,
                       double time, const Eigen::Matrix3f& K,
                       const cv::Mat1f& depth_est,
                       float min_depth = 0.0f,
                       float max_depth = std::numeric_limits<float>::max());

/**
 * @brief Compute confusion matrix using ground truth depths.
 */
void getDepthConfusionMatrix(const cv::Mat1f& idepths, const cv::Mat1f& depth,
                             cv::Mat1f* idepth_error, float* total_error,
                             int* true_pos, int* true_neg,
                             int* false_pos, int* false_neg);

}  // namespace flame_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(flame_ros::PointNormalUV,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, x, z)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                  (float, u, u)
                                  (float, v, v))
