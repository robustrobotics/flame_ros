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
 * @file utils.cc
 * @author W. Nicholas Greene
 * @date 2016-12-13 15:28:50 (Tue)
 */

#include "./utils.h" // NOLINT

#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>

#include <flame/utils/image_utils.h>
#include <flame/utils/visualization.h>

#include <flame_ros/FlameNodeletStats.h>
#include <flame_ros/FlameStats.h>

namespace fu = flame::utils;

namespace flame_ros {

void publishFlameNodeletStats(const ros::Publisher& pub,
                              int img_id, double time,
                              const std::unordered_map<std::string, double>& stats,
                              const std::unordered_map<std::string, double>& timings) {
  FlameNodeletStats::Ptr msg(new FlameNodeletStats());
  msg->header.stamp.fromSec(time);

  msg->img_id = img_id;
  msg->timestamp = time;

  // Fill stat if it exists in the map.
  auto fillStati = [](const std::unordered_map<std::string, double>& stats,
                      const std::string& name, int* out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };
  auto fillStatf = [](const std::unordered_map<std::string, double>& stats,
                      const std::string& name, float* out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };

  fillStati(stats, "queue_size", &msg->queue_size);
  fillStatf(stats, "fps", &msg->fps);
  fillStatf(stats, "fps_max", &msg->fps_max);
  fillStatf(timings, "main", &msg->main_ms);
  fillStatf(timings, "waiting", &msg->waiting_ms);
  fillStatf(timings, "process_frame", &msg->process_frame_ms);
  fillStatf(timings, "publishing", &msg->publishing_ms);
  fillStatf(timings, "debug_publishing", &msg->debug_publishing_ms);
  fillStatf(stats, "latency", &msg->latency_ms);

  fillStatf(stats, "max_load_cpu", &msg->max_load_cpu);
  fillStatf(stats, "max_load_mem", &msg->max_load_mem);
  fillStatf(stats, "max_load_swap", &msg->max_load_swap);
  fillStatf(stats, "sys_load_cpu", &msg->sys_load_cpu);
  fillStatf(stats, "sys_load_mem", &msg->sys_load_mem);
  fillStatf(stats, "sys_load_swap", &msg->sys_load_swap);
  fillStatf(stats, "pid_load_cpu", &msg->pid_load_cpu);
  fillStatf(stats, "pid_load_mem", &msg->pid_load_mem);
  fillStatf(stats, "pid_load_swap ", &msg->pid_load_swap );
  fillStati(stats, "pid", &msg->pid);

  pub.publish(msg);

  return;
}

void publishFlameStats(const ros::Publisher& pub,
                       int img_id, double time,
                       const std::unordered_map<std::string, double>& stats,
                       const std::unordered_map<std::string, double>& timings) {
  FlameStats::Ptr msg(new FlameStats());
  msg->header.stamp.fromSec(time);

  msg->img_id = img_id;
  msg->timestamp = time;

  // Fill stat if it exists in the map.
  auto fillStati = [](const std::unordered_map<std::string, double>& stats,
                      const std::string& name, int* out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };
  auto fillStatf = [](const std::unordered_map<std::string, double>& stats,
                      const std::string& name, float* out) {
    if (stats.count(name) > 0) {
      *out = stats.at(name);
    }
    return;
  };

  fillStati(stats, "num_feats", &msg->num_feats);
  fillStati(stats, "num_vtx", &msg->num_vtx);
  fillStati(stats, "num_tris", &msg->num_tris);
  fillStati(stats, "num_edges", &msg->num_edges);

  fillStatf(stats, "coverage", &msg->coverage);

  fillStati(stats, "num_idepth_updates", &msg->num_idepth_updates);
  fillStati(stats, "num_fail_max_var", &msg->num_fail_max_var);
  fillStati(stats, "num_fail_max_dropouts", &msg->num_fail_max_dropouts);
  fillStati(stats, "num_fail_ref_patch_grad", &msg->num_fail_ref_patch_grad);
  fillStati(stats, "num_fail_ambiguous_match", &msg->num_fail_ambiguous_match);
  fillStati(stats, "num_fail_max_cost", &msg->num_fail_max_cost);

  fillStatf(stats, "nltgv2_total_smoothness_cost",
            &msg->nltgv2_total_smoothness_cost);
  fillStatf(stats, "nltgv2_avg_smoothness_cost",
            &msg->nltgv2_avg_smoothness_cost);
  fillStatf(stats, "nltgv2_total_data_cost", &msg->nltgv2_total_data_cost);
  fillStatf(stats, "nltgv2_avg_data_cost", &msg->nltgv2_avg_data_cost);

  fillStatf(stats, "total_photo_error", &msg->total_photo_error);
  fillStatf(stats, "avg_photo_error", &msg->avg_photo_error);

  fillStatf(stats, "fps", &msg->fps);
  fillStatf(stats, "fps_max", &msg->fps_max);
  fillStatf(timings, "update", &msg->update_ms);
  fillStatf(timings, "update_locking", &msg->update_locking_ms);
  fillStatf(timings, "frame_creation", &msg->frame_creation_ms);
  fillStatf(timings, "interpolate", &msg->interpolate_ms);
  fillStatf(timings, "keyframe", &msg->keyframe_ms);
  fillStatf(timings, "detection", &msg->detection_ms);
  fillStatf(timings, "detection_loop", &msg->detection_loop_ms);
  fillStatf(timings, "update_idepths", &msg->update_idepths_ms);
  fillStatf(timings, "project_features", &msg->project_features_ms);
  fillStatf(timings, "project_graph", &msg->project_graph_ms);
  fillStatf(timings, "sync_graph", &msg->sync_graph_ms);
  fillStatf(timings, "triangulate", &msg->triangulate_ms);
  fillStatf(timings, "median_filter", &msg->median_filter_ms);
  fillStatf(timings, "lowpass_filter", &msg->lowpass_filter_ms);

  pub.publish(msg);

  return;
}

void publishDepthMesh(const ros::Publisher& mesh_pub,
                      const std::string& frame_id,
                      double time,
                      const Eigen::Matrix3f& Kinv,
                      const std::vector<cv::Point2f>& vertices,
                      const std::vector<float>& idepths,
                      const std::vector<Eigen::Vector3f>& normals,
                      const std::vector<flame::Triangle>& triangles,
                      const std::vector<bool>& tri_validity,
                      const cv::Mat3b& rgb) {
  pcl_msgs::PolygonMesh::Ptr msg(new pcl_msgs::PolygonMesh());
  msg->header.stamp.fromSec(time);
  msg->header.frame_id = frame_id;

  // Create point cloud to hold vertices.
  pcl::PointCloud<flame_ros::PointNormalUV> cloud;
  cloud.width = vertices.size();
  cloud.height = 1;
  cloud.points.resize(vertices.size());
  cloud.is_dense = false;

  for (int ii = 0; ii < vertices.size(); ++ii) {
    float id = idepths[ii];
    if (!std::isnan(id) && (id > 0.0f)) {
      Eigen::Vector3f uhom(vertices[ii].x, vertices[ii].y, 1.0f);
      uhom /= id;
      Eigen::Vector3f p(Kinv * uhom);
      cloud.points[ii].x = p(0);
      cloud.points[ii].y = p(1);
      cloud.points[ii].z = p(2);

      cloud.points[ii].normal_x = normals[ii](0);
      cloud.points[ii].normal_y = normals[ii](1);
      cloud.points[ii].normal_z = normals[ii](2);

      // OpenGL textures range from 0 to 1.
      cloud.points[ii].u = vertices[ii].x / (rgb.cols - 1);
      cloud.points[ii].v = vertices[ii].y / (rgb.rows - 1);
    } else {
      // Add invalid value to skip this point. Note that the initial value
      // is (0, 0, 0), so you must manually invalidate the point.
      cloud.points[ii].x = std::numeric_limits<float>::quiet_NaN();
      cloud.points[ii].y = std::numeric_limits<float>::quiet_NaN();
      cloud.points[ii].z = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }

  pcl::toROSMsg(cloud, msg->cloud);

  // NOTE: Header fields need to be filled in after pcl::toROSMsg() call.
  msg->cloud.header = std_msgs::Header();
  msg->cloud.header.stamp.fromSec(time);
  msg->cloud.header.frame_id = frame_id;

  // Fill in faces.
  msg->polygons.reserve(triangles.size());
  for (int ii = 0; ii < triangles.size(); ++ii) {
    if (tri_validity[ii]) {
      pcl_msgs::Vertices vtx_ii;
      vtx_ii.vertices.resize(3);
      vtx_ii.vertices[0] = triangles[ii][2];
      vtx_ii.vertices[1] = triangles[ii][1];
      vtx_ii.vertices[2] = triangles[ii][0];

      msg->polygons.push_back(vtx_ii);
    }
  }

  if (msg->polygons.size() > 0) {
    mesh_pub.publish(msg);
  }

  return;
}

void publishDepthMap(const image_transport::CameraPublisher& pub,
                     const std::string& frame_id,
                     double time, const Eigen::Matrix3f& K,
                     const cv::Mat1f& depth_est) {
  // Publish depthmap.
  std_msgs::Header header;
  header.stamp.fromSec(time);
  header.frame_id = frame_id;

  sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo);
  cinfo->header = header;
  cinfo->height = depth_est.rows;
  cinfo->width = depth_est.cols;
  cinfo->distortion_model = "plumb_bob";
  cinfo->D = {0.0, 0.0, 0.0, 0.0, 0.0};
  for (int ii = 0; ii < 3; ++ii) {
    for (int jj = 0; jj < 3; ++jj) {
      cinfo->K[ii*3 + jj] = K(ii, jj);
      cinfo->P[ii*4 + jj] = K(ii, jj);
      cinfo->R[ii*3 + jj] = 0.0;
    }
  }
  cinfo->P[3] = 0.0;
  cinfo->P[7] = 0.0;
  cinfo->P[11] = 0.0;
  cinfo->R[0] = 1.0;
  cinfo->R[4] = 1.0;
  cinfo->R[8] = 1.0;

  cv_bridge::CvImage depth_cvi(header, "32FC1", depth_est);

  pub.publish(depth_cvi.toImageMsg(), cinfo);

  return;
}

void publishPointCloud(const ros::Publisher& pub,
                       const std::string& frame_id,
                       double time, const Eigen::Matrix3f& K,
                       const cv::Mat1f& depth_est,
                       float min_depth, float max_depth) {
  int height = depth_est.rows;
  int width = depth_est.cols;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = width;
  cloud.height = height;
  cloud.is_dense = false;
  cloud.points.resize(width * height);

  Eigen::Matrix3f Kinv(K.inverse());
  for (int ii = 0; ii < height; ++ii) {
    for (int jj = 0; jj < width; ++jj) {
      int idx = ii*width + jj;

      float depth = depth_est(ii, jj);

      if (std::isnan(depth) || (depth < min_depth) || (depth > max_depth)) {
        // Add invalid value to skip this point. Note that the initial value
        // is (0, 0, 0), so you must manually invalidate the point.
        cloud.points[idx].x = std::numeric_limits<float>::quiet_NaN();
        cloud.points[idx].y = std::numeric_limits<float>::quiet_NaN();
        cloud.points[idx].z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      Eigen::Vector3f xyz(jj * depth, ii * depth, depth);
      xyz = Kinv * xyz;

      cloud.points[idx].x = xyz(0);
      cloud.points[idx].y = xyz(1);
      cloud.points[idx].z = xyz(2);
    }
  }

  sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(cloud, *msg);

  msg->header = std_msgs::Header();
  msg->header.stamp.fromSec(time);
  msg->header.frame_id = frame_id;

  pub.publish(msg);

  return;
}

void getDepthConfusionMatrix(const cv::Mat1f& idepths, const cv::Mat1f& depth,
                             cv::Mat1f* idepth_error,  float* total_error,
                             int* true_pos, int* true_neg,
                             int* false_pos, int* false_neg) {
  // Compute confusion matrix with detection being strictly positive idepth.
  *true_pos = 0;
  *true_neg = 0;
  *false_pos = 0;
  *false_neg = 0;

  *total_error = 0.0f;
  *idepth_error = cv::Mat1f(depth.rows, depth.cols,
                            std::numeric_limits<float>::quiet_NaN());
  for (int ii = 0; ii < depth.rows; ++ii) {
    for (int jj = 0; jj < depth.cols; ++jj) {
      if (depth(ii, jj) > 0) {
        if (!std::isnan(idepths(ii, jj))) {
          float idepth_est = idepths(ii, jj);
          float idepth_true = 1.0f / depth(ii, jj);

          float error = fu::fast_abs(idepth_est - idepth_true);
          (*idepth_error)(ii, jj) = error;
          *total_error += error;

          (*true_pos)++;
        } else {
          (*false_neg)++;
        }
      } else if (!std::isnan(idepths(ii, jj))) {
        float idepth_est = idepths(ii, jj);
        float error = fu::fast_abs(idepth_est);
        (*idepth_error)(ii, jj) = error;
        *total_error += error;

        (*false_pos)++;
      } else {
        (*true_neg)++;
      }
    }
  }

  return;
}

}  // namespace flame_ros
