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
 * @file asl_rgbd_offline_stream.cc
 * @author W. Nicholas Greene
 * @date 2017-07-29 19:27:10 (Sat)
 */

#include "ros_sensor_streams/asl_rgbd_offline_stream.h"

#include <stdio.h>

#include <algorithm>
#include <unordered_set>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>

namespace bfs = boost::filesystem;

namespace du = dataset_utils;
namespace dua = dataset_utils::asl;

namespace ros_sensor_streams {

ASLRGBDOfflineStream::ASLRGBDOfflineStream(ros::NodeHandle& nh,
                                           const std::string& pose_path,
                                           const std::string& rgb_path,
                                           const std::string& depth_path,
                                           const std::string& camera_name,
                                           const std::string& camera_world_frame_id,
                                           const std::string& camera_frame_id,
                                           WorldFrame world_frame,
                                           bool publish) :
    nh_(nh),
    inited_(false),
    camera_name_(camera_name),
    camera_world_frame_id_(camera_world_frame_id),
    camera_frame_id_(camera_frame_id),
    world_frame_(world_frame),
    publish_(publish),
    curr_idx_(0),
    pose_data_(pose_path),
    rgb_data_(rgb_path),
    depth_data_(),
    pose_idxs_(),
    rgb_idxs_(),
    depth_idxs_(),
    q_pose_in_body_(),
    t_pose_in_body_(),
    q_cam_in_body_(),
    t_cam_in_body_(),
    width_(),
    height_(),
    K_(Eigen::Matrix3f::Identity()),
    cinfo_(),
    intensity_to_depth_factor_(),
    tf_pub_(),
    it_(nh),
    rgb_pub_(),
    depth_pub_() {
  bfs::path depth_path_fs(depth_path);
  if (bfs::exists(depth_path_fs)) {
    // Read in depth data if it exists.
    depth_data_ = std::move(dataset_utils::asl::
                            Dataset<dataset_utils::asl::FileData>(depth_path));
  }

  // Set calibration information.
  width_ = rgb_data_.metadata()["resolution"][0].as<uint32_t>();
  height_ = rgb_data_.metadata()["resolution"][1].as<uint32_t>();
  cinfo_.width = width_;
  cinfo_.height = height_;
  cinfo_.distortion_model = "plumb_bob";

  float fu = rgb_data_.metadata()["intrinsics"][0].as<float>();
  float fv = rgb_data_.metadata()["intrinsics"][1].as<float>();
  float cu = rgb_data_.metadata()["intrinsics"][2].as<float>();
  float cv = rgb_data_.metadata()["intrinsics"][3].as<float>();

  cinfo_.K = {fu, 0, cu,
              0, fv, cv,
              0, 0, 1};

  K_(0, 0) = fu;
  K_(0, 2) = cu;
  K_(1, 1) = fv;
  K_(1, 2) = cv;

  cinfo_.P = {fu, 0, cu, 0,
              0, fv, cv, 0,
              0, 0, 1, 0};

  float k1 = rgb_data_.metadata()["distortion_coefficients"][0].as<float>();
  float k2 = rgb_data_.metadata()["distortion_coefficients"][1].as<float>();
  float p1 = rgb_data_.metadata()["distortion_coefficients"][2].as<float>();
  float p2 = rgb_data_.metadata()["distortion_coefficients"][3].as<float>();
  float k3 = 0.0f;

  cinfo_.D = {k1, k2, p1, p2, k3};

  if (!depth_data_.path().empty()) {
    intensity_to_depth_factor_ = depth_data_.metadata()["depth_scale_factor"].as<float>();
  }

  if (publish_) {
    rgb_pub_ = it_.advertiseCamera("/" + camera_name_ + "/rgb/image_rect_color", 5);

    if (!depth_data_.path().empty()) {
      depth_pub_ = it_.advertiseCamera("/" + camera_name_ + "/depth_registered/image_rect", 5);
    }
  }

  associateData();

  // Extract transform of pose sensor in body frame.
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_pose_in_body;
  du::readMatrix(pose_data_.metadata(), "T_BS", 4, 4, T_pose_in_body.data());
  q_pose_in_body_ = Eigen::Quaterniond(T_pose_in_body.block<3, 3>(0, 0));
  t_pose_in_body_ = T_pose_in_body.block<3, 1>(0, 3);

  // Extract transform of camera in body frame.
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_cam_in_body;
  du::readMatrix(rgb_data_.metadata(), "T_BS", 4, 4, T_cam_in_body.data());
  q_cam_in_body_ = Eigen::Quaterniond(T_cam_in_body.block<3, 3>(0, 0));
  t_cam_in_body_ = T_cam_in_body.block<3, 1>(0, 3);

  return;
}

void ASLRGBDOfflineStream::associateData() {
  auto diff = [](const dua::FileData& x, const dua::PoseData& y) {
    double tx = static_cast<double>(x.timestamp) * 1e-9;
    double ty = static_cast<double>(y.timestamp) * 1e-9;
    return std::fabs(tx - ty);
  };

  // Match rgb and depth to pose separately.
  std::vector<std::size_t> pose_rgb_idxs;
  std::vector<std::size_t> rgb_pose_idxs;
  du::associate(rgb_data_.data(), pose_data_.data(), &rgb_pose_idxs,
                &pose_rgb_idxs, diff);

  std::vector<std::size_t> pose_depth_idxs;
  std::vector<std::size_t> depth_pose_idxs;
  if (!depth_data_.path().empty()) {
    du::associate(depth_data_.data(), pose_data_.data(), &depth_pose_idxs,
                  &pose_depth_idxs, diff);
  } else {
    // No depth data. Just copy rgb data.
    pose_depth_idxs = pose_rgb_idxs;
    depth_pose_idxs = rgb_pose_idxs;
  }

  // Now take the intersection of pose_rgb_idxs and pose_depth_idxs to get the
  // indices that match both.
  pose_idxs_.clear();
  std::set_intersection(pose_rgb_idxs.begin(), pose_rgb_idxs.end(),
                        pose_depth_idxs.begin(), pose_depth_idxs.end(),
                        std::back_inserter(pose_idxs_));

  // Now get corresponding rgb and depth idxs.
  std::unordered_set<std::size_t> pose_idxs_set(pose_idxs_.begin(), pose_idxs_.end());

  rgb_idxs_.clear();
  for (int ii = 0; ii < pose_rgb_idxs.size(); ++ii) {
    if (pose_idxs_set.count(pose_rgb_idxs[ii]) > 0) {
      rgb_idxs_.push_back(rgb_pose_idxs[ii]);
    }
  }

  depth_idxs_.clear();
  if (!depth_data_.path().empty()) {
    for (int ii = 0; ii < pose_depth_idxs.size(); ++ii) {
      if (pose_idxs_set.count(pose_depth_idxs[ii]) > 0) {
        depth_idxs_.push_back(depth_pose_idxs[ii]);
      }
    }
  }

  return;
}

void ASLRGBDOfflineStream::get(uint32_t* id, double* time,
                               cv::Mat3b* rgb, cv::Mat1f* depth,
                               Eigen::Quaterniond* quat,
                               Eigen::Vector3d* trans) {
  // Make sure we actually have data to read in.
  if (empty()) {
    ROS_ERROR("No more data!\n");
    return;
  }

  *id = curr_idx_;
  *time = static_cast<double>(rgb_data_[rgb_idxs_[curr_idx_]].timestamp) * 1e-9;

  // Load raw pose, which is the pose of the pose sensor wrt a given world
  // frame.
  Eigen::Quaterniond q_pose_in_world(pose_data_[pose_idxs_[curr_idx_]].quat);
  Eigen::Vector3d t_pose_in_world(pose_data_[pose_idxs_[curr_idx_]].trans);
  q_pose_in_world.normalize();

  // Get pose of camera wrt to world frame.
  Eigen::Quaterniond q_body_in_pose(q_pose_in_body_.inverse());
  Eigen::Vector3d t_body_in_pose(-(q_pose_in_body_.inverse() * t_pose_in_body_));

  Eigen::Quaterniond q_body_in_world(q_pose_in_world * q_body_in_pose);
  Eigen::Vector3d t_body_in_world(q_pose_in_world * t_body_in_pose + t_pose_in_world);

  Eigen::Quaterniond q_cam_in_world = q_body_in_world * q_cam_in_body_;
  Eigen::Vector3d t_cam_in_world = q_body_in_world * t_cam_in_body_ + t_body_in_world;

  // Convert poses to optical coordinates.
  switch (world_frame_) {
    case RDF: {
      *quat = q_cam_in_world;
      *trans = t_cam_in_world;
      break;
    }
    case FLU: {
      // Local RDF frame in global FLU frame.
      Eigen::Quaterniond q_flu_to_rdf(-0.5, -0.5, 0.5, -0.5);
      *quat = q_flu_to_rdf * q_cam_in_world;
      *trans = q_flu_to_rdf * t_cam_in_world;
      break;
    }
    case FRD: {
      // Local RDF frame in global FRD frame.
      Eigen::Matrix3d R_frd_to_rdf;
      R_frd_to_rdf << 0.0, 1.0, 0.0,
          0.0, 0.0, 1.0,
          1.0, 0.0, 0.0;

      Eigen::Quaterniond q_frd_to_rdf(R_frd_to_rdf);
      *quat = q_frd_to_rdf * q_cam_in_world;
      *trans = q_frd_to_rdf * t_cam_in_world;
      break;
    }
    case RFU: {
      // Local RDF frame in global RFU frame.
      Eigen::Matrix3d R_rfu_to_rdf;
      R_rfu_to_rdf << 1.0, 0.0, 0.0,
          0.0, 0.0, -1.0,
          0.0, 1.0, 0.0;

      Eigen::Quaterniond q_rfu_to_rdf(R_rfu_to_rdf);
      *quat = q_rfu_to_rdf * q_cam_in_world;
      *trans = q_rfu_to_rdf * t_cam_in_world;
      break;
    }
    default:
      ROS_ERROR("Unknown input frame specified!\n");
      return;
  }

  // Load RGB.
  bfs::path rgb_path = bfs::path(rgb_data_.path()) / bfs::path("data") /
      bfs::path(rgb_data_[rgb_idxs_[curr_idx_]].filename);
  cv::Mat3b rgb_raw = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);

  // Undistort image.
  cv::Mat Kcv;
  eigen2cv(K_, Kcv);
  cv::undistort(rgb_raw, *rgb, Kcv, cinfo_.D);

  if (!depth_data_.path().empty()) {
    // Have depth data.
    bfs::path depth_path = bfs::path(depth_data_.path()) / bfs::path("data") /
        bfs::path(depth_data_[depth_idxs_[curr_idx_]].filename);
    cv::Mat_<uint16_t> depth_raw =
        cv::imread(depth_path.string(), cv::IMREAD_ANYDEPTH);

    // Assume depthmaps are undistorted!
    cv::Mat depth_undistorted;
    depth_undistorted = depth_raw.clone();

    // Scale depth information.
    depth->create(height_, width_);
    float* depth_ptr = reinterpret_cast<float*>(depth->data);
    uint16_t* depth_raw_ptr = reinterpret_cast<uint16_t*>(depth_undistorted.data);
    for (uint32_t ii = 0; ii < height_ * width_; ++ii) {
      depth_ptr[ii] = static_cast<float>(depth_raw_ptr[ii]) / intensity_to_depth_factor_;
    }
  }

  if (publish_) {
    // Publish pose over tf.
    geometry_msgs::TransformStamped tf;
    tf.header.stamp.fromSec(*time);
    tf.header.frame_id = camera_world_frame_id_;
    tf.child_frame_id = camera_frame_id_;
    tf.transform.rotation.w = quat->w();
    tf.transform.rotation.x = quat->x();
    tf.transform.rotation.y = quat->y();
    tf.transform.rotation.z = quat->z();

    tf.transform.translation.x = (*trans)(0);
    tf.transform.translation.y = (*trans)(1);
    tf.transform.translation.z = (*trans)(2);
    tf_pub_.sendTransform(tf);

    // Publish messages over ROS.
    std_msgs::Header header;
    header.stamp.fromSec(*time);
    header.frame_id = camera_frame_id_;

    sensor_msgs::CameraInfo::Ptr cinfo_msg(new sensor_msgs::CameraInfo);
    *cinfo_msg = cinfo_;
    cinfo_msg->header = header;

    cv_bridge::CvImage rgb_cvi(header, "bgr8", *rgb);
    rgb_pub_.publish(rgb_cvi.toImageMsg(), cinfo_msg);

    if (!depth_data_.path().empty()) {
      cv_bridge::CvImage depth_cvi(header, "32FC1", *depth);
      depth_pub_.publish(depth_cvi.toImageMsg(), cinfo_msg);
    }
  }

  // Increment counter.
  curr_idx_++;

  return;
}

}  // namespace ros_sensor_streams
