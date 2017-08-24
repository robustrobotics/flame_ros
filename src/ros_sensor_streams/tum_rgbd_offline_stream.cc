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
 * @file tum_rgbd_offline_stream.cc
 * @author W. Nicholas Greene
 * @date 2017-08-21 21:36:54 (Mon)
 */

#include "ros_sensor_streams/tum_rgbd_offline_stream.h"

#include <stdio.h>

#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <camera_info_manager/camera_info_manager.h>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>

namespace fs = boost::filesystem;

namespace ros_sensor_streams {

TUMRGBDOfflineStream::TUMRGBDOfflineStream(ros::NodeHandle& nh,
                                           const std::string& input_file,
                                           const std::string& calib_file,
                                           const std::string& camera_name,
                                           const std::string& camera_world_frame_id,
                                           const std::string& camera_frame_id,
                                           InputFrame input_frame,
                                           float depth_scale_factor,
                                           bool publish) :
    nh_(nh),
    inited_(false),
    input_file_(input_file),
    calib_file_(calib_file),
    camera_name_(camera_name),
    camera_world_frame_id_(camera_world_frame_id),
    camera_frame_id_(camera_frame_id),
    input_frame_(input_frame),
    width_(),
    height_(),
    K_(),
    cinfo_(),
    model_(),
    curr_idx_(0),
    input_(),
    base_dir_(),
    depth_scale_factor_(depth_scale_factor),
    publish_(publish),
    tf_pub_(),
    it_(nh),
    rgb_pub_(),
    depth_pub_() {
  // Make sure files exist.
  if (!fs::exists(input_file)) {
    ROS_ERROR("Input file does not exist: %s\n", input_file_.c_str());
    return;
  }
  if (!fs::exists(calib_file)) {
    ROS_ERROR("Calibration file does not exist: %s\n", calib_file_.c_str());
    return;
  }

  // Load calibration information.
  camera_info_manager::CameraInfoManager manager(nh_);
  manager.loadCameraInfo("file://" + calib_file);
  cinfo_ = manager.getCameraInfo();
  model_.fromCameraInfo(cinfo_);

  // Set the camera intrinsics matrix. Note that we use elements of the P matrix
  // instead of the K matrix in the CameraInfo message. The upper left corner of
  // the P matrix represents the parameters for the undistored images (what we
  // will pass outside).
  width_ = cinfo_.width;
  height_ = cinfo_.height;

  for (int ii = 0; ii < 3; ++ii) {
    for (int jj = 0; jj < 3; ++jj) {
      K_(ii, jj) = cinfo_.P[ii*4 + jj];
    }
  }

  // Read in input data.
  std::ifstream input_stream(input_file_);
  std::string line;
  while (std::getline(input_stream, line)) {
    input_.push_back(line);
  }

  // Get parent directory.
  fs::path p(input_file_);
  base_dir_ = p.parent_path().string();

  if (publish_) {
    rgb_pub_ = it_.advertiseCamera("/" + camera_name_ + "/rgb/image_rect_color", 5);
    depth_pub_ = it_.advertiseCamera("/" + camera_name_ + "/depth_registered/image_rect", 5);
  }

  return;
}

void TUMRGBDOfflineStream::get(uint32_t* id, double* time,
                               cv::Mat3b* rgb, cv::Mat1f* depth,
                               Eigen::Quaternionf* quat,
                               Eigen::Vector3f* trans) {
  // Make sure we actually have data to read in.
  if (empty()) {
    ROS_ERROR("No more data!\n");
    return;
  }

  *id = curr_idx_;

  Eigen::Quaternionf input_quat;
  Eigen::Vector3f input_trans;
  cv::Mat rgb_raw, depth_raw;
  if (!parseLine(input_[curr_idx_], time, &rgb_raw, &depth_raw, &input_quat,
                 &input_trans)) {
    ROS_ERROR("Could not parse input!\n");
  }
  input_quat.normalize();

  // Convert poses to optical coordinates.
  switch (input_frame_) {
    case RDF: {
      // Right-Down-Forward
      *quat = input_quat;
      *trans = input_trans;
      break;
    }
    case FLU: {
      // Forward-Left-Up
      Eigen::Quaternionf q_flu_to_rdf(-0.5, -0.5, 0.5, -0.5);
      *quat = q_flu_to_rdf * input_quat * q_flu_to_rdf.inverse();
      *trans = q_flu_to_rdf * input_trans;
      break;
    }
    case FRD: {
      // Forward-Right-Down
      Eigen::Matrix3f R_frd_to_rdf;
      R_frd_to_rdf << 0.0f, 1.0f, 0.0f,
          0.0f, 0.0f, 1.0f,
          1.0f, 0.0f, 0.0f;

      Eigen::Quaternionf q_frd_to_rdf(R_frd_to_rdf);
      *quat = q_frd_to_rdf * input_quat * q_frd_to_rdf.inverse();
      *trans = q_frd_to_rdf * input_trans;
      break;
    }
    case RDF_IN_FLU: {
      // Local RDF frame in global FLU frame.
      Eigen::Quaternionf q_flu_to_rdf(-0.5, -0.5, 0.5, -0.5);
      *quat = q_flu_to_rdf * input_quat;
      *trans = q_flu_to_rdf * input_trans;
      break;
    }
    case RDF_IN_FRD: {
      // Local RDF frame in global FRD frame.
      Eigen::Matrix3f R_frd_to_rdf;
      R_frd_to_rdf << 0.0f, 1.0f, 0.0f,
          0.0f, 0.0f, 1.0f,
          1.0f, 0.0f, 0.0f;

      Eigen::Quaternionf q_frd_to_rdf(R_frd_to_rdf);
      *quat = q_frd_to_rdf * input_quat;
      *trans = q_frd_to_rdf * input_trans;
      break;
    }
    default:
      ROS_ERROR("Unknown input frame specified!\n");
      return;
  }

  // Undistort images.
  model_.rectifyImage(rgb_raw, *rgb);

  cv::Mat depth_undistorted;
  model_.rectifyImage(depth_raw, depth_undistorted);

  // Scale depth information.
  depth->create(height_, width_);
  float* depth_ptr = reinterpret_cast<float*>(depth->data);
  uint16_t* depth_raw_ptr = reinterpret_cast<uint16_t*>(depth_undistorted.data);
  for (uint32_t ii = 0; ii < height_ * width_; ++ii) {
    depth_ptr[ii] = static_cast<float>(depth_raw_ptr[ii]) / depth_scale_factor_;
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
    cv_bridge::CvImage depth_cvi(header, "32FC1", *depth);

    rgb_pub_.publish(rgb_cvi.toImageMsg(), cinfo_msg);
    depth_pub_.publish(depth_cvi.toImageMsg(), cinfo_msg);
  }

  // Increment counter.
  curr_idx_++;

  return;
}

bool TUMRGBDOfflineStream::parseLine(const std::string& line, double* time, cv::Mat* rgb,
                                     cv::Mat* depth, Eigen::Quaternionf* quat,
                                     Eigen::Vector3f* trans) {
  // Parse line.
  double pose_time;
  float tx, ty, tz, qx, qy, qz, qw;
  double rgb_time;
  char rgb_cstr[256];
  double depth_time;
  char depth_cstr[256];

  int num_tokens = 0;
  std::istringstream ss(line);
  if (ss >> pose_time) num_tokens++;
  if (ss >> tx) num_tokens++;
  if (ss >> ty) num_tokens++;
  if (ss >> tz) num_tokens++;
  if (ss >> qx) num_tokens++;
  if (ss >> qy) num_tokens++;
  if (ss >> qz) num_tokens++;
  if (ss >> qw) num_tokens++;
  if (ss >> rgb_time) num_tokens++;
  if (ss >> rgb_cstr) num_tokens++;
  if (ss >> depth_time) num_tokens++;
  if (ss >> depth_cstr) num_tokens++;

  // Use rgb time as associated time.
  *time = rgb_time;

  // Set pose.
  quat->x() = qx;
  quat->y() = qy;
  quat->z() = qz;
  quat->w() = qw;
  (*trans)(0) = tx;
  (*trans)(1) = ty;
  (*trans)(2) = tz;

  // Read in rgb image.
  *rgb = cv::imread(base_dir_ + "/" + std::string(rgb_cstr), cv::IMREAD_COLOR);

  if (num_tokens < 11) {
    // No depth information provided. Create dummy depthmap.
    depth->create(rgb->rows, rgb->cols, cv::DataType<uint16_t>::type);
    *depth = cv::Scalar(0);
  } else {
    // Read in raw depth image.
    *depth = cv::imread(base_dir_ + "/" + std::string(depth_cstr),
                        cv::IMREAD_ANYDEPTH);
  }

  return true;
}

}  // namespace ros_sensor_streams
