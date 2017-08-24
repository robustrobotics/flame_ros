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
 * @file asl_rgbd_offline_stream.h
 * @author W. Nicholas Greene
 * @date 2017-07-29 19:27:14 (Sat)
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <sensor_msgs/CameraInfo.h>

#include <dataset_utils/asl/dataset.h>
#include <dataset_utils/asl/types.h>

namespace ros_sensor_streams {

/**
 * @brief Class that represents an input stream of tracked, undistorted RGBD images.
 *
 * Input is taken from data in ASL format:
 * http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
 *
 * Three ASL folders are needed:
 *   1. Pose data relative to a given world frame.
 *   2. RGB data
 *   3. Depthmap data (optional)
 *
 * The world frame of the input poses can use one of several frame conventions
 * (see the WorldFrame enum), but the output poses should always be relative to
 * a Right-Down-Forward world frame (optical coordinates).
 *
 * We assume the RGB and depth images are registered (i.e. they are relative to
 * the same sensor, as if they were taken with the same camera).
 */
class ASLRGBDOfflineStream final  {
 public:
  // Enums for the coordinate frame of the input poses. Output poses will always
  // be in Right-Down-Forward.
  enum WorldFrame {
    RDF, // Pose is the transform from the local RDF frame attached to the
         // camera to the global RDF frame.
    FLU, // Pose is the transform from the local RDF frame attached to the
         // camera to the global FLU frame.
    FRD, // Pose is the transform from the local RDF frame attached to camera to
         // global FRD frame.
    RFU, // Pose is the transform from the local RDF frame attached to the
         // camera to the global RFU frame.
  };

  /**
   * @brief Constructor.
   *
   * @param[in] nh NodeHandle.
   * @param[in] pose_path Path of ASL pose data.
   * @param[in] rgb_path Path of ASL image data.
   * @param[in] depth_path Path of ASL depthmap data.
   * @param[in] camera_name Name of camera.
   * @param[in] camera_world_frame_id Frame ID of the camera world frame.
   * @param[in] camera_frame_id Frame ID of the camera.
   * @param[in] world_frame Frame of the input poses.
   * @param[in] publish Publish data over ROS.
   */
  ASLRGBDOfflineStream(ros::NodeHandle& nh,
                       const std::string& pose_path,
                       const std::string& rgb_path,
                       const std::string& depth_path,
                       const std::string& camera_name,
                       const std::string& camera_world_frame_id,
                       const std::string& camera_frame_id,
                       WorldFrame world_frame,
                       bool publish = true);

  ~ASLRGBDOfflineStream() = default;

  ASLRGBDOfflineStream(const ASLRGBDOfflineStream& rhs) = delete;
  ASLRGBDOfflineStream& operator=(const ASLRGBDOfflineStream& rhs) = delete;

  ASLRGBDOfflineStream(const ASLRGBDOfflineStream&& rhs) = delete;
  ASLRGBDOfflineStream& operator=(const ASLRGBDOfflineStream&& rhs) = delete;

  /**
   * @brief Get next RGB-D image with pose.
   *
   * @parma[out] id Image ID.
   * @param[out] time Timestamp [sec]
   * @param[out] rgb RGB image.
   * @param[out] depth Depthmap as float image.
   * @param[out] quat Camera orientation.
   * @param[out] trans Camera translation.
   */
  void get(uint32_t* id, double* time, cv::Mat3b* rgb, cv::Mat1f* depth,
           Eigen::Quaterniond* quat, Eigen::Vector3d* trans);

  // Accessors.
  bool empty() const { return curr_idx_ >= static_cast<int>(pose_idxs_.size()); }
  bool inited() const { return inited_; }

  int width() const { return width_; }
  int height() const { return height_; }

  const Eigen::Matrix3f& K() const { return K_; }

  const std::string& camera_frame_id() const { return camera_frame_id_; }

 private:
  // Associate datasets so that each timestamp has one piece of data.
  void associateData();

  ros::NodeHandle& nh_;

  bool inited_;

  std::string camera_name_;
  std::string camera_world_frame_id_;
  std::string camera_frame_id_;
  WorldFrame world_frame_;
  bool publish_;

  int curr_idx_;

  // Raw ASl datasets.
  dataset_utils::asl::Dataset<dataset_utils::asl::PoseData> pose_data_;
  dataset_utils::asl::Dataset<dataset_utils::asl::FileData> rgb_data_;
  dataset_utils::asl::Dataset<dataset_utils::asl::FileData> depth_data_;

  // Synchronized indexes into each dataset.
  std::vector<std::size_t> pose_idxs_;
  std::vector<std::size_t> rgb_idxs_;
  std::vector<std::size_t> depth_idxs_;

  // Sensor extrinsics relative to body frame.
  Eigen::Quaterniond q_pose_in_body_; // Transform of the pose sensor in body frame.
  Eigen::Vector3d t_pose_in_body_;

  Eigen::Quaterniond q_cam_in_body_; // Assume both rgb/depth images have same extrinsics.
  Eigen::Vector3d t_cam_in_body_;

  int width_;
  int height_;
  Eigen::Matrix3f K_;
  sensor_msgs::CameraInfo cinfo_;
  float intensity_to_depth_factor_;

  // Publishers.
  tf2_ros::TransformBroadcaster tf_pub_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher rgb_pub_;
  image_transport::CameraPublisher depth_pub_;
};

}  // namespace ros_sensor_streams
