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
 * @file tum_rgbd_offline_stream.h
 * @author W. Nicholas Greene
 * @date 2017-08-21 21:36:49 (Mon)
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <sensor_msgs/CameraInfo.h>

namespace ros_sensor_streams {

/**
 * \brief Class that represents an input stream of tracked RGBD images.
 *
 * Input is taken from a text file using the TUM RGB-D benchmarks format:
 * http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
 *
 * Each line in the input file should have the following information:
 *
 *   timestamp tx ty tz qx qy qz qw rgb_timestamp rgb/frame.png depth_timestamp depth/frame.png
 *
 * where timestamp is in seconds, t[xyz] is the translation of the camera,
 * q[xyzw] is the orientation of the camera (as a quaternion), rgb/frame.png is
 * the RGB image and depth/frame.png is the depth image. The depth image should
 * be stored in 16-bit monochrome format scaled by a given factor. The input
 * poses can use one of several frame conventions (see the InputFrame enum), but
 * the output poses should always be in optical coordinates
 * (Right-Down-Forward).
 *
 * The input will be prepared as needed so that no data will be dropped. The
 * depth image is optinal.
 */
class TUMRGBDOfflineStream final  {
 public:
  // Enums for the coordinate frame of the input poses. Output poses will always
  // be in Right-Down-Forward.
  enum InputFrame {
    RDF, // Right-Down-Forward (optical coordinates)
    FLU, // Forward-Left-Up (ROS convention)
    FRD, // Forward-Right-Down (navigation frame)
    RDF_IN_FLU, // Pose is the transform from the local RDF frame attached to
                // the camera to the global FLU frame.
    RDF_IN_FRD, // Pose is the transform from the local RDF frame attached to
                // camera to global FRD frame.
  };

  /**
   * @brief Constructor.
   *
   * @param[in] nh NodeHandle.
   * @param[in] input_file Input text file.
   * @param[in] calib_file Calibration file in ROS CameraInfo/YAML format.
   * @param[in] camera_name Name of camera.
   * @param[in] camera_world_frame_id Frame ID of the camera world frame.
   * @param[in] camera_frame_id Frame ID of the camera.
   * @param[in] input_frame Frame of the input poses.
   * @param[in] depth_scale_factor Factor to scale raw depthmaps to meters.
   * @param[in] publish Publish data over ROS.
   */
  TUMRGBDOfflineStream(ros::NodeHandle& nh,
                       const std::string& input_file,
                       const std::string& calib_file,
                       const std::string& camera_name,
                       const std::string& camera_world_frame_id,
                       const std::string& camera_frame_id,
                       InputFrame input_frame,
                       float depth_scale_factor = 5000.0f,
                       bool publish = true);

  ~TUMRGBDOfflineStream() = default;

  TUMRGBDOfflineStream(const TUMRGBDOfflineStream& rhs) = delete;
  TUMRGBDOfflineStream& operator=(const TUMRGBDOfflineStream& rhs) = delete;

  TUMRGBDOfflineStream(const TUMRGBDOfflineStream&& rhs) = delete;
  TUMRGBDOfflineStream& operator=(const TUMRGBDOfflineStream&& rhs) = delete;

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
           Eigen::Quaternionf* quat, Eigen::Vector3f* trans);

  /**
   * @brief Returns true if all data has been published.
   */
  bool empty() const {
    return curr_idx_ >= static_cast<int>(input_.size());
  }

  /**
   * \brief Returns true if stream is initialized.
   */
  bool inited() const {
    return inited_;
  }

  /**
   * \brief Get the image width.
   */
  int width() const {
    return width_;
  }

  /**
   * \brief Get the image height.
   */
  int height() const {
    return height_;
  }

  /**
   * \brief Return the camera intrinsic matrix.
   */
  const Eigen::Matrix3f& K() const {
    return K_;
  }

  /**
   * \brief Return id of the live frame (i.e. the pose of the camera).
   */
  const std::string& camera_frame_id() const {
    return camera_frame_id_;
  }

 private:
  // Extract information from line.
  bool parseLine(const std::string& line, double* time, cv::Mat* rgb,
                 cv::Mat* depth, Eigen::Quaternionf* quat,
                 Eigen::Vector3f* trans);

  ros::NodeHandle& nh_;

  bool inited_;

  std::string input_file_;
  std::string calib_file_;

  std::string camera_name_;
  std::string camera_world_frame_id_;
  std::string camera_frame_id_;
  InputFrame input_frame_;

  int width_;
  int height_;
  Eigen::Matrix3f K_;
  sensor_msgs::CameraInfo cinfo_;
  image_geometry::PinholeCameraModel model_; // Needed to undistort images.

  int curr_idx_;
  std::vector<std::string> input_;
  std::string base_dir_; // Base directory of dataset.

  float depth_scale_factor_;
  bool publish_;

  // Publishers.
  tf2_ros::TransformBroadcaster tf_pub_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher rgb_pub_;
  image_transport::CameraPublisher depth_pub_;
};

}  // namespace ros_sensor_streams
