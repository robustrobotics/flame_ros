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
 * @file tracked_image_stream.cc
 * @author W. Nicholas Greene
 * @date 2016-12-14 11:25:22 (Wed)
 */

#include "ros_sensor_streams/tracked_image_stream.h"

#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sophus/se3.hpp>

#include <cv_bridge/cv_bridge.h>

#include "ros_sensor_streams/conversions.h"

namespace ros_sensor_streams {

TrackedImageStream::TrackedImageStream(const std::string& world_frame_id,
                                       ros::NodeHandle& nh,
                                       const Eigen::Matrix3f& K,
                                       const Eigen::VectorXf& D,
                                       bool undistort,
                                       int resize_factor,
                                       int queue_size) :
    nh_(nh),
    inited_(false),
    use_external_cal_(true),
    resize_factor_(resize_factor),
    undistort_(undistort),
    world_frame_id_(world_frame_id),
    live_frame_id_(),
    width_(0),
    height_(0),
    K_(K),
    D_(D),
    tf_listener_(nullptr),
    tf_buffer_(),
    image_transport_(nullptr),
    cam_sub_(),
    queue_(queue_size) {
  // Double check intrinsics matrix.
  if (K_(0, 0) <= 0) {
    ROS_ERROR("Camera intrinsics matrix is probably invalid!\n");
    ROS_ERROR_STREAM("K = " << std::endl << K_);
    return;
  }

  // Subscribe to topics.
  image_transport::ImageTransport it_(nh_);
  image_transport_.reset(new image_transport::ImageTransport(nh_));

  cam_sub_ = image_transport_->subscribeCamera("image", 10,
                                               &TrackedImageStream::callback,
                                               this);

  // Set up tf.
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  return;
}

TrackedImageStream::TrackedImageStream(const std::string& world_frame_id,
                                       ros::NodeHandle& nh,
                                       int queue_size) :
    nh_(nh),
    inited_(false),
    use_external_cal_(false),
    resize_factor_(1),
    undistort_(false),
    world_frame_id_(world_frame_id),
    live_frame_id_(),
    width_(0),
    height_(0),
    K_(),
    D_(5),
    tf_listener_(nullptr),
    tf_buffer_(),
    image_transport_(nullptr),
    cam_sub_(),
    queue_(queue_size) {
  // Subscribe to topics.
  image_transport::ImageTransport it_(nh_);
  image_transport_.reset(new image_transport::ImageTransport(nh_));

  cam_sub_ = image_transport_->subscribeCamera("image", 10,
                                               &TrackedImageStream::callback,
                                               this);

  // Set up tf.
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  return;
}

void TrackedImageStream::callback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                                  const sensor_msgs::CameraInfo::ConstPtr& info) {
  ROS_DEBUG("Received image data!");

  // Grab rgb data.
  cv::Mat3b rgb = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;

  assert(rgb.isContinuous());

  if (resize_factor_ != 1) {
    cv::Mat3b resized_rgb(static_cast<float>(rgb.rows)/resize_factor_,
                          static_cast<float>(rgb.cols)/resize_factor_);
    cv::resize(rgb, resized_rgb, resized_rgb.size());
    rgb = resized_rgb;
  }

  if (!inited_) {
    live_frame_id_ = rgb_msg->header.frame_id;

    // Set calibration.
    width_ = rgb.cols;
    height_ = rgb.rows;

    if (!use_external_cal_) {
      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          K_(ii, jj) = info->P[ii*4 + jj];
        }
      }

      if (K_(0, 0) <= 0) {
        ROS_ERROR("Camera intrinsics matrix is probably invalid!\n");
        ROS_ERROR_STREAM("K = " << std::endl << K_);
        return;
      }

      for (int ii = 0; ii < 5; ++ii) {
        D_(ii) = info->D[ii];
      }
    }

    inited_ = true;

    ROS_DEBUG("Set camera calibration!");
  }

  if (undistort_) {
    cv::Mat1f Kcv, Dcv;
    cv::eigen2cv(K_, Kcv);
    cv::eigen2cv(D_, Dcv);
    cv::Mat3b rgb_undistorted;
    cv::undistort(rgb, rgb_undistorted, Kcv, Dcv);
    rgb = rgb_undistorted;
  }

  // Get pose of camera.
  geometry_msgs::TransformStamped tf;
  try {
    // Need to remove leading "/" if it exists.
    std::string rgb_frame_id = rgb_msg->header.frame_id;
    if (rgb_frame_id[0] == '/') {
      rgb_frame_id = rgb_frame_id.substr(1, rgb_frame_id.size()-1);
    }

    tf = tf_buffer_.lookupTransform(world_frame_id_, rgb_frame_id,
                                    ros::Time(rgb_msg->header.stamp),
                                    ros::Duration(1.0/10));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Sophus::SE3f pose;
  tfToSophusSE3<float>(tf.transform, &pose);

  Frame frame;
  frame.id = rgb_msg->header.seq;
  frame.time = rgb_msg->header.stamp.toSec();
  frame.quat = pose.unit_quaternion();
  frame.trans = pose.translation();
  frame.img = rgb;

  queue_.push(frame);

  return;
}

}  // namespace ros_sensor_streams
