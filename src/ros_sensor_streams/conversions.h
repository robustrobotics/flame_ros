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
 * @file conversions.h
 * @author W. Nicholas Greene
 * @date 2017-08-21 21:31:38 (Mon)
 */

#pragma once

#include <sstream>
#include <string>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

namespace ros_sensor_streams {

template <typename Scalar>
std::string eigenQuaternionToString(const Eigen::Quaternion<Scalar>& q) {
  std::stringstream ss;

  ss << "[w = " << q.w() << ", x = " << q.x()
     << ", y = " << q.y() << ", z = " << q.z() << "]";

  return ss.str();
}

template <typename Scalar>
std::string eigenTranslationToString(const Eigen::Matrix<Scalar, 3, 1>& t) {
  std::stringstream ss;

  ss << "[" << t(0) << ", " << t(1) << ", " << t(2) << "]";

  return ss.str();
}

template <typename Scalar>
std::string sophusSE3ToString(const Sophus::SE3Group<Scalar>& se3) {
  std::string q_str = eigenQuaternionToString(se3.unit_quaternion());
  std::string t_str = eigenTranslationToString(se3.translation());

  return q_str + ", " + t_str;
}

template <typename Scalar>
std::string sophusSim3ToString(const Sophus::Sim3Group<Scalar>& sim3) {
  Scalar scale = sim3.scale();
  std::string q_str = eigenQuaternionToString(sim3.quaternion().normalized());
  std::string t_str = eigenTranslationToString(sim3.translation());

  return std::to_string(scale) + ", " + q_str + ", " + t_str;
}

template <typename Scalar>
inline void sophusSE3ToTf(const Sophus::SE3Group<Scalar>& se3,
                          geometry_msgs::Transform* tf) {
  tf->rotation.w = se3.unit_quaternion().w();
  tf->rotation.x = se3.unit_quaternion().x();
  tf->rotation.y = se3.unit_quaternion().y();
  tf->rotation.z = se3.unit_quaternion().z();

  tf->translation.x = se3.translation()(0);
  tf->translation.y = se3.translation()(1);
  tf->translation.z = se3.translation()(2);

  return;
}

template <typename Scalar>
inline void tfToSophusSE3(const geometry_msgs::Transform& tf,
                          Sophus::SE3Group<Scalar>* se3) {
  Eigen::Quaternion<Scalar> q(tf.rotation.w,
                              tf.rotation.x,
                              tf.rotation.y,
                              tf.rotation.z);
  Eigen::Matrix<Scalar, 3, 1> trans(tf.translation.x,
                                    tf.translation.y,
                                    tf.translation.z);
  *se3 = Sophus::SE3Group<Scalar>(q, trans);
  return;
}

template <typename Scalar>
inline void poseToSophusSE3(const geometry_msgs::Pose& pose,
                            Sophus::SE3Group<Scalar>* se3) {
  Eigen::Quaternion<Scalar> q(pose.orientation.w,
                              pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z);
  Eigen::Matrix<Scalar, 3, 1> trans(pose.position.x,
                                    pose.position.y,
                                    pose.position.z);
  *se3 = Sophus::SE3Group<Scalar>(q, trans);
  return;
}

}  // namespace ros_sensor_streams
