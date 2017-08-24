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
 * @file flame_nodelet.cc
 * @author W. Nicholas Greene
 * @date 2016-12-13 17:40:34 (Tue)
 */

#include <stdio.h>
#include <sys/stat.h>

#include <cstdio>
#include <ctime>
#include <csignal>
#include <thread>

#include <memory>
#include <limits>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <nodelet/nodelet.h>

#include <nav_msgs/Path.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>

#include <tf2_ros/transform_listener.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <ros_sensor_streams/tracked_image_stream.h>
#include <ros_sensor_streams/conversions.h>

#ifdef FLAME_WITH_FLA
#include <fla_msgs/ProcessStatus.h>
#endif

#include <flame/flame.h>
#include <flame/utils/image_utils.h>
#include <flame/utils/stats_tracker.h>
#include <flame/utils/load_tracker.h>

#include <flame_ros/FlameStats.h>
#include <flame_ros/FlameNodeletStats.h>

#include "./utils.h"

namespace bfs = boost::filesystem;
namespace fu = flame::utils;

/**
 * @brief Signal handler to debug crashes.
 */
void crash_handler(int sig) {
  FLAME_ASSERT(false);
  return;
}

namespace flame_ros {

/**
 * @brief Runs FLAME using ROS for input/output.
 *
 * Subscribes to calibrated image stream on topics:
 *  - /image
 *  - /camera_info
 *
 * tf for image frame_id must be filled in.
 */
class FlameNodelet : public nodelet::Nodelet {
 public:
  // // Convenience alias.
  using Frame = ros_sensor_streams::TrackedImageStream::Frame;

#ifdef FLAME_WITH_FLA
  enum Status {
    GOOD = 0,
    ALARM_TIMEOUT = 2,
    FAIL_TIMEOUT = 3,
  };
#endif

  /**
   * \brief Constructor.
   *
   * NOTE: Default, no-args constructor must exist.
   */
  FlameNodelet() = default;

  virtual ~FlameNodelet() {
    if (thread_.joinable()) {
      thread_.join();
    }

    return;
  }

  FlameNodelet(const FlameNodelet& rhs) = delete;
  FlameNodelet& operator=(const FlameNodelet& rhs) = delete;

  FlameNodelet(FlameNodelet&& rhs) = delete;
  FlameNodelet& operator=(FlameNodelet&& rhs) = delete;

  /**
   * \brief Nodelet initialization.
   *
   * Subclasses of nodelet::Nodelet need to override this virtual method.
   * It takes the place of the nodelet constructor.
   */
  virtual void onInit() {
    // Install signal handlers.
    // std::signal(SIGINT, crash_handler);
    std::signal(SIGSEGV, crash_handler);
    std::signal(SIGILL, crash_handler);
    std::signal(SIGABRT, crash_handler);
    std::signal(SIGFPE, crash_handler);

    // Grab a handle to the parent node.
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();

    load_ = std::move(fu::LoadTracker(getpid()));

    num_imgs_ = 0;

    // Setup tf.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    /*==================== Input Params ====================*/
    getParamOrFail(pnh, "input/camera_frame_id", &camera_frame_id_);
    getParamOrFail(pnh, "input/camera_world_frame_id", &camera_world_frame_id_);
    getParamOrFail(pnh, "input/subsample_factor", &subsample_factor_);
    getParamOrFail(pnh, "input/poseframe_subsample_factor",
                   &poseframe_subsample_factor_);
    getParamOrFail(pnh, "input/use_poseframe_updates",
                   &use_poseframe_updates_);
    getParamOrFail(pnh, "input/poseframe_child_frame_id", &poseframe_child_frame_id_);
    getParamOrFail(pnh, "input/resize_factor", &resize_factor_);

    /*==================== Output Params ====================*/
    getParamOrFail(pnh, "output/quiet", &params_.debug_quiet);
    getParamOrFail(pnh, "output/mesh", &publish_mesh_);
    getParamOrFail(pnh, "output/idepthmap", &publish_idepthmap_);
    getParamOrFail(pnh, "output/depthmap", &publish_depthmap_);
    getParamOrFail(pnh, "output/cloud", &publish_cloud_);
    getParamOrFail(pnh, "output/features", &publish_features_);
    getParamOrFail(pnh, "output/stats", &publish_stats_);
    getParamOrFail(pnh, "output/load_integration_factor",
                   &load_integration_factor_);
    getParamOrFail(pnh, "output/scene_color_scale", &params_.scene_color_scale);
    getParamOrFail(pnh, "output/filter_oblique_triangles",
                   &params_.do_oblique_triangle_filter);

    double oblique_normal_thresh;
    getParamOrFail(pnh, "output/oblique_normal_thresh", &oblique_normal_thresh);
    params_.oblique_normal_thresh = oblique_normal_thresh;

    getParamOrFail(pnh, "output/oblique_idepth_diff_factor",
                   &params_.oblique_idepth_diff_factor);
    getParamOrFail(pnh, "output/oblique_idepth_diff_abs",
                   &params_.oblique_idepth_diff_abs);

    getParamOrFail(pnh, "output/filter_long_edges",
                   &params_.do_edge_length_filter);

    double edge_length_thresh;
    getParamOrFail(pnh, "output/edge_length_thresh", &edge_length_thresh);
    params_.edge_length_thresh = edge_length_thresh;

    getParamOrFail(pnh, "output/filter_triangles_by_idepth",
                   &params_.do_idepth_triangle_filter);

    double min_triangle_idepth;
    getParamOrFail(pnh, "output/min_triangle_idepth", &min_triangle_idepth);
    params_.min_triangle_idepth = min_triangle_idepth;

    getParamOrFail(pnh, "output/max_angular_rate", &max_angular_rate_);

    /*==================== Debug Params ====================*/
    getParamOrFail(pnh, "debug/wireframe", &params_.debug_draw_wireframe);
    getParamOrFail(pnh, "debug/features", &params_.debug_draw_features);
    getParamOrFail(pnh, "debug/detections", &params_.debug_draw_detections);
    getParamOrFail(pnh, "debug/matches", &params_.debug_draw_matches);
    getParamOrFail(pnh, "debug/normals", &params_.debug_draw_normals);
    getParamOrFail(pnh, "debug/idepthmap", &params_.debug_draw_idepthmap);
    getParamOrFail(pnh, "debug/text_overlay", &params_.debug_draw_text_overlay);
    getParamOrFail(pnh, "debug/flip_images", &params_.debug_flip_images);

    /*==================== Threading Params ====================*/
    getParamOrFail(pnh, "threading/openmp/num_threads", &params_.omp_num_threads);
    getParamOrFail(pnh, "threading/openmp/chunk_size", &params_.omp_chunk_size);

    /*==================== Feature Params ====================*/
    getParamOrFail(pnh, "features/do_letterbox", &params_.do_letterbox);
    getParamOrFail(pnh, "features/detection/min_grad_mag", &params_.min_grad_mag);
    params_.fparams.min_grad_mag = params_.min_grad_mag;

    double min_error;
    getParamOrFail(pnh, "features/detection/min_error", &min_error);
    params_.min_error = min_error;

    getParamOrFail(pnh, "features/detection/win_size", &params_.detection_win_size);

    int win_size;
    getParamOrFail(pnh, "features/tracking/win_size", &win_size);
    params_.zparams.win_size = win_size;
    params_.fparams.win_size = win_size;

    getParamOrFail(pnh, "features/tracking/max_dropouts", &params_.max_dropouts);

    double epipolar_line_var;
    getParamOrFail(pnh, "features/tracking/epipolar_line_var",
                   &epipolar_line_var);
    params_.zparams.epipolar_line_var = epipolar_line_var;

    /*==================== Regularization Params ====================*/
    getParamOrFail(pnh, "regularization/do_nltgv2", &params_.do_nltgv2);
    getParamOrFail(pnh, "regularization/nltgv2/adaptive_data_weights",
                   &params_.adaptive_data_weights);
    getParamOrFail(pnh, "regularization/nltgv2/rescale_data", &params_.rescale_data);
    getParamOrFail(pnh, "regularization/nltgv2/init_with_prediction",
                   &params_.init_with_prediction);
    getParamOrFail(pnh, "regularization/nltgv2/idepth_var_max",
                   &params_.idepth_var_max_graph);
    getParamOrFail(pnh, "regularization/nltgv2/data_factor", &params_.rparams.data_factor);
    getParamOrFail(pnh, "regularization/nltgv2/step_x", &params_.rparams.step_x);
    getParamOrFail(pnh, "regularization/nltgv2/step_q", &params_.rparams.step_q);
    getParamOrFail(pnh, "regularization/nltgv2/theta", &params_.rparams.theta);
    getParamOrFail(pnh, "regularization/nltgv2/min_height", &params_.min_height);
    getParamOrFail(pnh, "regularization/nltgv2/max_height", &params_.max_height);
    getParamOrFail(pnh, "regularization/nltgv2/check_sticky_obstacles",
                   &params_.check_sticky_obstacles);

#ifdef FLAME_WITH_FLA
    bool use_camera_info = false;
    getParamOrFail(pnh, "input/use_camera_info", &use_camera_info);

    if (use_camera_info) {
      // Make sure we don't attempt to resize the image.
      FLAME_ASSERT(resize_factor_ == 1);

      // Setup input stream.
      input_ = std::make_shared<ros_sensor_streams::
          TrackedImageStream>(camera_world_frame_id_, nh);
    } else {
      // FLA does not follow ROS conventions for camera/image streams/calibration,
      // so we need to do some extra work to pass data through. First, the
      // camera_info messages containing the camera intrinsics/distortion are not
      // filled in. We need to grab the calibration from samros's params. Since
      // samwise estimates the paramters on the fly, we should really use the
      // refined versions, but this should work for now. Finally, we need to
      // undistort the images.
      int width, height;
      getParamOrFail(nh, "/samros/camera/image_width", &width);
      getParamOrFail(nh, "/samros/camera/image_height", &height);

      if (((width != 640) && (width != 1280)) ||
          ((height != 512) && (height != 1024))) {
        ROS_ERROR("FlameNodelet: Unexpected image size = (%i, % i)\n",
                  width, height);
      }

      double fx, fy, cx, cy;
      getParamOrFail(nh, "/samros/camera/intrinsics/fu", &fx);
      getParamOrFail(nh, "/samros/camera/intrinsics/fv", &fy);
      getParamOrFail(nh, "/samros/camera/intrinsics/pu", &cx);
      getParamOrFail(nh, "/samros/camera/intrinsics/pv", &cy);

      double k1, k2, p1, p2, k3;
      getParamOrFail(nh, "/samros/camera/distortion/k1", &k1);
      getParamOrFail(nh, "/samros/camera/distortion/k2", &k2);
      getParamOrFail(nh, "/samros/camera/distortion/p1", &p1);
      getParamOrFail(nh, "/samros/camera/distortion/p2", &p2);
      getParamOrFail(nh, "/samros/camera/distortion/k3", &k3);

      Eigen::VectorXf D(5);
      D << k1, k2, p1, p2, k3;

      Eigen::Matrix3f K(Eigen::Matrix3f::Zero());
      K(0, 0) = fx;
      K(0, 2) = cx;
      K(1, 1) = fy;
      K(1, 2) = cy;
      K(2, 2) = 1.0f;

      // Adjust calibration based on resize_factor.
      K /= resize_factor_;
      K(2, 2) = 1.0f;

      // Setup input stream.
      bool undistort = true;
      input_ = std::make_shared<ros_sensor_streams::
                                TrackedImageStream>(camera_world_frame_id_, nh, K,
                                                    D, undistort, resize_factor_);
    }

    // Setup health and status.
    int tmp; // getParam can't handle uint8_t.
    getParamOrFail(pnh, "fla/node_id", &tmp);
    node_id_ = tmp;

    getParamOrFail(pnh, "fla/heart_beat_dt", &heart_beat_dt_);
    getParamOrFail(pnh, "fla/alarm_timeout", &alarm_timeout_);
    getParamOrFail(pnh, "fla/fail_timeout", &fail_timeout_);

    heart_beat_ = nh.createTimer(ros::Duration(heart_beat_dt_),
                                 &FlameNodelet::heartBeat, this);
    heart_beat_pub_ = nh.advertise<fla_msgs::ProcessStatus>("/globalstatus", 1);
#else
    // Image resizing not supported for non-FLA.
    FLAME_ASSERT(resize_factor_ == 1);

    // Setup input stream.
    input_ = std::make_shared<ros_sensor_streams::
                              TrackedImageStream>(camera_world_frame_id_, nh);
#endif

    pfs_inited_ = true; // Default values.
    first_pf_id_ = 0;
    if (use_poseframe_updates_) {
      // Wait until first pf message received with >= 2 pfs so that we can
      // estimate the poseframe subsample factor and offset.
      pfs_inited_ = false;

      // Subscribe to poseframe topic.
      poseframe_sub_ = nh.subscribe("poseframes", 1,
                                    &FlameNodelet::poseframeCallback, this);
    }

    // Set up publishers. For some reason this appears to take a while.
    NODELET_INFO_COND(!params_.debug_quiet, "FlameNodelet: Setting up publishers...\n");

    it_ = std::make_shared<image_transport::ImageTransport>(nh);

    if (publish_idepthmap_) {
      idepth_pub_ = it_->advertiseCamera("idepth_registered/image_rect", 5);
    }
    if (publish_depthmap_) {
      depth_pub_ = it_->advertiseCamera("depth_registered/image_rect", 5);
    }
    if (publish_features_) {
      features_pub_ = it_->advertiseCamera("depth_registered_raw/image_rect", 5);
    }
    if (publish_mesh_) {
      mesh_pub_ = nh.advertise<pcl_msgs::PolygonMesh>("mesh", 5);
    }
    if (publish_cloud_) {
      cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 5);
    }
    if (publish_stats_) {
      stats_pub_ = nh.advertise<FlameStats>("stats", 5);
      nodelet_stats_pub_ = nh.advertise<FlameNodeletStats>("nodelet_stats", 5);
    }

    if (params_.debug_draw_wireframe) {
      debug_wireframe_pub_ = it_->advertise("debug/wireframe", 1);
    }
    if (params_.debug_draw_features) {
      debug_features_pub_ = it_->advertise("debug/features", 1);
    }
    if (params_.debug_draw_detections) {
      debug_detections_pub_ = it_->advertise("debug/detections", 1);
    }
    if (params_.debug_draw_matches) {
      debug_matches_pub_ = it_->advertise("debug/matches", 1);
    }
    if (params_.debug_draw_normals) {
      debug_normals_pub_ = it_->advertise("debug/normals", 1);
    }
    if (params_.debug_draw_idepthmap) {
      debug_idepthmap_pub_ = it_->advertise("debug/idepthmap", 1);
    }

    // Kick off main thread.
    thread_ = std::thread(&FlameNodelet::main, this);

    return;
  }

  /**
   * @brief Callback for receiving poseframe poses.
   */
  void poseframeCallback(const nav_msgs::Path::ConstPtr& msg) {
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet: Got a poseframe message!\n");

    // Get transform to camera_world.
    geometry_msgs::TransformStamped tf;
    try {
      // Need to remove leading "/" if it exists.
      std::string frame_id = msg->header.frame_id;
      if (frame_id[0] == '/') {
        frame_id = frame_id.substr(1, frame_id.size()-1);
      }
      tf = tf_buffer_.lookupTransform(camera_world_frame_id_, frame_id,
                                      ros::Time(msg->header.stamp),
                                      ros::Duration(1.0/15));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    Sophus::SE3f to_camera_world;
    ros_sensor_streams::tfToSophusSE3<float>(tf.transform, &to_camera_world);

    // Get transform of camera wrt body.
    try {
      // Need to remove leading "/" if it exists.
      std::string frame_id = msg->header.frame_id;
      if (frame_id[0] == '/') {
        frame_id = frame_id.substr(1, frame_id.size()-1);
      }
      tf = tf_buffer_.lookupTransform(poseframe_child_frame_id_, camera_frame_id_,
                                      ros::Time(msg->header.stamp),
                                      ros::Duration(1.0/15));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    Sophus::SE3f to_camera;
    ros_sensor_streams::tfToSophusSE3<float>(tf.transform, &to_camera);

    // Extract ids and poses.
    std::vector<uint32_t> pf_ids(msg->poses.size());
    std::vector<Sophus::SE3f> pf_poses(msg->poses.size());
    for (int ii = 0; ii < msg->poses.size(); ++ii) {
      pf_ids[ii] = msg->poses[ii].header.seq;

      Sophus::SE3f pose;
      ros_sensor_streams::poseToSophusSE3<float>(msg->poses[ii].pose, &pose);

      // Convert to camera world.
      pf_poses[ii] = to_camera_world * pose * to_camera;
    }

    if (!pfs_inited_ && (pf_ids.size() >= 2)) {
      // Initialize pf offset and subsample factor.
      first_pf_id_ = pf_ids[0];
      poseframe_subsample_factor_ = pf_ids[1] - pf_ids[0];
      pfs_inited_ = true;
    } else if ((sensor_ != nullptr) && pfs_inited_) {
      sensor_->updatePoseFramePoses(pf_ids, pf_poses);
      sensor_->prunePoseFrames(pf_ids);
    }

    return;
  }

#ifdef FLAME_WITH_FLA
  void heartBeat(const ros::TimerEvent&) {
    double now = ros::Time::now().toSec();

    fla_msgs::ProcessStatus::Ptr ps(new fla_msgs::ProcessStatus);

    ps->id = node_id_;
    ps->pid = getpid();

    if (now - last_update_sec_ > alarm_timeout_)  {
      ps->status = fla_msgs::ProcessStatus::ALARM;
      ps->arg = Status::ALARM_TIMEOUT; // Time since last update longer than expected.
    } else if (now - last_update_sec_ > fail_timeout_) {
      ps->status = fla_msgs::ProcessStatus::FAIL;
      ps->arg = Status::FAIL_TIMEOUT; // Time since last update probably error.
    } else {
      ps->status = fla_msgs::ProcessStatus::READY;
      ps->arg = Status::GOOD; // All good.
    }

    heart_beat_pub_.publish(ps);

    return;
  }
#endif

  /**
   * \brief Main processing loop.
   */
  void main() {
    // Wait until input is initialized.
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet: Waiting on calibration...\n");
    while (!input_->inited()) {
      std::this_thread::yield();
    }

    Kinv_ = input_->K().inverse();

    // Initialize depth sensor.
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet: Constructing Flame...\n");
    sensor_ = std::make_shared<flame::Flame>(input_->width(),
                                             input_->height(),
                                             input_->K(),
                                             Kinv_,
                                             params_);

    /*==================== Enter main loop ====================*/
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet: Done. We are GO for launch!\n");
    while (ros::ok()) {
      stats_.tick("main");

      // Wait for queue to have items.
      stats_.set("queue_size", input_->queue().size());
      stats_.tick("waiting");
      std::unique_lock<std::recursive_mutex> lock(input_->queue().mutex());
      input_->queue().non_empty().wait(lock, [this](){
          return (input_->queue().size() > 0);
        });
      lock.unlock();
      stats_.tock("waiting");
      NODELET_INFO_COND(!params_.debug_quiet,
                        "FlameNodelet/waiting = %4.1fms, queue_size = %i\n",
                        stats_.timings("waiting"),
                        static_cast<int>(stats_.stats("queue_size")));

      // Grab the first item in the queue.
      Frame frame = input_->queue().front();
      input_->queue().pop();
      if ((pfs_inited_) && (num_imgs_ % subsample_factor_ == 0)) {
        // Eat data.
        processFrame(frame.id, frame.time, Sophus::SE3f(frame.quat, frame.trans),
                     frame.img);
      }

      /*==================== Timing stuff ====================*/
      // Compute two measures of throughput in Hz. The first is the actual number of
      // frames per second, the second is the theoretical maximum fps based on the
      // runtime. They are not necessarily the same - the former takes external
      // latencies into account.

      // Compute maximum fps based on runtime.
      double fps_max = 0.0f;
      if (stats_.stats("fps_max") <= 0.0f) {
        fps_max = 1000.0f / stats_.timings("main");
      } else {
        fps_max = 1.0f / (0.99 * 1.0f/stats_.stats("fps_max") +
                          0.01 * stats_.timings("main")/1000.0f);
      }
      stats_.set("fps_max", fps_max);

      // Compute actual fps (overall throughput of system).
      stats_.tock("fps");
      double fps = 0.0;
      if (stats_.stats("fps") <= 0.0f) {
        fps = 1000.0f / stats_.timings("fps");
      } else {
        fps = 1.0f / (0.99 * 1.0f/stats_.stats("fps") +
                      0.01 * stats_.timings("fps")/1000.0f);
      }
      stats_.set("fps", fps);
      stats_.tick("fps");

#ifdef FLAME_WITH_FLA
      last_update_sec_ = ros::Time::now().toSec();
#endif

      stats_.tock("main");

      if ((num_imgs_ % load_integration_factor_) == 0) {
        // Compute load stats.
        fu::Load max_load, sys_load, pid_load;
        load_.get(&max_load, &sys_load, &pid_load);
        stats_.set("max_load_cpu", max_load.cpu);
        stats_.set("max_load_mem", max_load.mem);
        stats_.set("max_load_swap", max_load.swap);
        stats_.set("sys_load_cpu", sys_load.cpu);
        stats_.set("sys_load_mem", sys_load.mem);
        stats_.set("sys_load_swap", sys_load.swap);
        stats_.set("pid_load_cpu", pid_load.cpu);
        stats_.set("pid_load_mem", pid_load.mem);
        stats_.set("pid_load_swap", pid_load.swap);
        stats_.set("pid", getpid());
      }

      publishFlameNodeletStats(nodelet_stats_pub_,
                               frame.id, frame.time,
                               stats_.stats(), stats_.timings());

      NODELET_INFO_COND(!params_.debug_quiet,
                        "FlameNodelet/main(%i/%u) = %4.1fms/%.1fHz (%.1fHz)\n",
                        num_imgs_, frame.id, stats_.timings("main"),
                        stats_.stats("fps_max"), stats_.stats("fps"));

      num_imgs_++;
    }

    return;
  }

  void processFrame(const uint32_t img_id, const double time,
                    const Sophus::SE3f& pose, const cv::Mat3b& rgb) {
    stats_.tick("process_frame");

    /*==================== Process image ====================*/
    // Convert to grayscale.
    cv::Mat1b img_gray;
    cv::cvtColor(rgb, img_gray, cv::COLOR_RGB2GRAY);

    bool is_poseframe = ((static_cast<int>(img_id) -  first_pf_id_) %
                         poseframe_subsample_factor_) == 0;
    bool update_success = sensor_->update(time, img_id, pose, img_gray,
                                          is_poseframe);

    if (!update_success) {
      stats_.tock("process_frame");
      ROS_WARN_COND(!params_.debug_quiet,
                    "FlameNodelet: Unsuccessful update.\n");
      return;
    }

    if (max_angular_rate_ > 0.0f) {
      // Check angle difference between last and current pose. If we're rotating,
      // we shouldn't publish output since it's probably too noisy.
      Eigen::Quaternionf q_delta = pose.unit_quaternion() *
          prev_pose_.unit_quaternion().inverse();
      float angle_delta = fu::fast_abs(Eigen::AngleAxisf(q_delta).angle());
      float angle_rate = angle_delta / (time - prev_time_);

      prev_time_ = time;
      prev_pose_ = pose;

      if (angle_rate * 180.0f / M_PI > max_angular_rate_) {
        // Angular rate is too high.
        ROS_ERROR_COND(!params_.debug_quiet,
                       "Angle Delta = %.3f, rate = %f.3\n", angle_delta * 180.0f / M_PI,
                       angle_rate * 180.0f / M_PI);
        return;
      }
    }

    /*==================== Publish output ====================*/
    stats_.tick("publishing");

    if (publish_mesh_) {
      // Get current mesh.
      std::vector<cv::Point2f> vtx;
      std::vector<float> idepths;
      std::vector<Eigen::Vector3f> normals;
      std::vector<flame::Triangle> triangles;
      std::vector<flame::Edge> edges;
      std::vector<bool> tri_validity;
      sensor_->getInverseDepthMesh(&vtx, &idepths, &normals, &triangles,
                                   &tri_validity, &edges);
      publishDepthMesh(mesh_pub_, camera_frame_id_, time, Kinv_, vtx,
                       idepths, normals, triangles, tri_validity, rgb);
    }

    if (publish_idepthmap_ || publish_depthmap_ || publish_cloud_) {
      cv::Mat1f idepthmap;
      sensor_->getFilteredInverseDepthMap(&idepthmap);

      if (publish_idepthmap_) {
        // Publish full idepthmap.
        publishDepthMap(idepth_pub_, camera_frame_id_, time, input_->K(),
                        sensor_->getInverseDepthMap());
      }

      // Convert to depths.
      cv::Mat1f depth_est(idepthmap.rows, idepthmap.cols,
                          std::numeric_limits<float>::quiet_NaN());
#pragma omp parallel for collapse(2) num_threads(params_.omp_num_threads) schedule(dynamic, params_.omp_chunk_size) // NOLINT
      for (int ii = 0; ii < depth_est.rows; ++ii) {
        for (int jj = 0; jj < depth_est.cols; ++jj) {
          float idepth =  idepthmap(ii, jj);
          if (!std::isnan(idepth) && (idepth > 0)) {
            depth_est(ii, jj) = 1.0f/ idepth;
          }
        }
      }

      if (publish_depthmap_) {
        publishDepthMap(depth_pub_, input_->live_frame_id(), time, input_->K(),
                        depth_est);
      }

      if (publish_cloud_) {
        float max_depth = (params_.do_idepth_triangle_filter) ?
            1.0f / params_.min_triangle_idepth : std::numeric_limits<float>::max();
        publishPointCloud(cloud_pub_, input_->live_frame_id(), time, input_->K(),
                          depth_est, 0.1f, max_depth);
      }
    }

    if (publish_features_) {
      cv::Mat1f depth_raw(img_gray.rows, img_gray.cols,
                          std::numeric_limits<float>::quiet_NaN());
      if (publish_features_) {
        std::vector<cv::Point2f> vertices;
        std::vector<float> idepths_mu, idepths_var;
        sensor_->getRawIDepths(&vertices, &idepths_mu, &idepths_var);

        for (int ii = 0; ii < vertices.size(); ++ii) {
          float id = idepths_mu[ii];
          float var = idepths_var[ii];
          if (!std::isnan(id) && (id > 0)) {
            int x = fu::fast_roundf(vertices[ii].x);
            int y = fu::fast_roundf(vertices[ii].y);

            FLAME_ASSERT(x >= 0);
            FLAME_ASSERT(x < depth_raw.cols);
            FLAME_ASSERT(y >= 0);
            FLAME_ASSERT(y < depth_raw.rows);

            depth_raw(y, x) = 1.0f / id;
          }
        }
      }

      publishDepthMap(features_pub_, input_->live_frame_id(), time, input_->K(),
                      depth_raw);
    }

    if (publish_stats_) {
      auto stats = sensor_->stats().stats();
      auto timings = sensor_->stats().timings();
      publishFlameStats(stats_pub_, img_id, time, stats, timings);
    }

    stats_.set("latency", (ros::Time::now().toSec() - time) * 1000);
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet/latency = %4.1fms\n",
                      stats_.stats("latency"));

    stats_.tock("publishing");
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet/publishing = %4.1fms\n",
                      stats_.timings("publishing"));

    /*==================== Publish debug stuff ====================*/
    stats_.tick("debug_publishing");

    std_msgs::Header hdr;
    hdr.stamp.fromSec(time);
    hdr.frame_id = input_->live_frame_id();

    if (params_.debug_draw_wireframe) {
      sensor_msgs::Image::Ptr debug_img_msg =
          cv_bridge::CvImage(hdr, "bgr8",
                             sensor_->getDebugImageWireframe()).toImageMsg();
      debug_wireframe_pub_.publish(debug_img_msg);
    }

    if (params_.debug_draw_features) {
      sensor_msgs::Image::Ptr debug_img_msg =
          cv_bridge::CvImage(hdr, "bgr8",
                             sensor_->getDebugImageFeatures()).toImageMsg();
      debug_features_pub_.publish(debug_img_msg);
    }

    if (params_.debug_draw_detections) {
      sensor_msgs::Image::Ptr debug_img_msg =
          cv_bridge::CvImage(hdr, "bgr8",
                             sensor_->getDebugImageDetections()).toImageMsg();
      debug_detections_pub_.publish(debug_img_msg);
    }

    if (params_.debug_draw_matches) {
      sensor_msgs::Image::Ptr debug_img_msg =
          cv_bridge::CvImage(hdr, "bgr8",
                             sensor_->getDebugImageMatches()).toImageMsg();
      debug_matches_pub_.publish(debug_img_msg);
    }

    if (params_.debug_draw_normals) {
      sensor_msgs::Image::Ptr debug_img_msg =
          cv_bridge::CvImage(hdr, "bgr8",
                             sensor_->getDebugImageNormals()).toImageMsg();
      debug_normals_pub_.publish(debug_img_msg);
    }

    if (params_.debug_draw_idepthmap) {
      sensor_msgs::Image::Ptr debug_img_msg =
          cv_bridge::CvImage(hdr, "bgr8",
                             sensor_->getDebugImageInverseDepthMap()).toImageMsg();
      debug_idepthmap_pub_.publish(debug_img_msg);
    }

    stats_.tock("debug_publishing");
    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet/debug_publishing = %4.1fms\n",
                      stats_.timings("debug_publishing"));

    stats_.tock("process_frame");

    NODELET_INFO_COND(!params_.debug_quiet,
                      "FlameNodelet/process_frame = %4.1fms\n",
                      stats_.timings("process_frame"));

    return;
  }

 private:
  std::thread thread_;

  // Keeps track of stats and load.
  fu::StatsTracker stats_;
  fu::LoadTracker load_;

  // Number of images processed.
  int num_imgs_;

  // tf stuff.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  // input params.
  std::string camera_frame_id_; // Frame id of the camera in optical coordinates.
  std::string camera_world_frame_id_; // Frame id of the world in camera optical coordinates.
  int subsample_factor_; // Process one out of this many images.
  int poseframe_subsample_factor_; // Create a poseframe every this number of images.
  int resize_factor_;

  // Input stream object.
  std::shared_ptr<ros_sensor_streams::TrackedImageStream> input_;
  Eigen::Matrix3f Kinv_;

  // PoseFrame stuff.
  std::string poseframe_child_frame_id_; // Frame inside pf messages.
  bool pfs_inited_; // False until first poseframe message received with > 2 pfs.
  uint32_t first_pf_id_; // ID of first poseframe.
  bool use_poseframe_updates_;
  ros::Subscriber poseframe_sub_;

  // Stuff for checking angular rates.
  float max_angular_rate_;
  double prev_time_;
  Sophus::SE3f prev_pose_;

  // Depth sensor.
  flame::Params params_;
  std::shared_ptr<flame::Flame> sensor_;

  // Publishes mesh.
  bool publish_mesh_;
  ros::Publisher mesh_pub_;

  // Publishes depthmap.
  cv::Mat1f idepthmap_;
  bool publish_idepthmap_;
  bool publish_depthmap_;
  bool publish_features_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher idepth_pub_;
  image_transport::CameraPublisher depth_pub_;
  image_transport::CameraPublisher features_pub_;

  // Publish pointcloud.
  bool publish_cloud_;
  ros::Publisher cloud_pub_;

  // Publishes statistics.
  bool publish_stats_;
  ros::Publisher stats_pub_;
  ros::Publisher nodelet_stats_pub_;
  int load_integration_factor_;

  // Publishes debug images.
  image_transport::Publisher debug_wireframe_pub_;
  image_transport::Publisher debug_features_pub_;
  image_transport::Publisher debug_detections_pub_;
  image_transport::Publisher debug_matches_pub_;
  image_transport::Publisher debug_normals_pub_;
  image_transport::Publisher debug_idepthmap_pub_;

#ifdef FLAME_WITH_FLA
  uint8_t node_id_;
  double heart_beat_dt_;
  double alarm_timeout_;
  double fail_timeout_;
  ros::Timer heart_beat_;
  ros::Publisher heart_beat_pub_;
  double last_update_sec_;
#endif
};

}  // namespace flame_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(flame_ros::FlameNodelet, nodelet::Nodelet)
