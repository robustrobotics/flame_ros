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
 * @file flame_offline_asl.cc
 * @author W. Nicholas Greene
 * @date 2017-07-29 19:32:58 (Sat)
 */

#include <stdio.h>
#include <sys/stat.h>

#include <cstdio>
#include <ctime>
#include <csignal>

#include <memory>
#include <limits>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <ros_sensor_streams/asl_rgbd_offline_stream.h>

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
 * @brief Runs FLAME on a dataset of images.

 * Input is taken from data in ASL format:
 * http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
 *
 * Three ASL folders are needed:
 *   1. Pose data relative to a given world frame.
 *   2. RGB data
 *   3. Depthmap data (for comparison)
 */
class FlameOffline final {
 public:
  /**
   * @brief Constructor.
   */
  FlameOffline() :
      stats_(),
      load_(getpid()),
      nh_("flame"),
      pnh_("~"),
      num_imgs_(0),
      camera_world_frame_id_(),
      camera_frame_id_(),
      output_dir_(),
      pass_in_truth_(false),
      input_(nullptr),
      Kinv_(),
      max_angular_rate_(0.0f),
      prev_time_(0),
      prev_pose_(),
      params_(),
      sensor_(nullptr),
      it_(nh_),
      depth_pub_() {
    /*==================== Data Params ====================*/
    std::string pose_path;
    getParamOrFail(pnh_, "pose_path", &pose_path);

    std::string rgb_path;
    getParamOrFail(pnh_, "rgb_path", &rgb_path);

    std::string depth_path;
    getParamOrFail(pnh_, "depth_path", &depth_path);

    std::string world_frame_str;
    getParamOrFail(pnh_, "world_frame", &world_frame_str);

    ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame world_frame;
    if (world_frame_str == "RDF") {
      world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::RDF;
    } else if (world_frame_str == "FLU") {
      world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::FLU;
    } else if (world_frame_str == "FRD") {
      world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::FRD;
    } else if (world_frame_str == "RFU") {
      world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::RFU;
    } else {
      ROS_ERROR("Unknown world frame!\n");
      return;
    }

    /*==================== Input Params ====================*/
    getParamOrFail(pnh_, "input/camera_frame_id", &camera_frame_id_);
    getParamOrFail(pnh_, "input/camera_world_frame_id", &camera_world_frame_id_);
    getParamOrFail(pnh_, "input/subsample_factor", &subsample_factor_);
    getParamOrFail(pnh_, "input/poseframe_subsample_factor",
                   &poseframe_subsample_factor_);

    getParamOrFail(pnh_, "input/resize_factor", &resize_factor_);

    getParamOrFail(pnh_, "input/rate", &rate_);

    /*==================== Output Params ====================*/
    getParamOrFail(pnh_, "output/quiet", &params_.debug_quiet);
    getParamOrFail(pnh_, "output/mesh", &publish_mesh_);
    getParamOrFail(pnh_, "output/idepthmap", &publish_idepthmap_);
    getParamOrFail(pnh_, "output/depthmap", &publish_depthmap_);
    getParamOrFail(pnh_, "output/cloud", &publish_cloud_);
    getParamOrFail(pnh_, "output/features", &publish_features_);
    getParamOrFail(pnh_, "output/stats", &publish_stats_);
    getParamOrFail(pnh_, "output/load_integration_factor",
                   &load_integration_factor_);
    getParamOrFail(pnh_, "output/scene_color_scale", &params_.scene_color_scale);
    getParamOrFail(pnh_, "output/filter_oblique_triangles",
                   &params_.do_oblique_triangle_filter);

    double oblique_normal_thresh;
    getParamOrFail(pnh_, "output/oblique_normal_thresh", &oblique_normal_thresh);
    params_.oblique_normal_thresh = oblique_normal_thresh;

    getParamOrFail(pnh_, "output/oblique_idepth_diff_factor",
                   &params_.oblique_idepth_diff_factor);
    getParamOrFail(pnh_, "output/oblique_idepth_diff_abs",
                   &params_.oblique_idepth_diff_abs);

    getParamOrFail(pnh_, "output/filter_long_edges",
                   &params_.do_edge_length_filter);

    double edge_length_thresh;
    getParamOrFail(pnh_, "output/edge_length_thresh", &edge_length_thresh);
    params_.edge_length_thresh = edge_length_thresh;

    getParamOrFail(pnh_, "output/filter_triangles_by_idepth",
                   &params_.do_idepth_triangle_filter);

    double min_triangle_idepth;
    getParamOrFail(pnh_, "output/min_triangle_idepth", &min_triangle_idepth);
    params_.min_triangle_idepth = min_triangle_idepth;

    getParamOrFail(pnh_, "output/max_angular_rate", &max_angular_rate_);

    /*==================== Debug Params ====================*/
    getParamOrFail(pnh_, "debug/wireframe", &params_.debug_draw_wireframe);
    getParamOrFail(pnh_, "debug/features", &params_.debug_draw_features);
    getParamOrFail(pnh_, "debug/detections", &params_.debug_draw_detections);
    getParamOrFail(pnh_, "debug/matches", &params_.debug_draw_matches);
    getParamOrFail(pnh_, "debug/normals", &params_.debug_draw_normals);
    getParamOrFail(pnh_, "debug/idepthmap", &params_.debug_draw_idepthmap);
    getParamOrFail(pnh_, "debug/text_overlay", &params_.debug_draw_text_overlay);
    getParamOrFail(pnh_, "debug/flip_images", &params_.debug_flip_images);

    /*==================== Threading Params ====================*/
    getParamOrFail(pnh_, "threading/openmp/num_threads", &params_.omp_num_threads);
    getParamOrFail(pnh_, "threading/openmp/chunk_size", &params_.omp_chunk_size);

    /*==================== Features Params ====================*/
    getParamOrFail(pnh_, "features/do_letterbox", &params_.do_letterbox);
    getParamOrFail(pnh_, "features/detection/min_grad_mag", &params_.min_grad_mag);
    params_.fparams.min_grad_mag = params_.min_grad_mag;

    double min_error;
    getParamOrFail(pnh_, "features/detection/min_error", &min_error);
    params_.min_error = min_error;

    getParamOrFail(pnh_, "features/detection/win_size", &params_.detection_win_size);

    int win_size;
    getParamOrFail(pnh_, "features/tracking/win_size", &win_size);
    params_.zparams.win_size = win_size;
    params_.fparams.win_size = win_size;

    getParamOrFail(pnh_, "features/tracking/max_dropouts", &params_.max_dropouts);

    double epipolar_line_var;
    getParamOrFail(pnh_, "features/tracking/epipolar_line_var",
                   &epipolar_line_var);
    params_.zparams.epipolar_line_var = epipolar_line_var;

    /*==================== Regularizer Params ====================*/
    getParamOrFail(pnh_, "regularization/do_nltgv2", &params_.do_nltgv2);
    getParamOrFail(pnh_, "regularization/nltgv2/adaptive_data_weights",
                   &params_.adaptive_data_weights);
    getParamOrFail(pnh_, "regularization/nltgv2/rescale_data", &params_.rescale_data);
    getParamOrFail(pnh_, "regularization/nltgv2/init_with_prediction",
                   &params_.init_with_prediction);
    getParamOrFail(pnh_, "regularization/nltgv2/idepth_var_max",
                   &params_.idepth_var_max_graph);
    getParamOrFail(pnh_, "regularization/nltgv2/data_factor", &params_.rparams.data_factor);
    getParamOrFail(pnh_, "regularization/nltgv2/step_x", &params_.rparams.step_x);
    getParamOrFail(pnh_, "regularization/nltgv2/step_q", &params_.rparams.step_q);
    getParamOrFail(pnh_, "regularization/nltgv2/theta", &params_.rparams.theta);
    getParamOrFail(pnh_, "regularization/nltgv2/min_height", &params_.min_height);
    getParamOrFail(pnh_, "regularization/nltgv2/max_height", &params_.max_height);
    getParamOrFail(pnh_, "regularization/nltgv2/check_sticky_obstacles",
                   &params_.check_sticky_obstacles);

    /*==================== Analysis Params ====================*/
    getParamOrFail(pnh_, "analysis/pass_in_truth", &pass_in_truth_);

    // Setup input stream.
    FLAME_ASSERT(resize_factor_ == 1);
    input_ = std::make_shared<ros_sensor_streams::
        ASLRGBDOfflineStream>(nh_,
                              pose_path,
                              rgb_path,
                              depth_path,
                              "camera",
                              camera_world_frame_id_,
                              camera_frame_id_,
                              world_frame);

    // Set up publishers.
    if (publish_idepthmap_) {
      idepth_pub_ = it_.advertiseCamera("idepth_registered/image_rect", 5);
    }
    if (publish_depthmap_) {
      depth_pub_ = it_.advertiseCamera("depth_registered/image_rect", 5);
    }
    if (publish_features_) {
      features_pub_ = it_.advertiseCamera("depth_registered_raw/image_rect", 5);
    }
    if (publish_mesh_) {
      mesh_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>("mesh", 5);
    }
    if (publish_cloud_) {
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 5);
    }
    if (publish_stats_) {
      stats_pub_ = nh_.advertise<FlameStats>("stats", 5);
      nodelet_stats_pub_ = nh_.advertise<FlameNodeletStats>("nodelet_stats", 5);
    }

    if (params_.debug_draw_wireframe) {
      debug_wireframe_pub_ = it_.advertise("debug/wireframe", 1);
    }
    if (params_.debug_draw_features) {
      debug_features_pub_ = it_.advertise("debug/features", 1);
    }
    if (params_.debug_draw_detections) {
      debug_detections_pub_ = it_.advertise("debug/detections", 1);
    }
    if (params_.debug_draw_matches) {
      debug_matches_pub_ = it_.advertise("debug/matches", 1);
    }
    if (params_.debug_draw_normals) {
      debug_normals_pub_ = it_.advertise("debug/normals", 1);
    }
    if (params_.debug_draw_idepthmap) {
      debug_idepthmap_pub_ = it_.advertise("debug/idepthmap", 1);
    }

    return;
  }

  ~FlameOffline() = default;

  FlameOffline(const FlameOffline& rhs) = delete;
  FlameOffline& operator=(const FlameOffline& rhs) = delete;

  FlameOffline(FlameOffline&& rhs) = delete;
  FlameOffline& operator=(FlameOffline&& rhs) = delete;

  /**
   * @brief Compute some statistics using depth groundtruth.
   */
  void getTruthStats(double timestamp, const cv::Mat1b& img,
                     const cv::Mat1f& idepths, const cv::Mat1f& depth,
                     cv::Mat3b* debug_img_idepth_error) {
    // Compute confusion matrix.
    cv::Mat1f idepth_error;
    float total_error;
    int true_pos, false_pos, true_neg, false_neg;
    getDepthConfusionMatrix(idepths, depth, &idepth_error, &total_error,
                            &true_pos, &true_neg, &false_pos, &false_neg);

    // Compute some stats.
    float precision = static_cast<float>(true_pos) / (true_pos + false_pos);
    float recall = static_cast<float>(true_pos) / (true_pos + false_neg);

    float avg_error_per_pixel = total_error / (true_pos + false_pos);

    // Apply colomap.
    auto colormap = [this](float v, cv::Vec3b c) {
      return std::isnan(v) ? c : flame::utils::jet(v, 0.0f, 0.35f);
    };
    cv::cvtColor(img, *debug_img_idepth_error, cv::COLOR_GRAY2RGB);
    flame::utils::applyColorMap<float>(idepth_error, colormap,
                                       debug_img_idepth_error);

    // Print some info.
    char buf[200];
    snprintf(buf, sizeof(buf), "TotalError: %.3f, AvgError: %.3f, Prec: %.3f, Rec: %.3f",
             total_error, avg_error_per_pixel, precision, recall);
    cv::putText(*debug_img_idepth_error, buf,
                cv::Point(10, debug_img_idepth_error->rows - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 250), 1, 8);

    std::string stats_fname(output_dir_ + "/stats.txt");
    FILE* fid = fopen(stats_fname.c_str(), "a");
    if (fid == nullptr) {
      ROS_ERROR("Could not open statistics file!\n");
      return;
    }

    if (num_imgs_ == 0) {
      fprintf(fid, "idx ");
      fprintf(fid, "timestamp ");
      fprintf(fid, "runtime ");
      fprintf(fid, "num_vtx ");
      fprintf(fid, "num_tris ");
      fprintf(fid, "true_pos ");
      fprintf(fid, "true_neg ");
      fprintf(fid, "false_pos ");
      fprintf(fid, "false_neg ");
      fprintf(fid, "total_idepth_error ");
      fprintf(fid, "avg_idepth_error ");
      fprintf(fid, "precision ");
      fprintf(fid, "recall ");
      fprintf(fid, "total_photo_error ");
      fprintf(fid, "avg_photo_error");
      fprintf(fid, "\n");
    }

    fprintf(fid, "%i ", num_imgs_);
    fprintf(fid, "%.6f ", timestamp);
    fprintf(fid, "%.6f ", sensor_->stats().timings("sense"));
    fprintf(fid, "%i ", static_cast<int>(sensor_->stats().stats("num_vtx")));
    fprintf(fid, "%i ", static_cast<int>(sensor_->stats().stats("num_tris")));
    fprintf(fid, "%i ", true_pos);
    fprintf(fid, "%i ", true_neg);
    fprintf(fid, "%i ", false_pos);
    fprintf(fid, "%i ", false_neg);
    fprintf(fid, "%.6f ", total_error);
    fprintf(fid, "%.6f ", avg_error_per_pixel);
    fprintf(fid, "%.6f ", precision);
    fprintf(fid, "%.6f ", recall);
    fprintf(fid, "%.6f ", sensor_->stats().stats("total_photo_error"));
    fprintf(fid, "%.6f", sensor_->stats().stats("avg_photo_error"));
    fprintf(fid, "\n");

    fclose(fid);

    return;
  }

  /**
   * @brief Main processing loop.
   */
  void main() {
    Kinv_ = input_->K().inverse();

    // Initialize depth sensor.
    ROS_INFO_COND(!params_.debug_quiet, "Constructing Flame...\n");
    sensor_ = std::make_shared<flame::Flame>(input_->width(),
                                             input_->height(),
                                             input_->K(),
                                             Kinv_,
                                             params_);

    /*==================== Enter main loop ====================*/
    ros::Rate ros_rate(rate_);
    ROS_INFO_COND(!params_.debug_quiet, "Done. We are GO for launch!\n");
    while (ros::ok() && !input_->empty()) {
      stats_.tick("main");

      // Get input data.
      stats_.set("queue_size", 0);
      stats_.tick("waiting");
      uint32_t img_id;
      double time;
      cv::Mat3b rgb;
      cv::Mat1f depth;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      input_->get(&img_id, &time, &rgb, &depth, &q, &t);
      stats_.tock("waiting");
      ROS_INFO_COND(!params_.debug_quiet,
                    "FlameNodelet/waiting = %4.1fms, queue_size = %i\n",
                    stats_.timings("waiting"),
                    static_cast<int>(stats_.stats("queue_size")));

      if (num_imgs_ % subsample_factor_ == 0) {
        // Eat data.
        processFrame(img_id, time, Sophus::SE3f(q.cast<float>(), t.cast<float>()),
                     rgb, depth);
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

      ros::spinOnce();
      ros_rate.sleep();

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

      publishFlameNodeletStats(nodelet_stats_pub_, num_imgs_, time,
                               stats_.stats(), stats_.timings());

      ROS_INFO_COND(!params_.debug_quiet,
                    "FlameOffline/main(%i/%u) = %4.1fms/%.1fHz (%.1fHz)\n",
                    num_imgs_, img_id, stats_.timings("main"),
                    stats_.stats("fps_max"), stats_.stats("fps"));

      num_imgs_++;
    }

    if (input_->empty()) {
      ROS_INFO("Finished processing.\n");
    } else {
      ROS_ERROR("Unknown error occurred!\n");
    }

    return;
  }

  void processFrame(const uint32_t img_id, const double time,
                    const Sophus::SE3f& pose, const cv::Mat3b& rgb,
                    const cv::Mat1f& depth) {
    stats_.tick("process_frame");

    /*==================== Process image ====================*/
    // Convert to grayscale.
    cv::Mat1b img_gray;
    cv::cvtColor(rgb, img_gray, cv::COLOR_RGB2GRAY);

    bool is_poseframe = (img_id % poseframe_subsample_factor_) == 0;
    bool update_success = false;
    if (!pass_in_truth_) {
      update_success = sensor_->update(time, img_id, pose, img_gray,
                                       is_poseframe);
    } else {
      // Convert true depth to idepth.
      cv::Mat1f idepths_true(rgb.rows, rgb.cols, std::numeric_limits<float>::quiet_NaN());
      for (int ii = 0; ii < depth.rows; ++ii) {
        for (int jj = 0; jj < depth.cols; ++jj) {
          if (!std::isnan(depth(ii, jj)) && (depth(ii, jj) > 0)) {
            idepths_true(ii, jj) = 1.0f/depth(ii, jj);
          } else {
            idepths_true(ii, jj) = std::numeric_limits<float>::quiet_NaN();
          }
        }
      }

      update_success = sensor_->update(time, img_id, pose, img_gray, is_poseframe,
                                       idepths_true);
    }

    if (!update_success) {
      stats_.tock("process_frame");
      ROS_WARN("FlameOffline: Unsuccessful update.\n");
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

    cv::Mat1f idepthmap;
    if (publish_idepthmap_ || publish_depthmap_ || publish_cloud_) {
      cv::Mat1f idepthmap;
      sensor_->getFilteredInverseDepthMap(&idepthmap);

      if (publish_idepthmap_) {
        publishDepthMap(idepth_pub_, camera_frame_id_, time, input_->K(),
                        idepthmap);
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
        publishDepthMap(depth_pub_, camera_frame_id_, time, input_->K(),
                        depth_est);
      }

      if (publish_cloud_) {
        float max_depth = (params_.do_idepth_triangle_filter) ?
            1.0f / params_.min_triangle_idepth : std::numeric_limits<float>::max();
        publishPointCloud(cloud_pub_, camera_frame_id_, time, input_->K(),
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

      publishDepthMap(features_pub_, camera_frame_id_, time, input_->K(),
                      depth_raw);
    }

    if (publish_stats_) {
      auto stats = sensor_->stats().stats();
      auto timings = sensor_->stats().timings();
      publishFlameStats(stats_pub_, img_id, time, stats, timings);
    }

    stats_.set("latency", (ros::Time::now().toSec() - time) * 1000);
    ROS_INFO_COND(!params_.debug_quiet,
                  "FlameNodelet/latency = %4.1fms\n",
                  stats_.stats("latency"));

    stats_.tock("publishing");
    ROS_INFO_COND(!params_.debug_quiet,
                  "FlameOffline/publishing = %4.1fms\n",
                  stats_.timings("publishing"));

    /*==================== Publish debug stuff ====================*/
    stats_.tick("debug_publishing");

    std_msgs::Header hdr;
    hdr.stamp.fromSec(time);
    hdr.frame_id = camera_frame_id_;

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
    ROS_INFO_COND(!params_.debug_quiet,
                  "FlameOffline/debug_publishing = %4.1fms\n",
                  stats_.timings("debug_publishing"));

    stats_.tock("process_frame");

    ROS_INFO_COND(!params_.debug_quiet,
                  "FlameOffline/process_frame = %4.1fms\n",
                  stats_.timings("process_frame"));

    return;
  }

 private:
  // Keeps track of stats and load.
  fu::StatsTracker stats_;
  fu::LoadTracker load_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Number of images processed.
  int num_imgs_;

  // Target processing rate. Can artificially slow down processing so that ROS can
  // keep up.
  float rate_;

  // Frame ID of the camera in frame camera_world_frame_id.
  std::string camera_frame_id_;

  // Frame id of the world in camera (Right-Down-Forward) coordinates.
  std::string camera_world_frame_id_;

  int subsample_factor_; // Process one out of this many images.
  int poseframe_subsample_factor_; // Create a poseframe every this number of images.
  int resize_factor_;

  // Save truth stats.
  std::string output_dir_;
  bool pass_in_truth_; // Pass truth into processing.

  // Input stream object.
  std::shared_ptr<ros_sensor_streams::ASLRGBDOfflineStream> input_;
  Eigen::Matrix3f Kinv_;

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

  // Publishes depthmap and stuff.
  image_transport::ImageTransport it_;
  bool publish_idepthmap_;
  bool publish_depthmap_;
  bool publish_features_;
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
};

}  // namespace flame_ros

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "flame");

  // Install signal handlers.
  std::signal(SIGINT, crash_handler);
  std::signal(SIGSEGV, crash_handler);

  flame_ros::FlameOffline node;
  node.main();

  return 0;
}
