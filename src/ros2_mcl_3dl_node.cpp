/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mcl_3dl/mcl_3dl_engine.h"
#include "mcl_3dl/parameters.h"
#include "mcl_3dl_msgs/msg/status.hpp"
#include "mcl_3dl_msgs/srv/load_pcd.hpp"
#include "mcl_3dl_msgs/srv/resize_particle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mcl_3dl
{
class MCL3dlNode : public rclcpp::Node
{
public:
  explicit MCL3dlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mcl_3dl", options),
    tfbuf_(this->get_clock()),
    tfl_(tfbuf_),
    tfb_(*this),
    engine_(tfbuf_, this->get_logger()),
    diag_updater_(this)
  {
    if (!loadParameters()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load parameters");
      throw std::runtime_error("Failed to load parameters");
    }

    if (!engine_.configure(params_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure engine");
      throw std::runtime_error("Failed to configure engine");
    }

    setupEngineCallbacks();
    setupSubscribers();
    setupPublishers();
    setupServices();
    setupTimers();
    setupDiagnostics();
  }

private:
  bool loadParameters()
  {
    params_.fake_imu_ = this->declare_parameter("fake_imu", false);
    params_.fake_odom_ = this->declare_parameter("fake_odom", false);
    if (params_.fake_imu_ && params_.fake_odom_) {
      RCLCPP_ERROR(this->get_logger(), "One of IMU and Odometry must be enabled");
      return false;
    }

    params_.frame_ids_["map"] = this->declare_parameter("map_frame", std::string("map"));
    params_.frame_ids_["base_link"] =
      this->declare_parameter("robot_frame", std::string("base_link"));
    params_.frame_ids_["odom"] = this->declare_parameter("odom_frame", std::string("odom"));
    params_.frame_ids_["floor"] = this->declare_parameter("floor_frame", std::string("floor"));

    params_.map_downsample_x_ = this->declare_parameter("map_downsample_x", 0.1);
    params_.map_downsample_y_ = this->declare_parameter("map_downsample_y", 0.1);
    params_.map_downsample_z_ = this->declare_parameter("map_downsample_z", 0.1);
    params_.downsample_x_ = this->declare_parameter("downsample_x", 0.1);
    params_.downsample_y_ = this->declare_parameter("downsample_y", 0.1);
    params_.downsample_z_ = this->declare_parameter("downsample_z", 0.05);
    params_.map_grid_min_ =
      std::min({params_.map_downsample_x_, params_.map_downsample_y_, params_.map_downsample_z_});
    params_.map_grid_max_ =
      std::max({params_.map_downsample_x_, params_.map_downsample_y_, params_.map_downsample_z_});

    params_.update_downsample_x_ = this->declare_parameter("update_downsample_x", 0.3);
    params_.update_downsample_y_ = this->declare_parameter("update_downsample_y", 0.3);
    params_.update_downsample_z_ = this->declare_parameter("update_downsample_z", 0.3);

    const double map_update_interval_t =
      this->declare_parameter("map_update_interval_interval", 2.0);
    params_.map_update_interval_ =
      std::make_shared<rclcpp::Duration>(rclcpp::Duration::from_seconds(map_update_interval_t));

    params_.dist_weight_[0] = static_cast<float>(this->declare_parameter("dist_weight_x", 1.0));
    params_.dist_weight_[1] = static_cast<float>(this->declare_parameter("dist_weight_y", 1.0));
    params_.dist_weight_[2] = static_cast<float>(this->declare_parameter("dist_weight_z", 5.0));
    params_.dist_weight_[3] = 0.0f;

    params_.global_localization_grid_ =
      this->declare_parameter("global_localization_grid_lin", 0.3);
    const double grid_ang = this->declare_parameter("global_localization_grid_ang", 0.524);
    params_.global_localization_div_yaw_ = std::lround(2 * M_PI / grid_ang);

    params_.num_particles_ = this->declare_parameter("num_particles", 64);

    params_.resample_var_x_ = this->declare_parameter("resample_var_x", 0.05);
    params_.resample_var_y_ = this->declare_parameter("resample_var_y", 0.05);
    params_.resample_var_z_ = this->declare_parameter("resample_var_z", 0.05);
    params_.resample_var_roll_ = this->declare_parameter("resample_var_roll", 0.05);
    params_.resample_var_pitch_ = this->declare_parameter("resample_var_pitch", 0.05);
    params_.resample_var_yaw_ = this->declare_parameter("resample_var_yaw", 0.05);
    params_.expansion_var_x_ = this->declare_parameter("expansion_var_x", 0.2);
    params_.expansion_var_y_ = this->declare_parameter("expansion_var_y", 0.2);
    params_.expansion_var_z_ = this->declare_parameter("expansion_var_z", 0.2);
    params_.expansion_var_roll_ = this->declare_parameter("expansion_var_roll", 0.05);
    params_.expansion_var_pitch_ = this->declare_parameter("expansion_var_pitch", 0.05);
    params_.expansion_var_yaw_ = this->declare_parameter("expansion_var_yaw", 0.05);
    params_.match_ratio_thresh_ = this->declare_parameter("match_ratio_thresh", 0.0);

    params_.odom_err_lin_lin_ = this->declare_parameter("odom_err_lin_lin", 0.10);
    params_.odom_err_lin_ang_ = this->declare_parameter("odom_err_lin_ang", 0.05);
    params_.odom_err_ang_lin_ = this->declare_parameter("odom_err_ang_lin", 0.05);
    params_.odom_err_ang_ang_ = this->declare_parameter("odom_err_ang_ang", 0.05);

    params_.odom_err_integ_lin_tc_ = this->declare_parameter("odom_err_integ_lin_tc", 10.0);
    params_.odom_err_integ_lin_sigma_ = this->declare_parameter("odom_err_integ_lin_sigma", 100.0);
    params_.odom_err_integ_ang_tc_ = this->declare_parameter("odom_err_integ_ang_tc", 10.0);
    params_.odom_err_integ_ang_sigma_ = this->declare_parameter("odom_err_integ_ang_sigma", 100.0);

    params_.lpf_step_ = this->declare_parameter("lpf_step", 16.0);
    params_.acc_lpf_step_ = this->declare_parameter("acc_lpf_step", 128.0);

    params_.acc_var_ = this->declare_parameter("acc_var", M_PI / 4.0);

    params_.jump_dist_ = this->declare_parameter("jump_dist", 1.0);
    params_.jump_ang_ = this->declare_parameter("jump_ang", 1.57);
    params_.fix_dist_ = this->declare_parameter("fix_dist", 0.2);
    params_.fix_ang_ = this->declare_parameter("fix_ang", 0.1);
    params_.bias_var_dist_ = this->declare_parameter("bias_var_dist", 2.0);
    params_.bias_var_ang_ = this->declare_parameter("bias_var_ang", 1.57);

    params_.skip_measure_ = this->declare_parameter("skip_measure", 1);
    params_.accum_cloud_ = this->declare_parameter("accum_cloud", 1);
    params_.total_accum_cloud_max_ =
      this->declare_parameter("total_accum_cloud_max", params_.accum_cloud_ * 10);

    const double match_output_interval_t =
      this->declare_parameter("match_output_interval_interval", 0.2);
    params_.match_output_interval_ =
      std::make_shared<rclcpp::Duration>(rclcpp::Duration::from_seconds(match_output_interval_t));

    const double tf_tolerance_t = this->declare_parameter("tf_tolerance", 0.05);
    params_.tf_tolerance_ =
      std::make_shared<rclcpp::Duration>(rclcpp::Duration::from_seconds(tf_tolerance_t));

    params_.match_output_dist_ = this->declare_parameter("match_output_dist", 0.1);
    params_.unmatch_output_dist_ = this->declare_parameter("unmatch_output_dist", 0.5);

    params_.publish_tf_ = this->declare_parameter("publish_tf", true);
    params_.output_pcd_ = this->declare_parameter("output_pcd", false);

    const float float_max = std::numeric_limits<float>::max();
    params_.std_warn_thresh_[0] = static_cast<float>(
      this->declare_parameter("std_warn_thresh_xy", static_cast<double>(float_max)));
    params_.std_warn_thresh_[1] = static_cast<float>(
      this->declare_parameter("std_warn_thresh_z", static_cast<double>(float_max)));
    params_.std_warn_thresh_[2] = static_cast<float>(
      this->declare_parameter("std_warn_thresh_yaw", static_cast<double>(float_max)));

    params_.map_chunk_ = this->declare_parameter("map_chunk", 20.0);

    const double x = this->declare_parameter("init_x", 0.0);
    const double y = this->declare_parameter("init_y", 0.0);
    const double z = this->declare_parameter("init_z", 0.0);
    const double roll = this->declare_parameter("init_roll", 0.0);
    const double pitch = this->declare_parameter("init_pitch", 0.0);
    const double yaw = this->declare_parameter("init_yaw", 0.0);
    const double v_x = this->declare_parameter("init_var_x", 2.0);
    const double v_y = this->declare_parameter("init_var_y", 2.0);
    const double v_z = this->declare_parameter("init_var_z", 0.5);
    const double v_roll = this->declare_parameter("init_var_roll", 0.1);
    const double v_pitch = this->declare_parameter("init_var_pitch", 0.1);
    const double v_yaw = this->declare_parameter("init_var_yaw", 0.5);
    params_.initial_pose_ = State6DOF(Vec3(x, y, z), Quat(Vec3(roll, pitch, yaw)));
    params_.initial_pose_std_ = State6DOF(Vec3(v_x, v_y, v_z), Vec3(v_roll, v_pitch, v_yaw));

    params_.use_random_sampler_with_normal_ =
      this->declare_parameter("use_random_sampler_with_normal", false);

    if (params_.use_random_sampler_with_normal_) {
      params_.random_sampler_with_normal_params_->perform_weighting_ratio_ =
        this->declare_parameter("random_sampler_with_normal.perform_weighting_ratio", 2.0);
      params_.random_sampler_with_normal_params_->max_weight_ratio_ =
        this->declare_parameter("random_sampler_with_normal.max_weight_ratio", 5.0);
      params_.random_sampler_with_normal_params_->max_weight_ =
        this->declare_parameter("random_sampler_with_normal.max_weight", 5.0);
      params_.random_sampler_with_normal_params_->normal_search_range_ =
        this->declare_parameter("random_sampler_with_normal.normal_search_range", 0.4);
    }

    // Likelihood model parameters
    {
      int num_points = this->declare_parameter("likelihood.num_points", 96);
      int num_points_global = this->declare_parameter("likelihood.num_points_global", 8);
      params_.lidar_measurement_likelihood_params_->num_points_default_ = num_points;
      params_.lidar_measurement_likelihood_params_->num_points_global_ = num_points_global;

      const double clip_near = this->declare_parameter("likelihood.clip_near", 0.5);
      const double clip_far = this->declare_parameter("likelihood.clip_far", 10.0);
      params_.lidar_measurement_likelihood_params_->clip_near_ = clip_near;
      params_.lidar_measurement_likelihood_params_->clip_far_ = clip_far;

      const double clip_z_min = this->declare_parameter("likelihood.clip_z_min", -2.0);
      const double clip_z_max = this->declare_parameter("likelihood.clip_z_max", 2.0);
      params_.lidar_measurement_likelihood_params_->clip_z_min_ = clip_z_min;
      params_.lidar_measurement_likelihood_params_->clip_z_max_ = clip_z_max;

      const double match_weight = this->declare_parameter("likelihood.match_weight", 5.0);
      params_.lidar_measurement_likelihood_params_->match_weight_ = match_weight;

      const double match_dist_min = this->declare_parameter("likelihood.match_dist_min", 0.2);
      const double match_dist_flat = this->declare_parameter("likelihood.match_dist_flat", 0.05);
      params_.lidar_measurement_likelihood_params_->match_dist_min_ = match_dist_min;
      params_.lidar_measurement_likelihood_params_->match_dist_flat_ = match_dist_flat;
    }

    // Beam model parameters
    {
      params_.lidar_measurement_beam_params_->map_grid_x_ = params_.map_downsample_x_;
      params_.lidar_measurement_beam_params_->map_grid_y_ = params_.map_downsample_y_;
      params_.lidar_measurement_beam_params_->map_grid_z_ = params_.map_downsample_z_;

      int num_points = this->declare_parameter("beam.num_points", 3);
      int num_points_global = this->declare_parameter("beam.num_points_global", 0);
      params_.lidar_measurement_beam_params_->num_points_default_ = num_points;
      params_.lidar_measurement_beam_params_->num_points_global_ = num_points_global;

      const double clip_near = this->declare_parameter("beam.clip_near", 0.5);
      const double clip_far = this->declare_parameter("beam.clip_far", 4.0);
      params_.lidar_measurement_beam_params_->clip_near_ = clip_near;
      params_.lidar_measurement_beam_params_->clip_far_ = clip_far;

      const double clip_z_min = this->declare_parameter("beam.clip_z_min", -2.0);
      const double clip_z_max = this->declare_parameter("beam.clip_z_max", 2.0);
      params_.lidar_measurement_beam_params_->clip_z_min_ = clip_z_min;
      params_.lidar_measurement_beam_params_->clip_z_max_ = clip_z_max;

      const double beam_likelihood = this->declare_parameter("beam.beam_likelihood", 0.2);
      params_.lidar_measurement_beam_params_->beam_likelihood_min_ = beam_likelihood;

      const double ang_total_ref = this->declare_parameter("beam.ang_total_ref", M_PI / 6.0);
      params_.lidar_measurement_beam_params_->ang_total_ref_ = ang_total_ref;

      int filter_label_max =
        this->declare_parameter("beam.filter_label_max", static_cast<int>(0x7FFFFFFF));
      params_.lidar_measurement_beam_params_->filter_label_max_ =
        static_cast<uint32_t>(filter_label_max);

      params_.lidar_measurement_beam_params_->add_penalty_short_only_mode_ =
        this->declare_parameter("beam.add_penalty_short_only_mode", true);
      const double hit_range = this->declare_parameter("beam.hit_range", 0.3);
      params_.lidar_measurement_beam_params_->hit_range_ = hit_range;

      params_.lidar_measurement_beam_params_->use_raycast_using_dda_ =
        this->declare_parameter("beam.use_raycast_using_dda", false);
      if (params_.lidar_measurement_beam_params_->use_raycast_using_dda_) {
        const double ray_angle_half =
          this->declare_parameter("beam.ray_angle_half", 0.25 * M_PI / 180.0);
        params_.lidar_measurement_beam_params_->ray_angle_half_ = ray_angle_half;

        double dda_grid_size = this->declare_parameter("beam.dda_grid_size", 0.2);
        const double grid_size_max = std::max({
          static_cast<double>(params_.lidar_measurement_beam_params_->map_grid_x_),
          static_cast<double>(params_.lidar_measurement_beam_params_->map_grid_y_),
          static_cast<double>(params_.lidar_measurement_beam_params_->map_grid_z_),
        });
        if (dda_grid_size < grid_size_max) {
          RCLCPP_WARN(
            this->get_logger(), "dda_grid_size must be larger than grid size. New value: %f",
            grid_size_max);
          dda_grid_size = grid_size_max;
        }
        params_.lidar_measurement_beam_params_->dda_grid_size_ = dda_grid_size;
      }
    }

    // Set up dynamic parameter callback for std_warn_thresh
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MCL3dlNode::onParameterChange, this, std::placeholders::_1));

    return true;
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "std_warn_thresh_xy")
        params_.std_warn_thresh_[0] = static_cast<float>(param.as_double());
      else if (param.get_name() == "std_warn_thresh_z")
        params_.std_warn_thresh_[1] = static_cast<float>(param.as_double());
      else if (param.get_name() == "std_warn_thresh_yaw")
        params_.std_warn_thresh_[2] = static_cast<float>(param.as_double());
    }
    return result;
  }

  void setupEngineCallbacks()
  {
    using std::placeholders::_1;
    engine_.setPublishParticlesCallback([this](const geometry_msgs::msg::PoseArray & pa) {
      auto msg = std::make_unique<geometry_msgs::msg::PoseArray>(pa);
      pub_particle_->publish(std::move(msg));
    });
    engine_.setPublishDebugMarkerCallback(
      [this](const visualization_msgs::msg::MarkerArray & markers) {
        auto msg = std::make_unique<visualization_msgs::msg::MarkerArray>(markers);
        pub_debug_marker_->publish(std::move(msg));
      });
    engine_.setPublishMatchedCallback([this](const sensor_msgs::msg::PointCloud2 & pc) {
      auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>(pc);
      pub_matched_->publish(std::move(msg));
    });
    engine_.setPublishUnmatchedCallback([this](const sensor_msgs::msg::PointCloud2 & pc) {
      auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>(pc);
      pub_unmatched_->publish(std::move(msg));
    });
    engine_.setPublishPoseCallback(
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & pose) {
        auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>(pose);
        pub_pose_->publish(std::move(msg));
      });
    engine_.setPublishStatusCallback([this](const mcl_3dl_msgs::msg::Status & status) {
      auto msg = std::make_unique<mcl_3dl_msgs::msg::Status>(status);
      pub_status_->publish(std::move(msg));
    });
    engine_.setPublishMapCloudCallback([this](const sensor_msgs::msg::PointCloud2 & pc) {
      auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>(pc);
      pub_mapcloud_->publish(std::move(msg));
    });
    engine_.setBroadcastTransformCallback(
      [this](const std::vector<geometry_msgs::msg::TransformStamped> & transforms) {
        tfb_.sendTransform(transforms);
      });
    engine_.setDiagUpdateCallback([this]() { diag_updater_.force_update(); });
    engine_.setCheckSubscribersCallback([this](const std::string & topic) -> bool {
      if (topic == "matched") return pub_matched_->get_subscription_count() > 0;
      if (topic == "unmatched") return pub_unmatched_->get_subscription_count() > 0;
      return false;
    });
  }

  void setupSubscribers()
  {
    if (!params_.fake_odom_) {
      const int odom_queue_size = this->declare_parameter("odom_queue_size", 200);
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", odom_queue_size,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr & msg) { engine_.processOdom(msg); });
    }
    if (!params_.fake_imu_) {
      const int imu_queue_size = this->declare_parameter("imu_queue_size", 200);
      sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", imu_queue_size,
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr & msg) { engine_.processImu(msg); });
    }

    const int cloud_queue_size = this->declare_parameter("cloud_queue_size", 100);
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", cloud_queue_size, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        engine_.processCloud(msg);
      });
    sub_mapcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "mapcloud", 1, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        engine_.processMapCloud(msg);
      });
    sub_mapcloud_update_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "mapcloud_update", 1, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        engine_.processMapCloudUpdate(msg);
      });
    sub_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 1,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg) {
        engine_.processPosition(msg);
      });
    sub_landmark_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "mcl_measurement", 1,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg) {
        engine_.processLandmark(msg);
      });
  }

  void setupPublishers()
  {
    pub_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 5);

    auto latched_qos = rclcpp::QoS(1).transient_local();
    pub_particle_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("~/particles", latched_qos);
    pub_mapcloud_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("~/updated_map", latched_qos);
    pub_debug_marker_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug_marker", latched_qos);
    pub_status_ = this->create_publisher<mcl_3dl_msgs::msg::Status>("~/status", latched_qos);
    pub_matched_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/matched", rclcpp::QoS(2).transient_local());
    pub_unmatched_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/unmatched", rclcpp::QoS(2).transient_local());
  }

  void setupServices()
  {
    srv_particle_size_ = this->create_service<mcl_3dl_msgs::srv::ResizeParticle>(
      "~/resize_particle", [this](
                             const mcl_3dl_msgs::srv::ResizeParticle::Request::SharedPtr request,
                             mcl_3dl_msgs::srv::ResizeParticle::Response::SharedPtr /*response*/) {
        engine_.resizeParticle(request->size);
      });
    srv_global_localization_ = this->create_service<std_srvs::srv::Trigger>(
      "~/global_localization", [this](
                                 const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                                 std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::string message;
        response->success = engine_.globalLocalization(message);
        response->message = message;
      });
    srv_expansion_reset_ = this->create_service<std_srvs::srv::Trigger>(
      "~/expansion_resetting",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
        std_srvs::srv::Trigger::Response::SharedPtr /*response*/) { engine_.expansionReset(); });
    srv_load_pcd_ = this->create_service<mcl_3dl_msgs::srv::LoadPCD>(
      "load_pcd", [this](
                    const mcl_3dl_msgs::srv::LoadPCD::Request::SharedPtr request,
                    mcl_3dl_msgs::srv::LoadPCD::Response::SharedPtr response) {
        response->success = engine_.loadPCD(request->pcd_path);
      });
  }

  void setupTimers()
  {
    map_update_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
        params_.map_update_interval_->seconds() +
        params_.map_update_interval_->nanoseconds() * 1e-9)),
      [this]() { engine_.mapUpdateTimer(); });
  }

  void setupDiagnostics()
  {
    diag_updater_.setHardwareID("none");
    diag_updater_.add("Status", this, &MCL3dlNode::diagnoseStatus);
  }

  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    bool has_error, has_warn;
    std::string message;
    bool has_map, has_odom, has_imu;
    engine_.diagnoseStatus(has_error, has_warn, message, has_map, has_odom, has_imu);

    if (has_error) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, message);
    }

    stat.add("Map Availability", has_map ? "true" : "false");
    stat.add("Odometry Availability", has_odom ? "true" : "false");
    stat.add("IMU Availability", has_imu ? "true" : "false");

    mcl_3dl_msgs::msg::Status status = engine_.getStatus();
    status.entropy = engine_.getEntropy();
    auto msg = std::make_unique<mcl_3dl_msgs::msg::Status>(status);
    pub_status_->publish(std::move(msg));
  }

  // TF
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  tf2_ros::TransformBroadcaster tfb_;

  // Engine
  Parameters params_;
  MCL3dlEngine engine_;

  // Diagnostics
  diagnostic_updater::Updater diag_updater_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_mapcloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_mapcloud_update_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_position_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_landmark_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particle_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_mapcloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<mcl_3dl_msgs::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_matched_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unmatched_;

  // Services
  rclcpp::Service<mcl_3dl_msgs::srv::ResizeParticle>::SharedPtr srv_particle_size_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_global_localization_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_expansion_reset_;
  rclcpp::Service<mcl_3dl_msgs::srv::LoadPCD>::SharedPtr srv_load_pcd_;

  // Timer
  rclcpp::TimerBase::SharedPtr map_update_timer_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
}  // namespace mcl_3dl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mcl_3dl::MCL3dlNode)
