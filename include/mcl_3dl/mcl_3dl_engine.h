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

#ifndef MCL_3DL_MCL_3DL_ENGINE_H
#define MCL_3DL_MCL_3DL_ENGINE_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mcl_3dl_msgs/msg/status.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#ifdef IS_ROS1_BUILD
#include <tf2_ros/transform_listener.h>
#else
#include <tf2_ros/buffer.hpp>
#endif

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/cloud_accum.h>
#include <mcl_3dl/filter.h>
#include <mcl_3dl/filter_vec3.h>
#include <mcl_3dl/imu_measurement_model_base.h>
#include <mcl_3dl/imu_measurement_models/imu_measurement_model_gravity.h>
#include <mcl_3dl/lidar_measurement_model_base.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_beam.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>
#include <mcl_3dl/motion_prediction_model_base.h>
#include <mcl_3dl/motion_prediction_models/motion_prediction_model_differential_drive.h>
#include <mcl_3dl/nd.h>
#include <mcl_3dl/noise_generators/multivariate_noise_generator.h>
#include <mcl_3dl/parameters.h>
#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_samplers/point_cloud_sampler_with_normal.h>
#include <mcl_3dl/point_cloud_random_samplers/point_cloud_uniform_sampler.h>
#include <mcl_3dl/point_conversion.h>
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

namespace mcl_3dl
{
/// Result of the measure() cycle, containing data for the node to publish.
struct MeasureResult
{
  bool valid;

  // Pose estimate
  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  // TF transforms to broadcast
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // Particle cloud for visualization
  geometry_msgs::msg::PoseArray particles;

  // Status
  mcl_3dl_msgs::msg::Status status;

  // Debug marker (beam model visualization)
  visualization_msgs::msg::MarkerArray debug_markers;
  bool has_debug_markers;

  // Matched/unmatched point clouds for visualization
  sensor_msgs::msg::PointCloud2 matched_cloud;
  bool has_matched_cloud;
  sensor_msgs::msg::PointCloud2 unmatched_cloud;
  bool has_unmatched_cloud;

  MeasureResult()
    : valid(false)
    , has_debug_markers(false)
    , has_matched_cloud(false)
    , has_unmatched_cloud(false)
  {
  }
};

/// Result of map update timer
struct MapUpdateResult
{
  bool valid;
  sensor_msgs::msg::PointCloud2 map_cloud;

  MapUpdateResult()
    : valid(false)
  {
  }
};

/// Core MCL engine, separated from ROS interface.
/// Uses tf2_ros::Buffer for coordinate transforms, rclcpp::Time/Duration for time,
/// and ROS message types for data exchange. Does NOT create publishers, subscribers,
/// services, or node handles.
class MCL3dlEngine
{
public:
  using PointType = mcl_3dl::PointXYZIL;

  MCL3dlEngine(tf2_ros::Buffer& tfbuf, const rclcpp::Logger& logger);
  ~MCL3dlEngine();

  /// Initialize the engine with loaded parameters.
  /// The Parameters object must outlive the engine.
  bool configure(Parameters& params);

  /// Process incoming map pointcloud
  void processMapCloud(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg);

  /// Process incoming map update pointcloud
  void processMapCloudUpdate(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg);

  /// Process incoming initial pose
  void processPosition(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg);

  /// Process incoming odometry
  void processOdom(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg);

  /// Process incoming pointcloud
  void processCloud(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg);

  /// Process incoming IMU data
  void processImu(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg);

  /// Process incoming landmark measurement
  void processLandmark(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg);

  /// Handle resize particle service
  bool resizeParticle(int size);

  /// Handle expansion reset service
  bool expansionReset();

  /// Handle global localization service
  bool globalLocalization(std::string& message);

  /// Handle load PCD service
  bool loadPCD(const std::string& pcd_path);

  /// Timer callback for map updates. Returns data to publish.
  MapUpdateResult mapUpdateTimer();

  /// Get current particles as PoseArray
  geometry_msgs::msg::PoseArray getParticles() const;

  /// Get current status
  mcl_3dl_msgs::msg::Status getStatus() const;

  /// Get diagnostic information
  void diagnoseStatus(bool& has_error, bool& has_warn,
                      std::string& message,
                      bool& has_map, bool& has_odom, bool& has_imu) const;

  /// Get the entropy from the particle filter
  float getEntropy() const;

  /// Access to parameters (read-only)
  const Parameters& params() const
  {
    return *params_;
  }

  /// Check if map has been received
  bool hasMap() const
  {
    return has_map_;
  }
  bool hasOdom() const
  {
    return has_odom_;
  }
  bool hasImu() const
  {
    return has_imu_;
  }

  /// Set callbacks for publish operations that must happen during processing
  using PublishParticlesCallback = std::function<void(const geometry_msgs::msg::PoseArray&)>;
  using PublishDebugMarkerCallback = std::function<void(const visualization_msgs::msg::MarkerArray&)>;
  using PublishMatchedCallback = std::function<void(const sensor_msgs::msg::PointCloud2&)>;
  using PublishUnmatchedCallback = std::function<void(const sensor_msgs::msg::PointCloud2&)>;
  using PublishPoseCallback = std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped&)>;
  using PublishStatusCallback = std::function<void(const mcl_3dl_msgs::msg::Status&)>;
  using PublishMapCloudCallback = std::function<void(const sensor_msgs::msg::PointCloud2&)>;
  using BroadcastTransformCallback = std::function<void(const std::vector<geometry_msgs::msg::TransformStamped>&)>;
  using DiagUpdateCallback = std::function<void()>;
  using CheckSubscribersCallback = std::function<bool(const std::string& topic)>;

  void setPublishParticlesCallback(PublishParticlesCallback cb)
  {
    publish_particles_cb_ = cb;
  }
  void setPublishDebugMarkerCallback(PublishDebugMarkerCallback cb)
  {
    publish_debug_marker_cb_ = cb;
  }
  void setPublishMatchedCallback(PublishMatchedCallback cb)
  {
    publish_matched_cb_ = cb;
  }
  void setPublishUnmatchedCallback(PublishUnmatchedCallback cb)
  {
    publish_unmatched_cb_ = cb;
  }
  void setPublishPoseCallback(PublishPoseCallback cb)
  {
    publish_pose_cb_ = cb;
  }
  void setPublishStatusCallback(PublishStatusCallback cb)
  {
    publish_status_cb_ = cb;
  }
  void setPublishMapCloudCallback(PublishMapCloudCallback cb)
  {
    publish_map_cloud_cb_ = cb;
  }
  void setBroadcastTransformCallback(BroadcastTransformCallback cb)
  {
    broadcast_transform_cb_ = cb;
  }
  void setDiagUpdateCallback(DiagUpdateCallback cb)
  {
    diag_update_cb_ = cb;
  }
  void setCheckSubscribersCallback(CheckSubscribersCallback cb)
  {
    check_subscribers_cb_ = cb;
  }

protected:
  class MyPointRepresentation : public pcl::PointRepresentation<PointType>
  {
    using pcl::PointRepresentation<PointType>::nr_dimensions_;

  public:
    MyPointRepresentation()
    {
      nr_dimensions_ = 3;
      trivial_ = true;
    }

    virtual void copyToFloatArray(const PointType& p, float* out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
    }
  };

  void measure();
  bool accumCloud(const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& msg);
  void accumClear();
  void loadMapCloud(const pcl::PointCloud<PointType>::Ptr& map_cloud);
  void publishParticles();

  tf2_ros::Buffer& tfbuf_;
  rclcpp::Logger logger_;

  Parameters* params_;

  std::shared_ptr<pf::ParticleFilter<State6DOF, float, ParticleWeightedMeanQuat, std::default_random_engine>> pf_;

  std::shared_ptr<FilterVec3> f_pos_;
  std::shared_ptr<FilterVec3> f_ang_;
  std::shared_ptr<FilterVec3> f_acc_;
  std::shared_ptr<Filter> localize_rate_;
  rclcpp::Time localized_last_;
  rclcpp::Duration tf_tolerance_base_;

  rclcpp::Time match_output_last_;
  rclcpp::Time odom_last_;
  bool has_map_;
  bool has_odom_;
  bool has_imu_;
  State6DOF odom_;
  State6DOF odom_prev_;
  State6DOF state_prev_;
  rclcpp::Time imu_last_;
  size_t cnt_measure_;
  Quat imu_quat_;
  size_t global_localization_fix_cnt_;
  mcl_3dl_msgs::msg::Status status_;

  MyPointRepresentation::Ptr point_rep_;

  pcl::PointCloud<PointType>::Ptr pc_map_;
  pcl::PointCloud<PointType>::Ptr pc_map2_;
  pcl::PointCloud<PointType>::Ptr pc_update_;
  pcl::PointCloud<PointType>::Ptr pc_all_accum_;
  ChunkedKdtree<PointType>::Ptr kdtree_;

  CloudAccumulationLogicBase::Ptr accum_;
  pcl::PointCloud<PointType>::Ptr pc_local_accum_;
  std::vector<std_msgs::msg::Header> pc_accum_header_;

  std::map<std::string, LidarMeasurementModelBase::Ptr> lidar_measurements_;
  std::unique_ptr<PointCloudRandomSampler<PointType>> sampler_;
  ImuMeasurementModelBase::Ptr imu_measurement_model_;
  MotionPredictionModelBase::Ptr motion_prediction_model_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;

  // Callbacks for publishing (injected by node)
  PublishParticlesCallback publish_particles_cb_;
  PublishDebugMarkerCallback publish_debug_marker_cb_;
  PublishMatchedCallback publish_matched_cb_;
  PublishUnmatchedCallback publish_unmatched_cb_;
  PublishPoseCallback publish_pose_cb_;
  PublishStatusCallback publish_status_cb_;
  PublishMapCloudCallback publish_map_cloud_cb_;
  BroadcastTransformCallback broadcast_transform_cb_;
  DiagUpdateCallback diag_update_cb_;
  CheckSubscribersCallback check_subscribers_cb_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_MCL_3DL_ENGINE_H
