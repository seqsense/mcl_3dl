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

#include <functional>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mcl_3dl_msgs/msg/status.hpp>
#include <mcl_3dl_msgs/ResizeParticle.h>
#include <mcl_3dl_msgs/LoadPCD.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <mcl_3dl/mcl_3dl_engine.h>
#include <mcl_3dl/parameters.h>

#include <mcl_3dl_compat/compatibility.h>

namespace mcl_3dl
{
class MCL3dlNode
{
public:
  MCL3dlNode()
    : pnh_("~")
    , tfl_(tfbuf_, true, ros::TransportHints().tcpNoDelay(true))
    , engine_(tfbuf_, rclcpp::get_logger("mcl_3dl"))
  {
  }

  bool configure()
  {
    mcl_3dl_compat::checkCompatMode();

    if (!params_.load(pnh_))
    {
      ROS_ERROR("Failed to load parameters");
      return false;
    }

    if (!engine_.configure(params_))
    {
      ROS_ERROR("Failed to configure engine");
      return false;
    }

    // Set up publish callbacks
    setupEngineCallbacks();

    // Set up subscribers
    if (!params_.fake_odom_)
    {
      int odom_queue_size;
      pnh_.param("odom_queue_size", odom_queue_size, 200);
      sub_odom_ = mcl_3dl_compat::subscribe(
          nh_, "odom",
          pnh_, "odom", odom_queue_size, &MCL3dlNode::cbOdom, this,
          ros::TransportHints().tcpNoDelay(true));
    }
    if (!params_.fake_imu_)
    {
      int imu_queue_size;
      pnh_.param("imu_queue_size", imu_queue_size, 200);
      sub_imu_ = mcl_3dl_compat::subscribe(
          nh_, "imu/data",
          pnh_, "imu", imu_queue_size, &MCL3dlNode::cbImu, this,
          ros::TransportHints().tcpNoDelay(true));
    }

    int cloud_queue_size;
    pnh_.param("cloud_queue_size", cloud_queue_size, 100);
    sub_cloud_ = mcl_3dl_compat::subscribe(
        nh_, "cloud",
        pnh_, "cloud", cloud_queue_size, &MCL3dlNode::cbCloud, this);
    sub_mapcloud_ = mcl_3dl_compat::subscribe(
        nh_, "mapcloud",
        pnh_, "mapcloud", 1, &MCL3dlNode::cbMapcloud, this);
    sub_mapcloud_update_ = mcl_3dl_compat::subscribe(
        nh_, "mapcloud_update",
        pnh_, "mapcloud_update", 1, &MCL3dlNode::cbMapcloudUpdate, this);
    sub_position_ = mcl_3dl_compat::subscribe(
        nh_, "initialpose",
        pnh_, "initialpose", 1, &MCL3dlNode::cbPosition, this);
    sub_landmark_ = mcl_3dl_compat::subscribe(
        nh_, "mcl_measurement",
        pnh_, "landmark", 1, &MCL3dlNode::cbLandmark, this);

    // Set up publishers
    pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, false);
    pub_particle_ = pnh_.advertise<geometry_msgs::PoseArray>("particles", 1, true);
    pub_mapcloud_ = pnh_.advertise<sensor_msgs::PointCloud2>("updated_map", 1, true);
    pub_debug_marker_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug_marker", 1, true);
    pub_status_ = pnh_.advertise<mcl_3dl_msgs::Status>("status", 1, true);
    pub_matched_ = pnh_.advertise<sensor_msgs::PointCloud2>("matched", 2, true);
    pub_unmatched_ = pnh_.advertise<sensor_msgs::PointCloud2>("unmatched", 2, true);

    // Set up services
    srv_particle_size_ = mcl_3dl_compat::advertiseService(
        nh_, "resize_mcl_particle",
        pnh_, "resize_particle", &MCL3dlNode::cbResizeParticle, this);
    srv_global_localization_ = mcl_3dl_compat::advertiseService(
        nh_, "global_localization",
        pnh_, "global_localization", &MCL3dlNode::cbGlobalLocalization, this);
    srv_expansion_reset_ = mcl_3dl_compat::advertiseService(
        nh_, "expansion_resetting",
        pnh_, "expansion_resetting", &MCL3dlNode::cbExpansionReset, this);
    srv_load_pcd_ = nh_.advertiseService("load_pcd", &MCL3dlNode::cbLoadPCD, this);

    // Set up timer
    map_update_timer_ = nh_.createTimer(
        *params_.map_update_interval_,
        &MCL3dlNode::cbMapUpdateTimer, this);

    // Set up diagnostics
    diag_updater_.setHardwareID("none");
    diag_updater_.add("Status", this, &MCL3dlNode::diagnoseStatus);

    return true;
  }

protected:
  void setupEngineCallbacks()
  {
    using std::placeholders::_1;
    engine_.setPublishParticlesCallback(
        std::bind(&MCL3dlNode::publishParticles, this, _1));
    engine_.setPublishDebugMarkerCallback(
        std::bind(&MCL3dlNode::publishDebugMarker, this, _1));
    engine_.setPublishMatchedCallback(
        std::bind(&MCL3dlNode::publishMatched, this, _1));
    engine_.setPublishUnmatchedCallback(
        std::bind(&MCL3dlNode::publishUnmatched, this, _1));
    engine_.setPublishPoseCallback(
        std::bind(&MCL3dlNode::publishPose, this, _1));
    engine_.setPublishStatusCallback(
        std::bind(&MCL3dlNode::publishStatus, this, _1));
    engine_.setPublishMapCloudCallback(
        std::bind(&MCL3dlNode::publishMapCloud, this, _1));
    engine_.setBroadcastTransformCallback(
        std::bind(&MCL3dlNode::broadcastTransform, this, _1));
    engine_.setDiagUpdateCallback(
        std::bind(&MCL3dlNode::updateDiag, this));
    engine_.setCheckSubscribersCallback(
        std::bind(&MCL3dlNode::checkSubscribers, this, _1));
  }

  void publishParticles(const geometry_msgs::msg::PoseArray& pa)
  {
    pub_particle_.publish(static_cast<const geometry_msgs::PoseArray&>(pa));
  }
  void publishDebugMarker(const visualization_msgs::msg::MarkerArray& markers)
  {
    pub_debug_marker_.publish(static_cast<const visualization_msgs::MarkerArray&>(markers));
  }
  void publishMatched(const sensor_msgs::msg::PointCloud2& pc)
  {
    pub_matched_.publish(static_cast<const sensor_msgs::PointCloud2&>(pc));
  }
  void publishUnmatched(const sensor_msgs::msg::PointCloud2& pc)
  {
    pub_unmatched_.publish(static_cast<const sensor_msgs::PointCloud2&>(pc));
  }
  void publishPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose)
  {
    pub_pose_.publish(static_cast<const geometry_msgs::PoseWithCovarianceStamped&>(pose));
  }
  void publishStatus(const mcl_3dl_msgs::msg::Status& status)
  {
    pub_status_.publish(static_cast<const mcl_3dl_msgs::Status&>(status));
  }
  void publishMapCloud(const sensor_msgs::msg::PointCloud2& pc)
  {
    pub_mapcloud_.publish(static_cast<const sensor_msgs::PointCloud2&>(pc));
  }
  void broadcastTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
  {
    std::vector<geometry_msgs::TransformStamped> base_transforms(transforms.begin(), transforms.end());
    tfb_.sendTransform(base_transforms);
  }
  void updateDiag()
  {
    diag_updater_.force_update();
  }
  bool checkSubscribers(const std::string& topic)
  {
    if (topic == "matched")
      return pub_matched_.getNumSubscribers() > 0;
    if (topic == "unmatched")
      return pub_unmatched_.getNumSubscribers() > 0;
    return false;
  }

  // ROS callback wrappers - simply forward to engine
  void cbMapcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    engine_.processMapCloud(msg);
  }
  void cbMapcloudUpdate(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    engine_.processMapCloudUpdate(msg);
  }
  void cbPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    engine_.processPosition(msg);
  }
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
  {
    engine_.processOdom(msg);
  }
  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    engine_.processCloud(msg);
  }
  void cbImu(const sensor_msgs::Imu::ConstPtr& msg)
  {
    engine_.processImu(msg);
  }
  void cbLandmark(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    engine_.processLandmark(msg);
  }

  bool cbResizeParticle(mcl_3dl_msgs::ResizeParticleRequest& request,
                        mcl_3dl_msgs::ResizeParticleResponse& response)
  {
    return engine_.resizeParticle(request.size);
  }
  bool cbExpansionReset(std_srvs::TriggerRequest& request,
                        std_srvs::TriggerResponse& response)
  {
    return engine_.expansionReset();
  }
  bool cbGlobalLocalization(std_srvs::TriggerRequest& request,
                            std_srvs::TriggerResponse& response)
  {
    std::string message;
    response.success = engine_.globalLocalization(message);
    response.message = message;
    return true;
  }
  bool cbLoadPCD(mcl_3dl_msgs::LoadPCD::Request& req, mcl_3dl_msgs::LoadPCD::Response& resp)
  {
    resp.success = engine_.loadPCD(req.pcd_path);
    return true;
  }

  void cbMapUpdateTimer(const ros::TimerEvent& event)
  {
    engine_.mapUpdateTimer();
  }

  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    bool has_error, has_warn;
    std::string message;
    bool has_map, has_odom, has_imu;
    engine_.diagnoseStatus(has_error, has_warn, message, has_map, has_odom, has_imu);

    if (has_error)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, message);
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, message);
    }

    stat.add("Map Availability", has_map ? "true" : "false");
    stat.add("Odometry Availability", has_odom ? "true" : "false");
    stat.add("IMU Availability", has_imu ? "true" : "false");

    mcl_3dl_msgs::msg::Status status = engine_.getStatus();
    status.entropy = engine_.getEntropy();
    pub_status_.publish(static_cast<const mcl_3dl_msgs::Status&>(status));
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_mapcloud_;
  ros::Subscriber sub_mapcloud_update_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_position_;
  ros::Subscriber sub_landmark_;
  ros::Publisher pub_particle_;
  ros::Publisher pub_mapcloud_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_matched_;
  ros::Publisher pub_unmatched_;
  ros::Publisher pub_debug_marker_;
  ros::Publisher pub_status_;
  ros::Timer map_update_timer_;
  ros::ServiceServer srv_particle_size_;
  ros::ServiceServer srv_global_localization_;
  ros::ServiceServer srv_expansion_reset_;
  ros::ServiceServer srv_load_pcd_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  tf2_ros::TransformBroadcaster tfb_;

  diagnostic_updater::Updater diag_updater_;

  Parameters params_;
  MCL3dlEngine engine_;
};
}  // namespace mcl_3dl

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mcl_3dl");

  mcl_3dl::MCL3dlNode mcl;
  if (!mcl.configure())
  {
    return 1;
  }
  ros::spin();

  return 0;
}
