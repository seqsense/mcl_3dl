/*
 * Copyright (c) 2018, the mcl_3dl authors
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

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mcl_3dl_msgs/msg/status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
void generateSamplePointcloud2(
  sensor_msgs::msg::PointCloud2 & cloud, const float offset_x, const float offset_y,
  const float offset_z)
{
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  class Point
  {
  public:
    float x_, y_, z_;
    Point(const float x, const float y, const float z) : x_(x), y_(y), z_(z) {}
  };
  std::vector<Point> points;
  for (float x = -1; x < 1; x += 0.05) {
    for (float y = -1; y < 1; y += 0.05) {
      points.push_back(Point(x / 2 + offset_x, y + offset_y, 1.0 + offset_z));
      points.push_back(Point(x / 2 + offset_x, y + offset_y, -1.0 + offset_z));
      points.push_back(Point(1.0 / 2 + offset_x, y + offset_y, x + offset_z));
      points.push_back(Point(-1.0 / 2 + offset_x, y + offset_y, x + offset_z));
      points.push_back(Point(x / 2 + offset_x, 1.0 + offset_y, y + offset_z));
      points.push_back(Point(x / 2 + offset_x, -1.0 + offset_y, y + offset_z));
    }
  }

  modifier.resize(points.size());
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const Point & p : points) {
    *iter_x = p.x_;
    *iter_y = p.y_;
    *iter_z = p.z_;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

sensor_msgs::msg::PointCloud2 generateMapMsg(
  const float offset_x, const float offset_y, const float offset_z)
{
  sensor_msgs::msg::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, offset_x, offset_y, offset_z);
  cloud.header.frame_id = "map";
  return cloud;
}
sensor_msgs::msg::PointCloud2 generateCloudMsg(rclcpp::Node::SharedPtr node)
{
  sensor_msgs::msg::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, 0, 0, 0);
  cloud.header.frame_id = "base_link";
  cloud.header.stamp = node->now();
  return cloud;
}
sensor_msgs::msg::Imu generateImuMsg(rclcpp::Node::SharedPtr node)
{
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  imu.header.stamp = node->now();
  imu.orientation.w = 1;
  imu.linear_acceleration.z = 9.8;
  return imu;
}
nav_msgs::msg::Odometry generateOdomMsg(rclcpp::Node::SharedPtr node)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.stamp = node->now();
  odom.pose.pose.position.x = 1;
  odom.pose.pose.orientation.w = 1;
  return odom;
}
geometry_msgs::msg::PoseWithCovarianceStamped generateInitialPose(rclcpp::Node::SharedPtr node)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node->now();
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance[6 * 0 + 0] = 0.05 * 0.05;
  pose.pose.covariance[6 * 1 + 1] = 0.05 * 0.05;
  pose.pose.covariance[6 * 2 + 2] = 0.05 * 0.05;
  pose.pose.covariance[6 * 3 + 3] = 0.0;
  pose.pose.covariance[6 * 4 + 4] = 0.0;
  pose.pose.covariance[6 * 5 + 5] = 0.05 * 0.05;
  return pose;
}
}  // namespace

class ExpansionResetting : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_;
  rclcpp::Subscription<mcl_3dl_msgs::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_mapcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr src_expansion_resetting_;

  geometry_msgs::msg::PoseArray::ConstSharedPtr poses_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_;
  mcl_3dl_msgs::msg::Status::ConstSharedPtr status_;

  bool findTruePose(const tf2::Transform & true_pose)
  {
    if (!poses_) return false;

    bool found_true_positive(false);
    for (const auto & pose : poses_->poses) {
      tf2::Transform particle_pose;
      tf2::fromMsg(pose, particle_pose);

      const tf2::Transform tf_diff = particle_pose.inverse() * true_pose;
      if (tf_diff.getOrigin().length() < 2e-1 && fabs(tf2::getYaw(tf_diff.getRotation())) < 2e-1)
        found_true_positive = true;
    }
    return found_true_positive;
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_expansion_resetting");

    sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "mcl_3dl/particles", 1,
      [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr & msg) { poses_ = msg; });
    sub_status_ = node_->create_subscription<mcl_3dl_msgs::msg::Status>(
      "mcl_3dl/status", 1,
      [this](const mcl_3dl_msgs::msg::Status::ConstSharedPtr & msg) { status_ = msg; });
    sub_pose_cov_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 1,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg) {
        pose_cov_ = msg;
      });

    pub_mapcloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "mapcloud", rclcpp::QoS(1).transient_local());
    pub_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
    pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    pub_init_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1).transient_local());

    src_expansion_resetting_ =
      node_->create_client<std_srvs::srv::Trigger>("mcl_3dl/expansion_resetting");

    ASSERT_TRUE(src_expansion_resetting_->wait_for_service(std::chrono::seconds(10)));

    pub_init_->publish(generateInitialPose(node_));
    rclcpp::WallRate wait(10);
    for (int i = 0; i < 100; i++) {
      wait.sleep();
      rclcpp::spin_some(node_);
      if (pose_cov_) break;
      if (!rclcpp::ok()) break;
    }
  }
};

TEST_F(ExpansionResetting, ExpandAndResume)
{
  const float offset_x = 1;
  const float offset_y = 0;
  const float offset_z = 0;
  const auto map_msg = generateMapMsg(offset_x, offset_y, offset_z);
  pub_mapcloud_->publish(map_msg);

  rclcpp::sleep_for(std::chrono::seconds(2));
  rclcpp::WallRate rate(10);
  // Wait until finishing expansion resetting
  for (int i = 0; i < 40; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    if (status_ && status_->status == mcl_3dl_msgs::msg::Status::EXPANSION_RESETTING) i = 0;
    if (i % 10 == 0) pub_mapcloud_->publish(map_msg);
    pub_cloud_->publish(generateCloudMsg(node_));
    pub_imu_->publish(generateImuMsg(node_));
    pub_odom_->publish(generateOdomMsg(node_));
  }
  ASSERT_TRUE(rclcpp::ok());

  ASSERT_TRUE(static_cast<bool>(status_));
  ASSERT_TRUE(static_cast<bool>(poses_));

  ASSERT_TRUE(findTruePose(
    tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(offset_x, offset_y, offset_z))));
}

TEST_F(ExpansionResetting, ManualExpand)
{
  const float offset_x = 1;
  const float offset_y = 0;
  const float offset_z = 0;
  const auto map_msg = generateMapMsg(offset_x, offset_y, offset_z);
  pub_mapcloud_->publish(map_msg);

  ASSERT_TRUE(rclcpp::ok());
  rclcpp::WallRate rate(10);

  // Ensure that the node is not in expansion resetting mode
  for (int i = 0; i < 40; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    if (i > 5 && status_ && status_->status != mcl_3dl_msgs::msg::Status::EXPANSION_RESETTING)
      break;

    if (i % 10 == 0) pub_mapcloud_->publish(map_msg);
    pub_cloud_->publish(generateCloudMsg(node_));
    pub_imu_->publish(generateImuMsg(node_));
    pub_odom_->publish(generateOdomMsg(node_));
  }
  ASSERT_TRUE(rclcpp::ok());

  status_ = nullptr;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = src_expansion_resetting_->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  ASSERT_TRUE(future.valid());

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // Wait until finishing expansion resetting
  for (int i = 0; i < 40; ++i) {
    rate.sleep();
    rclcpp::spin_some(node_);
    if (status_) {
      ASSERT_NE(status_->status, mcl_3dl_msgs::msg::Status::EXPANSION_RESETTING);
    }
    pub_cloud_->publish(generateCloudMsg(node_));
    pub_imu_->publish(generateImuMsg(node_));
    pub_odom_->publish(generateOdomMsg(node_));
  }
  ASSERT_TRUE(rclcpp::ok());

  ASSERT_TRUE(static_cast<bool>(status_));
  ASSERT_TRUE(static_cast<bool>(poses_));

  ASSERT_TRUE(findTruePose(
    tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(offset_x, offset_y, offset_z))));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
