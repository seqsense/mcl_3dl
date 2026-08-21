/*
 * Copyright (c) 2018-2019, the mcl_3dl authors
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

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
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
  // Draw cube
  for (float x = -1; x < 1; x += 0.05) {
    for (float y = -1; y < 1; y += 0.05) {
      points.push_back(Point(1.0 / 2 + offset_x, y + offset_y, x + offset_z));
      points.push_back(Point(-1.0 / 2 + offset_x, y + offset_y, x + offset_z));
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
  rclcpp::Node::SharedPtr /*node*/, const float offset_x, const float offset_y,
  const float offset_z)
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
}  // namespace

TEST(DebugOutput, MatchedUnmatched)
{
  auto node = rclcpp::Node::make_shared("test_debug_output");

  sensor_msgs::msg::PointCloud2::ConstSharedPtr matched, unmatched;

  auto sub_matched = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mcl_3dl/matched", 1,
    [&matched](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { matched = msg; });
  auto sub_unmatched = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mcl_3dl/unmatched", 1,
    [&unmatched](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { unmatched = msg; });

  auto pub_mapcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapcloud", rclcpp::QoS(1).transient_local());
  auto pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
  auto pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  auto pub_odom = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  const float offset_x = 1;
  const float offset_y = 0;
  const float offset_z = 0;
  const auto map_msg = generateMapMsg(node, offset_x, offset_y, offset_z);

  // Wait for DDS discovery and map processing
  rclcpp::sleep_for(std::chrono::seconds(2));
  rclcpp::WallRate rate(10);
  for (int i = 0; i < 80; ++i) {
    rate.sleep();
    rclcpp::spin_some(node);
    if (matched && unmatched) break;
    // Re-publish map periodically to handle late subscriber discovery
    if (i % 10 == 0) pub_mapcloud->publish(map_msg);
    pub_cloud->publish(generateCloudMsg(node));
    pub_imu->publish(generateImuMsg(node));
    pub_odom->publish(generateOdomMsg(node));
  }
  ASSERT_TRUE(rclcpp::ok());

  ASSERT_TRUE(static_cast<bool>(matched));
  ASSERT_TRUE(static_cast<bool>(unmatched));

  ASSERT_GT(matched->width * matched->height, 0u);
  ASSERT_GT(unmatched->width * unmatched->height, 0u);

  {
    sensor_msgs::PointCloud2ConstIterator<float> x(*matched, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*matched, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*matched, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      ASSERT_NEAR(*x, 0.5f, 0.1f);
      ASSERT_TRUE(-1.1 < *y && *y < 1.1);
      ASSERT_TRUE(-1.1 < *z && *z < 1.1);
    }
  }
  {
    sensor_msgs::PointCloud2ConstIterator<float> x(*unmatched, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*unmatched, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*unmatched, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      ASSERT_NEAR(*x, -0.5f, 0.1f);
      ASSERT_TRUE(-1.1 < *y && *y < 1.1);
      ASSERT_TRUE(-1.1 < *z && *z < 1.1);
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
