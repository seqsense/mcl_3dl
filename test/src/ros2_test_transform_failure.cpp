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

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
void GenerateSinglePointPointcloud2(
  sensor_msgs::msg::PointCloud2 & cloud, const float x, const float y, const float z)
{
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  modifier.resize(1);
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
}
void publishSinglePointPointcloud2(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub, const float x, const float y,
  const float z, const std::string frame_id, const rclcpp::Time stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  GenerateSinglePointPointcloud2(cloud, x, y, z);
  pub->publish(cloud);
}
}  // namespace

TEST(TransformFailure, NoDeadAgainstTransformFailure)
{
  auto node = rclcpp::Node::make_shared("test_transform_failure");
  tf2_ros::TransformBroadcaster tfb(node);
  auto pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
  auto pub_mapcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapcloud", rclcpp::QoS(1).transient_local());

  rclcpp::WallRate rate(10);
  publishSinglePointPointcloud2(pub_mapcloud, 0.0, 0.0, 0.0, "map", node->now());

  // Wait a bit for the mcl_3dl node to be ready
  rclcpp::sleep_for(std::chrono::seconds(2));

  int cnt = 0;
  // mcl_3dl is launched in the same test. We just verify it does not crash.
  while (rclcpp::ok()) {
    ++cnt;
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = rclcpp::Time(node->now()) + rclcpp::Duration::from_seconds(0.1);
    trans.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));

    trans.header.frame_id = "laser_link_base";
    trans.child_frame_id = "laser_link";
    tfb.sendTransform(trans);

    trans.header.frame_id = "map";
    trans.child_frame_id = "odom";
    tfb.sendTransform(trans);

    if (cnt > 10) {
      trans.header.frame_id = "base_link";
      trans.child_frame_id = "laser_link_base";
      tfb.sendTransform(trans);
    }
    if (cnt > 20) {
      trans.header.frame_id = "odom";
      trans.child_frame_id = "base_link";
      tfb.sendTransform(trans);
    }
    if (cnt > 30) break;

    publishSinglePointPointcloud2(pub_cloud, 0.0, 0.0, 0.0, "laser_link", node->now());

    rclcpp::spin_some(node);
    rate.sleep();
  }
  ASSERT_TRUE(rclcpp::ok());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
