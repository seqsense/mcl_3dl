/*
 * Copyright (c) 2026, the mcl_3dl authors
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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
sensor_msgs::msg::PointCloud2 generateMapCloud(
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(9);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      *iter_x = static_cast<float>(i) * 0.5f;
      *iter_y = static_cast<float>(j) * 0.5f;
      *iter_z = 0.0f;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }
  return cloud;
}
}  // namespace

TEST(MapPublishedBeforeSubscribe, MapIsStillReceived)
{
  auto node = rclcpp::Node::make_shared("test_map_before_subscribe");
  auto pub_mapcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapcloud", rclcpp::QoS(1).transient_local());

  ASSERT_EQ(pub_mapcloud->get_subscription_count(), 0u);
  pub_mapcloud->publish(generateMapCloud("map", node->now()));

  // updated_map is published only while mcl_3dl holds a map.
  bool map_loaded = false;
  auto sub_updated_map = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mcl_3dl/updated_map", rclcpp::QoS(1).transient_local(),
    [&map_loaded](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      if (msg->width * msg->height > 0) map_loaded = true;
    });

  bool subscribed = false;
  const auto startup_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  auto map_deadline = startup_deadline;
  while (rclcpp::ok() && !map_loaded) {
    rclcpp::spin_some(node);
    if (!subscribed && pub_mapcloud->get_subscription_count() > 0) {
      subscribed = true;
      map_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    }
    const auto now = std::chrono::steady_clock::now();
    if (now > (subscribed ? map_deadline : startup_deadline)) {
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  ASSERT_TRUE(subscribed) << "mcl_3dl never subscribed to mapcloud";
  EXPECT_TRUE(map_loaded) << "mcl_3dl did not load the map that was published, latched, before "
                             "it subscribed";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
