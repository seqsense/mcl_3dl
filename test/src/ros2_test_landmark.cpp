/*
 * Copyright (c) 2019, the mcl_3dl authors
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

#include <limits>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <gtest/gtest.h>

namespace
{
geometry_msgs::msg::PoseWithCovarianceStamped generatePoseWithCov(
    rclcpp::Node::SharedPtr node,
    const float y, const float y_var, const float var_measure = 0.0)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node->now();
  pose.pose.pose.position.y = y;
  pose.pose.pose.orientation.w = 1.0;
  pose.pose.covariance[6 * 0 + 0] = var_measure;
  pose.pose.covariance[6 * 1 + 1] = y_var;
  pose.pose.covariance[6 * 2 + 2] = var_measure;
  pose.pose.covariance[6 * 3 + 3] = var_measure;
  pose.pose.covariance[6 * 4 + 4] = var_measure;
  pose.pose.covariance[6 * 5 + 5] = var_measure;
  return pose;
}
std::pair<float, float> getMean(const std::vector<geometry_msgs::msg::Pose>& poses)
{
  float mean = 0;
  for (const geometry_msgs::msg::Pose p : poses)
  {
    mean += p.position.y;
  }
  mean /= poses.size();

  float root_mean = 0;
  for (const geometry_msgs::msg::Pose p : poses)
  {
    root_mean += std::pow(p.position.y - mean, 2);
  }
  root_mean /= poses.size();

  return std::pair<float, float>(mean, root_mean);
}
}  // namespace

TEST(Landmark, Measurement)
{
  auto node = rclcpp::Node::make_shared("test_landmark");

  geometry_msgs::msg::PoseArray::ConstSharedPtr poses;

  auto sub_pose = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "mcl_3dl/particles", 1,
      [&poses](const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg)
      {
        poses = msg;
      });

  auto pub_init = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1).transient_local());
  auto pub_landmark = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "mcl_measurement", rclcpp::QoS(1).transient_local());

  rclcpp::sleep_for(std::chrono::seconds(1));
  pub_init->publish(generatePoseWithCov(node, 2.0, 1.0));

  rclcpp::WallRate wait(10);
  for (int i = 0; i < 100; i++)
  {
    wait.sleep();
    rclcpp::spin_some(node);
    if (poses)
      break;
    ASSERT_TRUE(rclcpp::ok());
  }
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  ASSERT_TRUE(static_cast<bool>(poses));
  for (const geometry_msgs::msg::Pose p : poses->poses)
  {
    ASSERT_FLOAT_EQ(p.position.x, 0.0f);
    ASSERT_FLOAT_EQ(p.position.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.x, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.y, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.w, 1.0f);
  }
  const std::pair<float, float> mean_init = getMean(poses->poses);
  ASSERT_NEAR(mean_init.first, 2.0f, 0.1f);
  ASSERT_NEAR(mean_init.second, 1.0f, 0.1f);

  poses = nullptr;
  pub_landmark->publish(generatePoseWithCov(node, 2.6, 1.0, 1000.0 * 1000.0));
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  ASSERT_TRUE(static_cast<bool>(poses));
  for (const geometry_msgs::msg::Pose p : poses->poses)
  {
    ASSERT_FLOAT_EQ(p.position.x, 0.0f);
    ASSERT_FLOAT_EQ(p.position.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.x, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.y, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.z, 0.0f);
    ASSERT_FLOAT_EQ(p.orientation.w, 1.0f);
  }
  const std::pair<float, float> mean_measured = getMean(poses->poses);
  ASSERT_NEAR(mean_measured.first, 2.3f, 0.1f);
  ASSERT_NEAR(mean_measured.second, 0.5f, 0.1f);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
