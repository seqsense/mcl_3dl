/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
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

// ROS 2 port of compare_pose.cpp.
//
// Behavior is equivalent to the ROS 1 rostest version: it subscribes to
// /amcl_pose, walks through the reference nav_msgs/Path stamped poses in
// time order, and asserts that each estimated pose is within
// `error_limit` and within 3-sigma of the reported covariance.
//
// The ROS 1 rostest published the reference Path via `rostopic pub -l -f
// <yaml>` and compare_pose subscribed to it; that mechanism does not
// exist in ROS 2, so this version loads the YAML reference directly via
// the `ref_path_file` parameter.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

namespace
{

// Parse the rostopic-style YAML dump of nav_msgs/Path.
// Tolerates both "secs/nsecs" (ROS 1 dump) and "sec/nanosec" (ROS 2) keys.
nav_msgs::msg::Path loadPathYaml(const std::string& path_file)
{
  std::ifstream ifs(path_file);
  if (!ifs)
  {
    throw std::runtime_error("failed to open ref Path file: " + path_file);
  }
  std::stringstream buf;
  buf << ifs.rdbuf();
  YAML::Node root = YAML::Load(buf.str());

  const auto stamp_of = [](const YAML::Node& n) -> builtin_interfaces::msg::Time
  {
    builtin_interfaces::msg::Time t;
    if (n["sec"])
      t.sec = n["sec"].as<int32_t>();
    else if (n["secs"])
      t.sec = n["secs"].as<int32_t>();
    if (n["nanosec"])
      t.nanosec = n["nanosec"].as<uint32_t>();
    else if (n["nsecs"])
      t.nanosec = n["nsecs"].as<uint32_t>();
    return t;
  };

  nav_msgs::msg::Path path;
  if (root["header"])
  {
    path.header.stamp = stamp_of(root["header"]["stamp"]);
    if (root["header"]["frame_id"])
      path.header.frame_id = root["header"]["frame_id"].as<std::string>();
  }
  for (const auto& p : root["poses"])
  {
    geometry_msgs::msg::PoseStamped ps;
    if (p["header"])
    {
      ps.header.stamp = stamp_of(p["header"]["stamp"]);
      if (p["header"]["frame_id"])
        ps.header.frame_id = p["header"]["frame_id"].as<std::string>();
    }
    const auto& pos = p["pose"]["position"];
    ps.pose.position.x = pos["x"].as<double>();
    ps.pose.position.y = pos["y"].as<double>();
    ps.pose.position.z = pos["z"].as<double>();
    const auto& q = p["pose"]["orientation"];
    ps.pose.orientation.x = q["x"].as<double>();
    ps.pose.orientation.y = q["y"].as<double>();
    ps.pose.orientation.z = q["z"].as<double>();
    ps.pose.orientation.w = q["w"].as<double>();
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace

class ComparePoseFixture : public ::testing::Test
{
protected:
  static rclcpp::Node::SharedPtr node_;

public:
  static void SetNode(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
  }
};

rclcpp::Node::SharedPtr ComparePoseFixture::node_ = nullptr;

TEST_F(ComparePoseFixture, Compare)
{
  ASSERT_NE(node_, nullptr) << "test node not initialized";

  const double error_limit = node_->declare_parameter<double>("error_limit", 0.3);
  const std::string ref_file = node_->declare_parameter<std::string>("ref_path_file", "");
  ASSERT_FALSE(ref_file.empty()) << "ref_path_file parameter must be set";

  const nav_msgs::msg::Path path = loadPathYaml(ref_file);
  ASSERT_FALSE(path.poses.empty()) << "ref Path is empty: " << ref_file;
  std::fprintf(stderr, "compare_pose: loaded %zu reference poses from %s\n",
               path.poses.size(), ref_file.c_str());

  size_t i_path = 0;
  bool finished = false;

  auto cb_pose = [&path, &i_path, &error_limit, &finished, this](
                     const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    if (i_path >= path.poses.size())
      return;
    const rclcpp::Time stamp(path.poses[i_path].header.stamp);
    if (stamp >= node_->now())
      return;

    const float x_error = path.poses[i_path].pose.position.x - msg->pose.pose.position.x;
    const float y_error = path.poses[i_path].pose.position.y - msg->pose.pose.position.y;
    const float z_error = path.poses[i_path].pose.position.z - msg->pose.pose.position.z;

    tf2::Quaternion q_ref(
        path.poses[i_path].pose.orientation.x, path.poses[i_path].pose.orientation.y,
        path.poses[i_path].pose.orientation.z, path.poses[i_path].pose.orientation.w);
    tf2::Quaternion q_est(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    float yaw_error = tf2::getYaw(q_ref) - tf2::getYaw(q_est);
    while (yaw_error > M_PI)
      yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI)
      yaw_error += 2 * M_PI;

    const float error =
        std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2) + std::pow(z_error, 2));
    const float x_sigma = std::sqrt(msg->pose.covariance[0 * 6 + 0]);
    const float y_sigma = std::sqrt(msg->pose.covariance[1 * 6 + 1]);
    const float z_sigma = std::sqrt(msg->pose.covariance[2 * 6 + 2]);
    const float yaw_sigma = std::sqrt(msg->pose.covariance[5 * 6 + 5]);

    std::fprintf(stderr, "compare_pose[%lu/%lu]:\n", i_path, path.poses.size());
    std::fprintf(stderr, "  position error/limit=%0.3f/%0.3f\n", error, error_limit);
    std::fprintf(stderr, "  x error/3sigma=%0.3f/%0.3f\n", x_error, x_sigma * 3.0);
    std::fprintf(stderr, "  y error/3sigma=%0.3f/%0.3f\n", y_error, y_sigma * 3.0);
    std::fprintf(stderr, "  z error/3sigma=%0.3f/%0.3f\n", z_error, z_sigma * 3.0);
    std::fprintf(stderr, "  yaw error/3sigma=%0.3f/%0.3f\n", yaw_error, yaw_sigma * 3.0);

    i_path++;
    if (i_path >= path.poses.size())
      finished = true;

    ASSERT_FALSE(error > error_limit) << "Position error is larger then expected.";
    ASSERT_FALSE(std::fabs(x_error) > x_sigma * 3.0)
        << "Estimated variance is too small to continue tracking. (x)";
    ASSERT_FALSE(std::fabs(y_error) > y_sigma * 3.0)
        << "Estimated variance is too small to continue tracking. (y)";
    ASSERT_FALSE(std::fabs(z_error) > z_sigma * 3.0)
        << "Estimated variance is too small to continue tracking. (z)";
    ASSERT_FALSE(std::fabs(yaw_error) > yaw_sigma * 3.0)
        << "Estimated variance is too small to continue tracking. (yaw)";
  };

  auto sub_pose = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 1, cb_pose);

  rclcpp::Rate wait(10);
  while (rclcpp::ok() && !finished)
  {
    rclcpp::spin_some(node_);
    wait.sleep();
  }
  std::fprintf(stderr, "compare_pose finished\n");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("compare_pose");
  ComparePoseFixture::SetNode(node);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
