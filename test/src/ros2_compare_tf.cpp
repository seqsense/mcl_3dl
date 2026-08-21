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

// ROS 2 port of compare_tf.cpp.
//
// Subscribes to /amcl_pose, looks up the current map->base_link transform
// at the same stamp via tf2_ros::Buffer, and asserts that the looked-up
// position differs from the published amcl_pose by < 5cm.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class CompareTfFixture : public ::testing::Test
{
protected:
  static rclcpp::Node::SharedPtr node_;

public:
  static void SetNode(const rclcpp::Node::SharedPtr & node) { node_ = node; }
};

rclcpp::Node::SharedPtr CompareTfFixture::node_ = nullptr;

TEST_F(CompareTfFixture, Compare)
{
  ASSERT_NE(node_, nullptr) << "test node not initialized";

  const int cnt_max = node_->declare_parameter<int>("cnt_max", 10);

  tf2_ros::Buffer tfbuf(node_->get_clock());
  tf2_ros::TransformListener tfl(tfbuf, node_);

  int cnt = 0;
  size_t tf_ex_cnt = 0;
  bool finished = false;

  auto cb_pose = [&tfbuf, &cnt, &cnt_max, &tf_ex_cnt, &finished](
                   const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg) {
    geometry_msgs::msg::PoseStamped pose;
    try {
      geometry_msgs::msg::PoseStamped pose_bl;
      pose_bl.header.frame_id = "base_link";
      pose_bl.header.stamp = msg->header.stamp;
      pose_bl.pose.orientation.w = 1.0;
      const auto trans = tfbuf.lookupTransform(
        "map", pose_bl.header.frame_id, pose_bl.header.stamp, tf2::durationFromSec(0.1));
      tf2::doTransform(pose_bl, pose, trans);
    } catch (const tf2::TransformException & e) {
      tf_ex_cnt++;
      return;
    }
    const float x_error = pose.pose.position.x - msg->pose.pose.position.x;
    const float y_error = pose.pose.position.y - msg->pose.pose.position.y;
    const float z_error = pose.pose.position.z - msg->pose.pose.position.z;
    const float error =
      std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2) + std::pow(z_error, 2));

    std::fprintf(stderr, "compare_tf[%d/%d]:\n", cnt, cnt_max);
    std::fprintf(stderr, "  error=%0.3f\n", error);

    cnt++;
    if (cnt >= cnt_max) finished = true;

    ASSERT_FALSE(error > 0.05) << "tf output diverges from amcl_pose.";
  };

  auto sub_pose = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 1, cb_pose);

  rclcpp::Rate wait(1);
  while (rclcpp::ok() && !finished) {
    rclcpp::spin_some(node_);
    wait.sleep();
  }

  ASSERT_FALSE(tf_ex_cnt > 1) << "tf exception occurs more than once.";
  std::fprintf(stderr, "compare_tf finished\n");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("compare_tf");
  CompareTfFixture::SetNode(node);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
