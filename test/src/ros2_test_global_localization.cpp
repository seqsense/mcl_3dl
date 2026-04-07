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

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mcl_3dl_msgs/msg/status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

namespace
{
void generateSamplePointcloud2(
    sensor_msgs::msg::PointCloud2& cloud,
    const float x0, const float y0,
    const float x1, const float y1,
    const float offset_x, const float offset_y,
    const float offset_z, const float offset_yaw)
{
  std::random_device seed;
  std::default_random_engine engine(seed());
  std::normal_distribution<float> rand(0, 0.01);

  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  class Point
  {
  public:
    float x_, y_, z_;
    Point(const float x, const float y, const float z)
      : x_(x), y_(y), z_(z) {}
  };
  std::vector<Point> points;
  const float grid_xy = 0.15;
  const float grid_z = 0.1;
  const float floor_size = 6.0;
  for (float x = -floor_size; x < floor_size; x += grid_xy)
    for (float y = -floor_size; y < floor_size; y += grid_xy)
      if (x0 < x && x < x1 && y0 < y && y < y1)
        points.push_back(Point(x, y, 0.0));
  for (float x = -2; x < 1.6; x += grid_xy)
    if (x0 < x && x < x1)
      for (float z = 0.5; z < 1.5; z += grid_z)
        points.push_back(Point(x, 1.6, z));
  for (float y = -0.5; y < 1.6; y += grid_xy)
    if (y0 < y && y < y1)
      for (float z = 0.5; z < 2.0; z += grid_z)
        points.push_back(Point(1.6, y, z));
  for (float x = 0; x < 1.6; x += grid_xy)
    if (x0 < x && x < x1)
      for (float z = 0.5; z < 2.0; z += grid_z)
        points.push_back(Point(x, -2.1 + x, z));

  const float o_cos = cosf(offset_yaw);
  const float o_sin = sinf(offset_yaw);
  modifier.resize(points.size());
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const Point& p : points)
  {
    *iter_x = p.x_ * o_cos - p.y_ * o_sin + offset_x + rand(engine);
    *iter_y = p.x_ * o_sin + p.y_ * o_cos + offset_y + rand(engine);
    *iter_z = p.z_ + offset_z + rand(engine);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

sensor_msgs::msg::PointCloud2 generateMapMsg()
{
  sensor_msgs::msg::PointCloud2 cloud;
  generateSamplePointcloud2(cloud, -100, -100, 100, 100, 0, 0, 0, 0);
  cloud.header.frame_id = "map";
  return cloud;
}
sensor_msgs::msg::PointCloud2 generateCloudMsg(
    rclcpp::Node::SharedPtr node,
    const float offset_x, const float offset_y,
    const float offset_z, const float offset_yaw)
{
  sensor_msgs::msg::PointCloud2 cloud;
  generateSamplePointcloud2(
      cloud, -2, -2, 2, 2,
      offset_x, offset_y, offset_z, offset_yaw);
  cloud.header.frame_id = "laser";
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
nav_msgs::msg::Odometry generateOdomMsg(rclcpp::Node::SharedPtr node, const float x)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.stamp = node->now();
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = 5;
  odom.pose.pose.orientation.w = 1;
  return odom;
}
}  // namespace

class GlobalLocalization : public ::testing::TestWithParam<float>
{
};

INSTANTIATE_TEST_CASE_P(
    OdometryOffset, GlobalLocalization,
    ::testing::Values(5.0, 100.0));

TEST_P(GlobalLocalization, Localize)
{
  auto node = rclcpp::Node::make_shared("test_global_localization");

  geometry_msgs::msg::PoseArray::ConstSharedPtr poses;
  mcl_3dl_msgs::msg::Status::ConstSharedPtr status;

  auto sub_pose = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "mcl_3dl/particles", 1,
      [&poses](const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) { poses = msg; });
  auto sub_status = node->create_subscription<mcl_3dl_msgs::msg::Status>(
      "mcl_3dl/status", 1,
      [&status](const mcl_3dl_msgs::msg::Status::ConstSharedPtr& msg) { status = msg; });

  auto src_global_localization =
      node->create_client<std_srvs::srv::Trigger>("mcl_3dl/global_localization");

  auto pub_mapcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "mapcloud", rclcpp::QoS(1).transient_local());
  auto pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
  auto pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  auto pub_odom = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  const auto map_msg = generateMapMsg();
  pub_mapcloud->publish(map_msg);
  rclcpp::sleep_for(std::chrono::seconds(2));
  rclcpp::WallRate wait(10);
  for (int i = 0; i < 100; i++)
  {
    wait.sleep();
    rclcpp::spin_some(node);
    if (poses)
      break;
    if (i % 10 == 0)
      pub_mapcloud->publish(map_msg);
    ASSERT_TRUE(rclcpp::ok());
  }

  for (float offset_x = -0.5; offset_x <= 0.51; offset_x += 1.0)
  {
    for (float offset_yaw = -M_PI / 2; offset_yaw <= M_PI / 2 + 0.1; offset_yaw += M_PI)
    {
      const float laser_frame_height = 0.5;
      const float offset_y = 0.54;
      const float offset_z = 0.0;

      rclcpp::WallRate rate(10);
      // Wait until mcl_3dl initialization
      while (rclcpp::ok())
      {
        rate.sleep();
        rclcpp::spin_some(node);
        if (status && status->status == mcl_3dl_msgs::msg::Status::NORMAL)
          break;
        pub_cloud->publish(
            generateCloudMsg(node, offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu->publish(generateImuMsg(node));
        pub_odom->publish(generateOdomMsg(node, 0.0));
      }
      ASSERT_TRUE(rclcpp::ok());
      for (int i = 0; i < 5; ++i)
      {
        pub_odom->publish(generateOdomMsg(node, GetParam()));
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node);
      }

      ASSERT_TRUE(src_global_localization->wait_for_service(std::chrono::seconds(5)));
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = src_global_localization->async_send_request(request);
      rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
      rclcpp::spin_some(node);

      // Wait until starting global localization
      while (rclcpp::ok())
      {
        rate.sleep();
        rclcpp::spin_some(node);
        if (status && status->status == mcl_3dl_msgs::msg::Status::GLOBAL_LOCALIZATION)
          break;
        pub_cloud->publish(
            generateCloudMsg(node, offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu->publish(generateImuMsg(node));
        pub_odom->publish(generateOdomMsg(node, GetParam()));
      }
      ASSERT_TRUE(rclcpp::ok());

      // Wait until finishing global localization
      while (rclcpp::ok())
      {
        rate.sleep();
        rclcpp::spin_some(node);
        if (status && status->status == mcl_3dl_msgs::msg::Status::NORMAL)
          break;
        pub_cloud->publish(
            generateCloudMsg(node, offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu->publish(generateImuMsg(node));
        pub_odom->publish(generateOdomMsg(node, GetParam()));
      }
      ASSERT_TRUE(rclcpp::ok());

      // Wait to improve accuracy
      for (int i = 0; i < 40; ++i)
      {
        rate.sleep();
        rclcpp::spin_some(node);
        pub_cloud->publish(
            generateCloudMsg(node, offset_x, offset_y, offset_z - laser_frame_height, offset_yaw));
        pub_imu->publish(generateImuMsg(node));
        pub_odom->publish(generateOdomMsg(node, GetParam()));
      }
      ASSERT_TRUE(rclcpp::ok());

      ASSERT_TRUE(static_cast<bool>(status));
      ASSERT_TRUE(static_cast<bool>(poses));

      const tf2::Transform true_pose(
          tf2::Quaternion(0, 0, sinf(-offset_yaw / 2), cosf(-offset_yaw / 2)),
          tf2::Vector3(
              -(offset_x * cos(-offset_yaw) - offset_y * sin(-offset_yaw)),
              -(offset_x * sin(-offset_yaw) + offset_y * cos(-offset_yaw)),
              -offset_z));
      bool found_true_positive(false);
      float dist_err_min = std::numeric_limits<float>::max();
      float ang_err_min = std::numeric_limits<float>::max();
      for (const auto& pose : poses->poses)
      {
        tf2::Transform particle_pose;
        tf2::fromMsg(pose, particle_pose);

        const tf2::Transform tf_diff = particle_pose.inverse() * true_pose;
        const float dist_err = tf_diff.getOrigin().length();
        const float ang_err = fabs(tf2::getYaw(tf_diff.getRotation()));
        if (dist_err < 2e-1 && ang_err < 2e-1)
          found_true_positive = true;

        if (dist_err_min > dist_err)
        {
          dist_err_min = dist_err;
          ang_err_min = ang_err;
        }
      }
      ASSERT_TRUE(found_true_positive)
          << "Minimum position error: " << dist_err_min << std::endl
          << "Angular error: " << ang_err_min << std::endl;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
