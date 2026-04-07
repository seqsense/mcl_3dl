/*
 * Copyright (c) 2016-2025, the mcl_3dl authors
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

#include <chrono>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

TEST(Parameters, DynamicParameters)
{
  auto node = rclcpp::Node::make_shared("test_parameters");

  // Create a client for the mcl_3dl node's set_parameters service
  auto param_client = node->create_client<rcl_interfaces::srv::SetParameters>(
      "mcl_3dl/set_parameters");

  ASSERT_TRUE(param_client->wait_for_service(std::chrono::seconds(10)));

  // Set new values via the parameter service
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  rcl_interfaces::msg::Parameter p1;
  p1.name = "std_warn_thresh_xy";
  p1.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  p1.value.double_value = 0.5;
  request->parameters.push_back(p1);

  rcl_interfaces::msg::Parameter p2;
  p2.name = "std_warn_thresh_z";
  p2.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  p2.value.double_value = 0.6;
  request->parameters.push_back(p2);

  rcl_interfaces::msg::Parameter p3;
  p3.name = "std_warn_thresh_yaw";
  p3.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  p3.value.double_value = 0.7;
  request->parameters.push_back(p3);

  auto future = param_client->async_send_request(request);
  ASSERT_EQ(
      rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);

  auto response = future.get();
  ASSERT_EQ(response->results.size(), 3u);
  for (const auto& result : response->results)
  {
    ASSERT_TRUE(result.successful);
  }

  // Verify parameters were set by reading them back
  auto get_client = node->create_client<rcl_interfaces::srv::GetParameters>(
      "mcl_3dl/get_parameters");
  ASSERT_TRUE(get_client->wait_for_service(std::chrono::seconds(5)));

  auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  get_request->names = {"std_warn_thresh_xy", "std_warn_thresh_z", "std_warn_thresh_yaw"};

  auto get_future = get_client->async_send_request(get_request);
  ASSERT_EQ(
      rclcpp::spin_until_future_complete(node, get_future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);

  auto get_response = get_future.get();
  ASSERT_EQ(get_response->values.size(), 3u);
  ASSERT_DOUBLE_EQ(get_response->values[0].double_value, 0.5);
  ASSERT_DOUBLE_EQ(get_response->values[1].double_value, 0.6);
  ASSERT_DOUBLE_EQ(get_response->values[2].double_value, 0.7);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
