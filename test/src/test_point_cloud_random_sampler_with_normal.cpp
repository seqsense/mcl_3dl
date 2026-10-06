/*
 * Copyright (c) 2020, the mcl_3dl authors
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

#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>

#include "mcl_3dl/parameters.h"
#include "mcl_3dl/point_cloud_random_samplers/point_cloud_sampler_with_normal.h"
#include "mcl_3dl/point_types.h"
#include "pcl/features/normal_3d.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"

namespace mcl_3dl
{
namespace test
{
std::vector<State6DOF> buildPoseCovarianceMatrix(
  const double yaw, const double front_std_dev, const double side_std_dev)
{
  Eigen::Matrix2d vt;
  vt(0, 0) = std::cos(yaw);
  vt(0, 1) = -std::sin(yaw);
  vt(1, 0) = std::sin(yaw);
  vt(1, 1) = std::cos(yaw);
  Eigen::Matrix2d m;
  m(0, 0) = std::pow(front_std_dev, 2);
  m(0, 1) = 0;
  m(1, 0) = 0;
  m(1, 1) = std::pow(side_std_dev, 2);
  const Eigen::Matrix2d xv_cov = vt.transpose() * m * vt;

  std::vector<State6DOF> result(6);
  for (auto & state : result) {
    for (size_t i = 0; i < 6; ++i) {
      state[i] = 0.0;
    }
  }
  result[0][0] = xv_cov(0, 0);
  result[1][0] = xv_cov(1, 0);
  result[0][1] = xv_cov(0, 1);
  result[1][1] = xv_cov(1, 1);
  return result;
}

void buildWall(
  pcl::PointCloud<PointXYZIL>::Ptr result_points, double rotation_angle, int wall_length)
{
  pcl::PointCloud<PointXYZIL> points;
  for (int ny = 0; ny < wall_length; ++ny) {
    for (int nz = 0; nz < wall_length; ++nz) {
      PointXYZIL point;
      point.x = 20.0;
      point.y = (ny - wall_length / 2) * 0.05;
      point.z = (nz - wall_length / 2) * 0.05;
      point.label = 0;
      point.intensity = 3000;
      points.push_back(point);
    }
  }
  const State6DOF s(mcl_3dl::Vec3(0, 0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), rotation_angle));
  s.transform(points);
  result_points->insert(result_points->end(), points.begin(), points.end());
}

TEST(PointCloudSamplerWithNormal, Sampling)
{
  pcl::PointCloud<PointXYZIL>::Ptr pc(new pcl::PointCloud<PointXYZIL>());
  int wall_length = 20;
  // Wall at right angles to the first principal component of particles
  buildWall(pc, 0, wall_length);
  // Wall parallel to the first principal component of particles
  buildWall(pc, M_PI / 2, wall_length);

  const double robot_yaw = M_PI / 6;
  const State6DOF mean(
    mcl_3dl::Vec3(3.5, -5.0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), robot_yaw));
  const std::vector<State6DOF> cov_matrix = buildPoseCovarianceMatrix(robot_yaw, 1.0, 0.2);

  // Fix random seeds to avoid flaky results
  const std::vector<unsigned int> seeds = {
    12345, 23456, 34567, 45678, 56789,
  };

  struct ParameterSet
  {
    double perform_weighting_ratio;
    double max_weight_ratio;
    double max_weight;
    double expected_ratio_min;
    double expected_ratio_max;
  };
  const std::vector<ParameterSet> parameters = {
    // Weights of points in the wall at right angles: 10, weights of points in the parallel wall: 1
    {2.0, 4.0, 10.0, 0.85, 1.0},
    // Weights of points in the wall at right angles: 1, weights of points in the parallel wall: 1
    {6.0, 7.0, 10.0, 0.4, 0.6},
    // Weights of points in the wall at right angles: 3, weights of points in the parallel wall: 1
    {2.0, 8.0, 5.0, 0.65, 0.85},
  };

  for (const unsigned int seed : seeds) {
    auto sampler_params = std::make_shared<PointCloudSamplerWithNormalParameters>();
    sampler_params->normal_search_range_ = 0.4;
    PointCloudSamplerWithNormal<PointXYZIL> sampler(sampler_params, seed);
    sampler.setParticleStatistics(mean, cov_matrix);
    const int sample_num = 100;
    for (const ParameterSet & parameter : parameters) {
      sampler_params->perform_weighting_ratio_ = parameter.perform_weighting_ratio;
      sampler_params->max_weight_ratio_ = parameter.max_weight_ratio;
      sampler_params->max_weight_ = parameter.max_weight;
      sampler.refreshParameters();

      const pcl::PointCloud<PointXYZIL>::Ptr extracted_cloud = sampler.sample(pc, sample_num);
      EXPECT_EQ(sample_num, static_cast<int>(extracted_cloud->size()));
      // count[0] : numbers of points chosen from the wall at right angles
      // count[1] : numbers of points chosen from the parallel wall
      std::vector<int> counts(2, 0);
      for (const auto & extracted_point : *extracted_cloud) {
        const auto pred = [&extracted_point](const PointXYZIL & p) {
          return (p.x == extracted_point.x) && (p.y == extracted_point.y) &&
                 (p.z == extracted_point.z);
        };
        const size_t index = std::find_if(pc->begin(), pc->end(), pred) - pc->begin();
        ++counts[index / std::pow(wall_length, 2)];
      }
      const double ratio = static_cast<double>(counts[0]) / (counts[0] + counts[1]);
      EXPECT_GE(ratio, parameter.expected_ratio_min) << "Failed at seed #" << seed;
      EXPECT_LE(ratio, parameter.expected_ratio_max) << "Failed at seed #" << seed;
    }
  }

  PointCloudSamplerWithNormal<PointXYZIL> sampler(
    std::make_shared<PointCloudSamplerWithNormalParameters>());
  sampler.setParticleStatistics(mean, cov_matrix);
  pcl::PointCloud<PointXYZIL>::Ptr invalid_cloud(new pcl::PointCloud<PointXYZIL>());
  // Empty cloud
  EXPECT_EQ(0u, sampler.sample(invalid_cloud, 100)->size());
  // Point cloud size is smaller than requested sample number
  invalid_cloud->push_back(PointXYZIL());
  EXPECT_EQ(1u, sampler.sample(invalid_cloud, 100)->size());
}

// The map-normal path: the sampler is handed the map and a normal per map
// point instead of estimating them from the scan, and the measurement models
// name a subset of one cloud instead of handing over a copy of it.
//
// The assertion is the weighting itself, on the same two walls as above. It is
// what goes when the lookups come back empty and every normal is a NaN, which
// is what a search radius measured in the wrong metric does -- and the draw
// still returns the right number of points while that happens, so counting
// them would not notice.
TEST(PointCloudSamplerWithNormal, MapNormals)
{
  const int wall_length = 30;
  pcl::PointCloud<PointXYZIL>::Ptr pc(new pcl::PointCloud<PointXYZIL>());
  buildWall(pc, 0, wall_length);
  buildWall(pc, M_PI / 2, wall_length);

  const double robot_yaw = M_PI / 6;
  const State6DOF mean(
    mcl_3dl::Vec3(3.5, -5.0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), robot_yaw));
  const std::vector<State6DOF> cov_matrix = buildPoseCovarianceMatrix(robot_yaw, 1.0, 0.2);

  // The map is those same surfaces, where the robot thinks they are.
  pcl::PointCloud<PointXYZIL>::Ptr map(new pcl::PointCloud<PointXYZIL>());
  *map = *pc;
  mean.transform(*map);

  pcl::PointCloud<pcl::Normal>::Ptr map_normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<PointXYZIL, pcl::Normal> ne;
  pcl::search::KdTree<PointXYZIL>::Ptr map_tree(new pcl::search::KdTree<PointXYZIL>());
  ne.setInputCloud(map);
  ne.setSearchMethod(map_tree);
  ne.setRadiusSearch(0.4);
  ne.compute(*map_normals);

  std::vector<size_t> indices(pc->size());
  std::iota(indices.begin(), indices.end(), 0);

  struct ParameterSet
  {
    double perform_weighting_ratio;
    double max_weight_ratio;
    double max_weight;
    double expected_ratio_min;
    double expected_ratio_max;
  };
  const std::vector<ParameterSet> parameters = {
    // Weighted: the wall at right angles to the uncertainty is worth 10 of the
    // parallel one, so the draw comes almost entirely from it.
    {2.0, 4.0, 10.0, 0.85, 1.0},
    // Between the two, where the weight is interpolated rather than clamped.
    // The band the robot actually runs in, and the one a weight of 1.0 would
    // pass unnoticed through the other two.
    {2.0, 8.0, 5.0, 0.65, 0.85},
    // The nearest setting that must not change: both walls worth 1, so the
    // draw is even. An all-NaN lookup lands here whatever the weights say.
    {6.0, 7.0, 10.0, 0.4, 0.6},
  };

  for (const unsigned int seed : {12345u, 45678u}) {
    auto sampler_params = std::make_shared<PointCloudSamplerWithNormalParameters>();
    sampler_params->normal_search_range_ = 0.4;
    PointCloudSamplerWithNormal<PointXYZIL> sampler(sampler_params, seed);
    sampler.setMap(map_tree, map_normals);
    sampler.setParticleStatistics(mean, cov_matrix);
    sampler.setInputCloud(pc);

    const size_t sample_num = 100;
    for (const ParameterSet & parameter : parameters) {
      sampler_params->perform_weighting_ratio_ = parameter.perform_weighting_ratio;
      sampler_params->max_weight_ratio_ = parameter.max_weight_ratio;
      sampler_params->max_weight_ = parameter.max_weight;
      sampler.refreshParameters();

      const pcl::PointCloud<PointXYZIL>::Ptr sampled = sampler.sample(pc, indices, sample_num);
      ASSERT_EQ(sample_num, sampled->size());

      std::vector<int> counts(2, 0);
      for (const auto & point : *sampled) {
        const auto pred = [&point](const PointXYZIL & p) {
          return (p.x == point.x) && (p.y == point.y) && (p.z == point.z);
        };
        const size_t index = std::find_if(pc->begin(), pc->end(), pred) - pc->begin();
        ASSERT_LT(index, pc->size()) << "sampled a point that is not in the named subset";
        ++counts[index / (wall_length * wall_length)];
      }
      const double ratio = static_cast<double>(counts[0]) / (counts[0] + counts[1]);
      EXPECT_GE(ratio, parameter.expected_ratio_min) << "Failed at seed #" << seed;
      EXPECT_LE(ratio, parameter.expected_ratio_max) << "Failed at seed #" << seed;
    }
  }
}

// pool_ratio 1 asks for a pool exactly `num` long, and a draw that takes each
// point once would then have to take all of them: the weights would decide
// nothing and the draw would come out uniform, while still paying for a map
// query per point. The sampler holds the ratio to the least that leaves a
// choice, so a 1 still weights -- weaker than at the default, because the pool
// is smaller, but nowhere near the even draw it would be without the bound.
TEST(PointCloudSamplerWithNormal, PoolRatioTooSmallToWeigh)
{
  const int wall_length = 30;
  pcl::PointCloud<PointXYZIL>::Ptr pc(new pcl::PointCloud<PointXYZIL>());
  buildWall(pc, 0, wall_length);
  buildWall(pc, M_PI / 2, wall_length);

  const double robot_yaw = M_PI / 6;
  const State6DOF mean(
    mcl_3dl::Vec3(3.5, -5.0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), robot_yaw));
  const std::vector<State6DOF> cov_matrix = buildPoseCovarianceMatrix(robot_yaw, 1.0, 0.2);

  pcl::PointCloud<PointXYZIL>::Ptr map(new pcl::PointCloud<PointXYZIL>());
  *map = *pc;
  mean.transform(*map);
  pcl::PointCloud<pcl::Normal>::Ptr map_normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<PointXYZIL, pcl::Normal> ne;
  pcl::search::KdTree<PointXYZIL>::Ptr map_tree(new pcl::search::KdTree<PointXYZIL>());
  ne.setInputCloud(map);
  ne.setSearchMethod(map_tree);
  ne.setRadiusSearch(0.4);
  ne.compute(*map_normals);

  std::vector<size_t> indices(pc->size());
  std::iota(indices.begin(), indices.end(), 0);

  for (const unsigned int seed : {12345u, 45678u}) {
    auto params = std::make_shared<PointCloudSamplerWithNormalParameters>();
    params->normal_search_range_ = 0.4;
    params->perform_weighting_ratio_ = 2.0;
    params->max_weight_ratio_ = 4.0;
    params->max_weight_ = 10.0;
    params->pool_ratio_ = 1;
    PointCloudSamplerWithNormal<PointXYZIL> sampler(params, seed);
    sampler.setMap(map_tree, map_normals);
    sampler.setParticleStatistics(mean, cov_matrix);
    sampler.setInputCloud(pc);

    const size_t sample_num = 100;
    const pcl::PointCloud<PointXYZIL>::Ptr sampled = sampler.sample(pc, indices, sample_num);
    ASSERT_EQ(sample_num, sampled->size());

    std::vector<int> counts(2, 0);
    for (const auto & point : *sampled) {
      const auto pred = [&point](const PointXYZIL & p) {
        return (p.x == point.x) && (p.y == point.y) && (p.z == point.z);
      };
      const size_t index = std::find_if(pc->begin(), pc->end(), pred) - pc->begin();
      ++counts[index / (wall_length * wall_length)];
    }
    const double ratio = static_cast<double>(counts[0]) / (counts[0] + counts[1]);
    // Weaker than at the default ratio, because the pool is smaller -- but far
    // from the 0.5 an unweighted draw gives.
    EXPECT_GE(ratio, 0.7) << "Failed at seed #" << seed;
  }
}

// What the draw does with nothing to draw from, or nothing to draw. A cycle
// whose clip leaves no points, and one that asks for none, both happen; both
// have to come back empty rather than reach the weighted draw.
TEST(PointCloudSamplerWithNormal, EmptyIndicesAndZeroCount)
{
  pcl::PointCloud<PointXYZIL>::Ptr pc(new pcl::PointCloud<PointXYZIL>());
  buildWall(pc, 0, 10);
  const State6DOF mean(mcl_3dl::Vec3(0, 0, 0), mcl_3dl::Quat(mcl_3dl::Vec3(0, 0, 1), 0.0));

  auto params = std::make_shared<PointCloudSamplerWithNormalParameters>();
  PointCloudSamplerWithNormal<PointXYZIL> sampler(params, 12345u);
  sampler.setParticleStatistics(mean, buildPoseCovarianceMatrix(0.0, 1.0, 0.2));
  sampler.setInputCloud(pc);

  std::vector<size_t> indices(pc->size());
  std::iota(indices.begin(), indices.end(), 0);

  EXPECT_EQ(0u, sampler.sample(pc, std::vector<size_t>(), 100)->size());
  EXPECT_EQ(0u, sampler.sample(pc, indices, 0)->size());
  // The nearest case that must still come back full.
  EXPECT_EQ(1u, sampler.sample(pc, indices, 1)->size());
}
}  // namespace test
}  // namespace mcl_3dl

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
