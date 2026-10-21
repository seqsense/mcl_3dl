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

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "mcl_3dl/nearest_point_grid.h"
#include "pcl/point_types.h"

namespace
{
constexpr float kMaxDist = 0.6;
constexpr float kCellSize = 0.1;
constexpr float kTolerance = 1e-4;

struct Weights
{
  float x, y, z;
};

pcl::PointXYZ makePoint(const float x, const float y, const float z)
{
  pcl::PointXYZ p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// The answer the grid is supposed to reproduce, computed the slow way, and
// without the cut-off so that the caller can tell a near miss from a far one.
float bruteForce(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointXYZ & q, const Weights & w)
{
  float best = std::numeric_limits<float>::max();
  for (const auto & p : cloud.points) {
    const float dx = (p.x - q.x) * w.x;
    const float dy = (p.y - q.y) * w.y;
    const float dz = (p.z - q.z) * w.z;
    best = std::min(best, dx * dx + dy * dy + dz * dz);
  }
  return best;
}

void expectAgreesWithBruteForce(const Weights & w)
{
  std::mt19937 rand(42);
  std::uniform_real_distribution<float> coord(-3.0, 3.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // A wall, a floor and some scattered points: surfaces where cells hold one
  // point each, and clusters where they hold several.
  for (int i = 0; i < 40; i++) {
    for (int j = 0; j < 40; j++) {
      cloud->points.push_back(makePoint(i * 0.1, j * 0.1, 0.0));
      cloud->points.push_back(makePoint(i * 0.1, 0.0, j * 0.1));
    }
  }
  for (int i = 0; i < 500; i++) {
    cloud->points.push_back(makePoint(coord(rand), coord(rand), coord(rand)));
  }

  mcl_3dl::NearestPointGrid<pcl::PointXYZ> grid(kCellSize, kMaxDist);
  const float rescale[4] = {w.x, w.y, w.z, 0.0};
  grid.setRescaleValues(rescale);
  grid.setInputCloud(cloud);

  std::uniform_real_distribution<float> query(-5.0, 5.0);
  int hits = 0;
  int misses = 0;
  for (int i = 0; i < 20000; i++) {
    const pcl::PointXYZ q = makePoint(query(rand), query(rand), query(rand));
    const float expected = std::sqrt(bruteForce(*cloud, q, w));
    // The grid scales each coordinate before subtracting, as PCL's point
    // representation does, so its rounding differs from the brute force's in
    // the last few digits. Right at the cut-off that can flip hit into miss.
    if (std::abs(expected - kMaxDist) < kTolerance) continue;

    float actual_sq = 0;
    const bool hit = grid.nearestSquaredDistance(q, actual_sq);
    ASSERT_EQ(expected < kMaxDist, hit) << "at (" << q.x << ", " << q.y << ", " << q.z << ")";
    if (!hit) {
      misses++;
      continue;
    }
    hits++;
    ASSERT_NEAR(expected, std::sqrt(actual_sq), kTolerance)
      << "at (" << q.x << ", " << q.y << ", " << q.z << ")";
  }
  // Both branches have to have been taken for the comparison to mean anything.
  ASSERT_GT(hits, 0);
  ASSERT_GT(misses, 0);
}
}  // namespace

TEST(NearestPointGrid, MatchesBruteForce) { expectAgreesWithBruteForce({1.0, 1.0, 1.0}); }

TEST(NearestPointGrid, MatchesBruteForceWithRescale)
{
  expectAgreesWithBruteForce({1.0, 1.0, 5.0});
}

// setInputCloud() is called again on every map update, so what it replaces has
// to go. A stale cell would keep matching scan points against an obstacle the
// map no longer has.
TEST(NearestPointGrid, ReplacesThePreviousMap)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr first(new pcl::PointCloud<pcl::PointXYZ>);
  first->points.push_back(makePoint(0.0, 0.0, 0.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr second(new pcl::PointCloud<pcl::PointXYZ>);
  second->points.push_back(makePoint(0.4, 0.0, 0.0));

  mcl_3dl::NearestPointGrid<pcl::PointXYZ> grid(kCellSize, kMaxDist);
  grid.setInputCloud(first);
  float d_sq = 0;
  ASSERT_TRUE(grid.nearestSquaredDistance(makePoint(0.0, 0.0, 0.0), d_sq));
  ASSERT_NEAR(0.0, std::sqrt(d_sq), kTolerance);

  grid.setInputCloud(second);
  ASSERT_TRUE(grid.nearestSquaredDistance(makePoint(0.0, 0.0, 0.0), d_sq));
  ASSERT_NEAR(0.4, std::sqrt(d_sq), kTolerance) << "the first map is still in the grid";
  // And the point the second map does have is found where it is.
  ASSERT_TRUE(grid.nearestSquaredDistance(makePoint(0.4, 0.0, 0.0), d_sq));
  ASSERT_NEAR(0.0, std::sqrt(d_sq), kTolerance);
}

TEST(NearestPointGrid, EmptyMap)
{
  mcl_3dl::NearestPointGrid<pcl::PointXYZ> grid(kCellSize, kMaxDist);
  float d_sq = 0;
  ASSERT_FALSE(grid.nearestSquaredDistance(makePoint(0, 0, 0), d_sq));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  grid.setInputCloud(cloud);
  ASSERT_FALSE(grid.nearestSquaredDistance(makePoint(0, 0, 0), d_sq));
}

TEST(NearestPointGrid, IgnoresNonFinitePoints)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.push_back(makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0, 0.0));
  cloud->points.push_back(makePoint(0.3, 0.0, 0.0));

  mcl_3dl::NearestPointGrid<pcl::PointXYZ> grid(kCellSize, kMaxDist);
  grid.setInputCloud(cloud);

  float d_sq = 0;
  ASSERT_TRUE(grid.nearestSquaredDistance(makePoint(0.0, 0.0, 0.0), d_sq));
  ASSERT_FLOAT_EQ(0.3 * 0.3, d_sq);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
