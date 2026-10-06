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

#ifndef MCL_3DL__POINT_CLOUD_RANDOM_SAMPLERS__POINT_CLOUD_SAMPLER_WITH_NORMAL_H_
#define MCL_3DL__POINT_CLOUD_RANDOM_SAMPLERS__POINT_CLOUD_SAMPLER_WITH_NORMAL_H_

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "mcl_3dl/parameters.h"
#include "mcl_3dl/point_cloud_random_sampler.h"
#include "mcl_3dl/state_6dof.h"
#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "rclcpp/rclcpp.hpp"

namespace mcl_3dl
{
template <class POINT_TYPE>
class PointCloudSamplerWithNormal : public PointCloudRandomSampler<POINT_TYPE>
{
  /// Below this the draw has no choice left to make; see sample().
  static constexpr size_t kMinPoolRatio = 2;

private:
  using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

  std::shared_ptr<std::default_random_engine> engine_;
  std::shared_ptr<PointCloudSamplerWithNormalParameters> params_;
  State6DOF mean_;
  Matrix eigen_vectors_;
  Vector eigen_values_;

public:
  PointCloudSamplerWithNormal(
    const std::shared_ptr<PointCloudSamplerWithNormalParameters> & params,
    const unsigned int random_seed = std::random_device()())
  : engine_(std::make_shared<std::default_random_engine>(random_seed))
  {
    params_ = params ? params : std::make_shared<PointCloudSamplerWithNormalParameters>();
  }

  void setParticleStatistics(
    const State6DOF & mean, const std::vector<State6DOF> & covariances) final
  {
    mean_ = mean;
    Matrix pos_cov(3, 3);
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        pos_cov(i, j) = std::abs(covariances[i][j]);
      }
    }
    const Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(pos_cov);
    eigen_vectors_ = eigen_solver.eigenvectors();
    eigen_values_ = eigen_solver.eigenvalues();
  }

private:
  bool mapNormalsUsable() const
  {
    return map_tree_ && map_normals_ && !map_normals_->points.empty();
  }

  /// A uniform draw of `want` of `indices`, or all of them when there are
  /// fewer. Without replacement, so the weighted draw that follows sees each
  /// point once.
  const std::vector<size_t> & drawPool(const std::vector<size_t> & indices, const size_t want) const
  {
    if (indices.size() <= want) return indices;
    pool_.assign(indices.begin(), indices.end());
    for (size_t i = 0; i < want; i++) {
      std::uniform_int_distribution<size_t> ud(i, pool_.size() - 1);
      std::swap(pool_[i], pool_[ud(*engine_)]);
    }
    pool_.resize(want);
    return pool_;
  }

  /// Every named point's normal, taken from the nearest map point.
  ///
  /// The cloud is in the robot's frame and the map is not, so each point is put
  /// through the particle mean first -- near enough for choosing which surface
  /// a point sits on, which is all the weighting needs. A point with no map
  /// point near it keeps a NaN normal and is weighted as if flat, the same as
  /// an estimate that failed.
  ///
  /// The tree is the plain one the normals were estimated with, not the one the
  /// measurement models match against: that one carries dist_weight_*, which
  /// would shrink this radius per axis. At the default dist_weight_z of 5 a
  /// 0.4 m range reaches 0.08 m vertically, and a z error past that turns every
  /// normal into a NaN -- the weighting would go quietly uniform exactly when
  /// the estimate is off and it is needed most.
  void lookUpNormalsInMap(
    const typename pcl::PointCloud<POINT_TYPE>::ConstPtr & pc,
    const std::vector<size_t> & indices) const
  {
    const float search_radius = static_cast<float>(params_->normal_search_range_);
    std::vector<int> id(1);
    std::vector<float> dist_sq(1);
    normals_->points.resize(indices.size());
    normals_->width = indices.size();
    normals_->height = 1;
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    normals_indexed_by_pool_ = true;
    for (size_t i = 0; i < indices.size(); i++) {
      const auto & src = pc->points[indices[i]];
      const Vec3 global = mean_.rot_ * Vec3(src.x, src.y, src.z) + mean_.pos_;
      POINT_TYPE query = src;
      query.x = global.x_;
      query.y = global.y_;
      query.z = global.z_;
      auto & out = normals_->points[i];
      if (
        map_tree_->radiusSearch(query, search_radius, id, dist_sq, 1) > 0 && !id.empty() &&
        static_cast<size_t>(id[0]) < map_normals_->points.size()) {
        // Back into the robot's frame: the weighting takes these against the
        // particle spread expressed there, and a normal read off the map is in
        // the map's.
        const auto & n = map_normals_->points[id[0]];
        const Vec3 local = mean_.rot_.inv() * Vec3(n.normal_x, n.normal_y, n.normal_z);
        out.normal_x = local.x_;
        out.normal_y = local.y_;
        out.normal_z = local.z_;
      } else {
        out.normal_x = out.normal_y = out.normal_z = nan;
      }
    }
  }

  /// The weight of each named point, accumulated so a draw is a binary search.
  ///
  /// A point whose surface faces the direction the particles are least sure
  /// about tells the filter more, so it is weighted up -- by how much depends
  /// on how elongated that uncertainty is.
  std::vector<double> accumulateWeights(const std::vector<size_t> & indices) const
  {
    const double eigen_value_ratio = std::sqrt(eigen_values_[2] / eigen_values_[1]);
    double max_weight = 1.0;
    if (eigen_value_ratio < params_->perform_weighting_ratio_) {
      max_weight = 1.0;
    } else if (eigen_value_ratio > params_->max_weight_ratio_) {
      max_weight = params_->max_weight_;
    } else {
      const double weight_ratio = (eigen_value_ratio - params_->perform_weighting_ratio_) /
                                  (params_->max_weight_ratio_ - params_->perform_weighting_ratio_);
      max_weight = 1.0 + (params_->max_weight_ - 1.0) * weight_ratio;
    }
    const mcl_3dl::Vec3 fpc_global(
      eigen_vectors_(0, 2), eigen_vectors_(1, 2), eigen_vectors_(2, 2));
    const mcl_3dl::Vec3 fpc_local = mean_.rot_.inv() * fpc_global;

    std::vector<double> cumulative_weight(indices.size(), 0.0);
    for (size_t i = 0; i < indices.size(); i++) {
      double weight = 1.0;
      const auto & normal = normals_->points[normals_indexed_by_pool_ ? i : indices[i]];
      if (
        !std::isnan(normal.normal_x) && !std::isnan(normal.normal_y) &&
        !std::isnan(normal.normal_z)) {
        double acos_angle = std::abs(
          normal.normal_x * fpc_local.x_ + normal.normal_y * fpc_local.y_ +
          normal.normal_z * fpc_local.z_);
        if (acos_angle > 1.0) acos_angle = 1.0;
        weight = 1.0 + (max_weight - 1.0) * ((M_PI / 2 - std::acos(acos_angle)) / (M_PI / 2));
      }
      cumulative_weight[i] = weight + ((i == 0) ? 0.0 : cumulative_weight[i - 1]);
    }
    return cumulative_weight;
  }

  mutable std::vector<size_t> pool_;
  /// Whether normals_ runs parallel to the pool (map lookups) or to the whole
  /// cloud (estimated from the scan).
  mutable bool normals_indexed_by_pool_{false};
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  typename pcl::PointCloud<POINT_TYPE>::ConstPtr normals_for_;
  typename pcl::search::KdTree<POINT_TYPE>::Ptr map_tree_;
  pcl::PointCloud<pcl::Normal>::ConstPtr map_normals_;

public:
  /// The map and a normal for each of its points, worked out once when the map
  /// arrives. A scan point lies on a map surface, so the surface it lies on is
  /// what the map says there -- and asking the map costs one query into a tree
  /// that already exists, where asking the scan costs building a tree per
  /// cycle. Without this the normals are estimated from the scan as before.
  void setMap(
    const typename pcl::search::KdTree<POINT_TYPE>::Ptr & tree,
    const pcl::PointCloud<pcl::Normal>::ConstPtr & normals)
  {
    map_tree_ = tree;
    map_normals_ = normals;
  }

  /// Build the kd-tree and the normals for this cycle's cloud, once.
  ///
  /// Both measurement models name subsets of it, and normals estimated here see
  /// the neighbours a model's own clip would have cut away -- so a point at the
  /// edge of a clip gets the normal its surface actually has.
  void setInputCloud(const typename pcl::PointCloud<POINT_TYPE>::ConstPtr & pc) final
  {
    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    normals_for_ = pc;
    normals_indexed_by_pool_ = false;
    if (!pc || pc->points.empty()) return;

    // With a map there is nothing to prepare: a normal is one query, and only
    // the points that reach the draw need one.
    if (mapNormalsUsable()) return;

    const auto start = std::chrono::steady_clock::now();
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;
    typename pcl::search::KdTree<POINT_TYPE>::Ptr tree(new pcl::search::KdTree<POINT_TYPE>());
    ne.setInputCloud(pc);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(params_->normal_search_range_);
    ne.compute(*normals_);
    RCLCPP_DEBUG(
      rclcpp::get_logger("mcl_3dl"),
      "PointCloudSamplerWithNormal::setInputCloud() %lu points, %f[s]", pc->points.size(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  }

  /// Draw from the named subset, weighting by the normals worked out in
  /// setInputCloud(). Falls back to computing them here when it was not called
  /// or was called with another cloud, so the sampler still works standalone.
  typename pcl::PointCloud<POINT_TYPE>::Ptr sample(
    const typename pcl::PointCloud<POINT_TYPE>::ConstPtr & pc, const std::vector<size_t> & indices,
    const size_t num) const final
  {
    const bool from_map = mapNormalsUsable() && normals_for_ == pc;
    if (
      !from_map &&
      (normals_for_ != pc || !normals_ || normals_->points.size() != pc->points.size())) {
      return PointCloudRandomSampler<POINT_TYPE>::sample(pc, indices, num);
    }
    typename pcl::PointCloud<POINT_TYPE>::Ptr output(new pcl::PointCloud<POINT_TYPE>);
    output->header = pc->header;
    if (indices.empty() || num == 0) return output;
    if (indices.size() <= num) {
      output->points.reserve(indices.size());
      for (const size_t i : indices) output->push_back(pc->points[i]);
      output->width = 1;
      output->height = output->points.size();
      return output;
    }
    // Weighing every point means a map query for every point, and the draw
    // keeps `num` of them. Drawing a pool first and weighing that costs
    // pool_ratio queries per kept point instead of one per point in the cloud.
    //
    // A ratio of one is not a weaker weighting, it is none: the pool would
    // hold exactly `num` and the draw, which takes each point once, would have
    // to take all of them whatever the weights said. Two is the least that
    // leaves the weights anything to decide. Turning the weighting off is
    // use_random_sampler_with_normal.
    const std::vector<size_t> & pool =
      from_map ? drawPool(indices, num * std::max<size_t>(kMinPoolRatio, params_->pool_ratio_))
               : indices;
    if (from_map) lookUpNormalsInMap(pc, pool);
    const std::vector<double> cumulative_weight = accumulateWeights(pool);
    std::uniform_real_distribution<double> ud(0, cumulative_weight.back());
    std::unordered_set<size_t> selected;
    while (selected.size() < num) {
      const auto it =
        std::lower_bound(cumulative_weight.begin(), cumulative_weight.end(), ud(*engine_));
      selected.insert(static_cast<size_t>(it - cumulative_weight.begin()));
    }
    output->points.reserve(num);
    for (const size_t i : selected) output->push_back(pc->points[pool[i]]);
    output->width = 1;
    output->height = output->points.size();
    return output;
  }

  typename pcl::PointCloud<POINT_TYPE>::Ptr sample(
    const typename pcl::PointCloud<POINT_TYPE>::ConstPtr & pc, const size_t num) const final
  {
    const auto start_timestamp = std::chrono::steady_clock::now();

    typename pcl::PointCloud<POINT_TYPE>::Ptr output(new pcl::PointCloud<POINT_TYPE>);
    output->header = pc->header;

    if ((pc->points.size() == 0) || (num == 0)) {
      return output;
    }
    if (pc->size() <= num) {
      *output = *pc;
      return output;
    }

    const double eigen_value_ratio = std::sqrt(eigen_values_[2] / eigen_values_[1]);

    double max_weight = 1.0;
    if (eigen_value_ratio < params_->perform_weighting_ratio_) {
      max_weight = 1.0;
    } else if (eigen_value_ratio > params_->max_weight_ratio_) {
      max_weight = params_->max_weight_;
    } else {
      const double weight_ratio = (eigen_value_ratio - params_->perform_weighting_ratio_) /
                                  (params_->max_weight_ratio_ - params_->perform_weighting_ratio_);
      max_weight = 1.0 + (params_->max_weight_ - 1.0) * weight_ratio;
    }
    const mcl_3dl::Vec3 fpc_global(
      eigen_vectors_(0, 2), eigen_vectors_(1, 2), eigen_vectors_(2, 2));
    const mcl_3dl::Vec3 fpc_local = mean_.rot_.inv() * fpc_global;
    pcl::NormalEstimation<POINT_TYPE, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    typename pcl::search::KdTree<POINT_TYPE>::Ptr tree(new pcl::search::KdTree<POINT_TYPE>());
    ne.setInputCloud(pc);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(params_->normal_search_range_);
    ne.compute(*cloud_normals);

    const auto compute_normal_timestamp = std::chrono::steady_clock::now();
    std::vector<double> cumulative_weight(cloud_normals->points.size(), 0.0);
    for (size_t i = 0; i < cloud_normals->points.size(); i++) {
      double weight = 1.0;
      const auto & normal = cloud_normals->points[i];
      if (
        !std::isnan(normal.normal_x) && !std::isnan(normal.normal_y) &&
        !std::isnan(normal.normal_z)) {
        double acos_angle = std::abs(
          normal.normal_x * fpc_local.x_ + normal.normal_y * fpc_local.y_ +
          normal.normal_z * fpc_local.z_);
        // Avoid that std::acos() returns nan because of calculation errors
        if (acos_angle > 1.0) {
          acos_angle = 1.0;
        }
        const double angle = std::acos(acos_angle);
        weight = 1.0 + (max_weight - 1.0) * ((M_PI / 2 - angle) / (M_PI / 2));
      }
      cumulative_weight[i] = weight + ((i == 0) ? 0.0 : cumulative_weight[i - 1]);
    }
    std::uniform_real_distribution<double> ud(0, cumulative_weight.back());
    // Use unordered_set to avoid duplication
    std::unordered_set<size_t> selected_ids;
    while (true) {
      const double random_value = ud(*engine_);
      auto it = std::lower_bound(cumulative_weight.begin(), cumulative_weight.end(), random_value);
      const size_t n = it - cumulative_weight.begin();
      selected_ids.insert(n);
      if (selected_ids.size() >= num) {
        break;
      }
    }
    output->points.reserve(num);
    for (const auto & index : selected_ids) {
      output->push_back(pc->points[index]);
    }

    const auto final_timestamp = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(
      rclcpp::get_logger("mcl_3dl"),
      "PointCloudSamplerWithNormal::sample() computation time: %f[s] (Normal calculation: %f[s])",
      std::chrono::duration<double>(final_timestamp - start_timestamp).count(),
      std::chrono::duration<double>(compute_normal_timestamp - start_timestamp).count());
    RCLCPP_DEBUG(
      rclcpp::get_logger("mcl_3dl"), "Chosen eigen vector: (%f, %f, %f), max weight: %f",
      eigen_vectors_(0, 2), eigen_vectors_(1, 2), eigen_vectors_(2, 2), max_weight);
    return output;
  }
};

}  // namespace mcl_3dl

#endif  // MCL_3DL__POINT_CLOUD_RANDOM_SAMPLERS__POINT_CLOUD_SAMPLER_WITH_NORMAL_H_
