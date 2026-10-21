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

#ifndef MCL_3DL__NEAREST_POINT_GRID_H_
#define MCL_3DL__NEAREST_POINT_GRID_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "pcl/point_cloud.h"

namespace mcl_3dl
{

/// The distance from a point to the nearest map point, when that is all that is
/// wanted.
///
/// The likelihood model asks a kd-tree for one neighbour of every sampled scan
/// point, once per particle, and uses only the distance: the index is thrown
/// away and the value is clipped at match_dist_min. A tree descent answers a
/// harder question than that. Here the map points are bucketed into cells, and
/// a query walks outward one shell of cells at a time until the shell it would
/// read next is already farther off than the best it has found. Cells of about
/// half the match radius keep that to a shell or two.
///
/// A scan point that hits nothing on the map -- a passer-by, an unmapped
/// corner -- would otherwise be the worst case rather than the best, since
/// nothing stops the walk before the last shell. So each cell also stores how
/// many shells away the nearest occupied cell is. The walk starts there, and a
/// cell with no occupied cell in reach is a miss without reading anything.
///
/// The distances are measured against the map points themselves, so the cells
/// only decide which points are worth measuring against: the answer is the
/// exact nearest distance, not a quantised one, which a test against brute
/// force covers. It is not quite the chunked kd-tree's answer, which is an
/// approximate search (setEpsilon) over chunks that overlap in unweighted
/// metres -- the grid is the stricter of the two, so a point near the cut-off
/// can match here where the tree let it go.
///
/// Distances follow the same per-axis weights as the kd-tree's point
/// representation (dist_weight_*), which the caller passes to
/// setRescaleValues(). Coordinates are stored pre-scaled so that a plain
/// Euclidean distance in stored space is the weighted distance, and so that the
/// shell bound stays exact whatever the weights are.
template <class POINT_TYPE>
class NearestPointGrid
{
public:
  using Ptr = std::shared_ptr<NearestPointGrid>;

  NearestPointGrid(const float cell_size, const float max_dist)
  : cell_size_(cell_size),
    max_dist_(max_dist),
    // One shell further out and every cell in it would start at max_dist or
    // beyond, by the bound nearestSquaredDistance() uses; so this is the last
    // one that can hold anything nearer.
    max_ring_(static_cast<int>(std::ceil(max_dist / cell_size)))
  {
  }

  /// Per-axis weights, in the order and meaning of
  /// pcl::PointRepresentation::setRescaleValues(): only the first three are
  /// read.
  void setRescaleValues(const float * w)
  {
    scale_[0] = w[0];
    scale_[1] = w[1];
    scale_[2] = w[2];
  }

  void setInputCloud(const typename pcl::PointCloud<POINT_TYPE>::ConstPtr & cloud)
  {
    blocks_.clear();
    pts_.clear();
    next_.clear();
    if (!cloud) return;

    pts_.reserve(cloud->points.size() * 3);
    next_.reserve(cloud->points.size());
    std::vector<Cell> occupied;
    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const float x = p.x * scale_[0];
      const float y = p.y * scale_[1];
      const float z = p.z * scale_[2];
      const int32_t i = static_cast<int32_t>(next_.size());
      pts_.push_back(x);
      pts_.push_back(y);
      pts_.push_back(z);

      const int32_t cx = cellOf(x);
      const int32_t cy = cellOf(y);
      const int32_t cz = cellOf(z);
      Block & block = blockFor(cx, cy, cz);
      const size_t local = localIndex(cx, cy, cz);
      int32_t & head = block.head[local];
      if (head < 0) {
        block.ring[local] = 0;
        occupied.push_back(Cell{cx, cy, cz});
      }
      next_.push_back(head);
      head = i;
    }
    buildRings(occupied);
  }

  /// The squared weighted distance to the nearest map point, if one lies within
  /// max_dist.
  bool nearestSquaredDistance(const POINT_TYPE & p, float & out_sq) const
  {
    if (blocks_.empty()) return false;
    const float x = p.x * scale_[0];
    const float y = p.y * scale_[1];
    const float z = p.z * scale_[2];
    const int32_t cx = cellOf(x);
    const int32_t cy = cellOf(y);
    const int32_t cz = cellOf(z);

    const auto block = blocks_.find(blockIdOf(cx, cy, cz));
    if (block == blocks_.end()) return false;
    const int first_ring = block->second.ring[localIndex(cx, cy, cz)];
    if (first_ring > max_ring_) return false;

    BlockCache cache(this);
    float best = max_dist_ * max_dist_;
    bool found = false;
    for (int ring = first_ring; ring <= max_ring_; ring++) {
      // A cell this many shells out cannot hold anything nearer than this, and
      // nor can anything beyond it, so a better result already in hand ends the
      // walk. One shell is given away because the query point may sit anywhere
      // within its own cell.
      const float bound = (ring < 1) ? 0.0f : (ring - 1) * cell_size_;
      if (found && best <= bound * bound) break;
      if (visitRing(x, y, z, cx, cy, cz, ring, cache, best)) found = true;
    }
    if (!found) return false;
    out_sq = best;
    return true;
  }

private:
  // Cells are grouped into blocks, each a contiguous array, so that walking a
  // shell is array indexing rather than hashing. At the cell sizes this is
  // built with, the farthest shell a query reaches spans at most two blocks
  // per axis, which is what BlockCache is sized for.
  static constexpr int BLOCK_BITS = 4;
  static constexpr int BLOCK_SIDE = 1 << BLOCK_BITS;
  static constexpr size_t BLOCK_CELLS = BLOCK_SIDE * BLOCK_SIDE * BLOCK_SIDE;
  static constexpr uint8_t kUnreachable = 255;

  struct Cell
  {
    int32_t x, y, z;
  };
  struct BlockId
  {
    int32_t x, y, z;
    bool operator==(const BlockId & a) const { return x == a.x && y == a.y && z == a.z; }
  };
  struct BlockHash
  {
    size_t operator()(const BlockId & b) const
    {
      return (static_cast<size_t>(static_cast<uint32_t>(b.x)) * 73856093u) ^
             (static_cast<size_t>(static_cast<uint32_t>(b.y)) * 19349663u) ^
             (static_cast<size_t>(static_cast<uint32_t>(b.z)) * 83492791u);
    }
  };
  struct Block
  {
    // The first point in each cell, -1 for none; chained through next_.
    std::vector<int32_t> head;
    // Shells from each cell to the nearest occupied one, kUnreachable beyond.
    std::vector<uint8_t> ring;

    Block() : head(BLOCK_CELLS, -1), ring(BLOCK_CELLS, kUnreachable) {}
  };

  int32_t cellOf(const float v) const { return static_cast<int32_t>(std::floor(v / cell_size_)); }

  static BlockId blockIdOf(const int32_t cx, const int32_t cy, const int32_t cz)
  {
    return BlockId{cx >> BLOCK_BITS, cy >> BLOCK_BITS, cz >> BLOCK_BITS};
  }

  static size_t localIndex(const int32_t cx, const int32_t cy, const int32_t cz)
  {
    return ((static_cast<size_t>(cz & (BLOCK_SIDE - 1)) << BLOCK_BITS |
             static_cast<size_t>(cy & (BLOCK_SIDE - 1)))
            << BLOCK_BITS) |
           static_cast<size_t>(cx & (BLOCK_SIDE - 1));
  }

  Block & blockFor(const int32_t cx, const int32_t cy, const int32_t cz)
  {
    const BlockId id = blockIdOf(cx, cy, cz);
    auto it = blocks_.find(id);
    if (it == blocks_.end()) {
      it = blocks_.emplace(id, Block()).first;
    }
    return it->second;
  }

  /// How many shells each cell is from the nearest occupied one, up to the
  /// point where the answer stops mattering. Stepping to any of the 26
  /// neighbours moves exactly one shell, so a breadth-first walk out from the
  /// occupied cells gives the exact count.
  void buildRings(const std::vector<Cell> & occupied)
  {
    std::vector<Cell> frontier = occupied;
    std::vector<Cell> next;
    for (int ring = 1; ring <= max_ring_; ring++) {
      next.clear();
      for (const Cell & c : frontier) {
        for (int dz = -1; dz <= 1; dz++) {
          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              if (!dx && !dy && !dz) continue;
              const Cell n{c.x + dx, c.y + dy, c.z + dz};
              uint8_t & r = blockFor(n.x, n.y, n.z).ring[localIndex(n.x, n.y, n.z)];
              if (r <= ring) continue;
              r = static_cast<uint8_t>(ring);
              next.push_back(n);
            }
          }
        }
      }
      frontier.swap(next);
    }
  }

  /// The blocks one query touches. A block is normally wide enough that the
  /// query spans at most two per axis, so the low bit of each block coordinate
  /// tells them apart and a block costs a hash lookup only the first time the
  /// query reaches it. A cell size small enough against max_dist to break that
  /// just makes slots collide, which the stored id catches.
  class BlockCache
  {
  public:
    explicit BlockCache(const NearestPointGrid * grid) : grid_(grid) {}

    const Block * get(const BlockId & id)
    {
      Entry & e = entry_[((id.z & 1) << 2) | ((id.y & 1) << 1) | (id.x & 1)];
      if (!e.filled || !(e.id == id)) {
        const auto it = grid_->blocks_.find(id);
        e.block = (it == grid_->blocks_.end()) ? nullptr : &it->second;
        e.id = id;
        e.filled = true;
      }
      return e.block;
    }

  private:
    struct Entry
    {
      BlockId id{0, 0, 0};
      const Block * block = nullptr;
      bool filled = false;
    };
    const NearestPointGrid * grid_;
    Entry entry_[8];
  };

  /// The gap along one axis between the query and the cell `d` cells away on
  /// it, given where the query sits inside its own cell. Nothing in that cell
  /// is nearer than the root of the three gaps squared and summed, which is
  /// what the caller does with them.
  float axisGap(const float offset_in_cell, const int d) const
  {
    if (d > 0) return (d * cell_size_ - offset_in_cell);
    if (d < 0) return (offset_in_cell - (d + 1) * cell_size_);
    return 0.0f;
  }

  /// Measure against every point in the shell of cells at Chebyshev distance
  /// `ring` from the query cell, lowering `best`. Cells that cannot hold
  /// anything nearer than `best` are skipped: most of a shell lies outside the
  /// sphere the search has already narrowed to.
  bool visitRing(
    const float x, const float y, const float z, const int32_t cx, const int32_t cy,
    const int32_t cz, const int ring, BlockCache & cache, float & best) const
  {
    const float ox = x - cx * cell_size_;
    const float oy = y - cy * cell_size_;
    const float oz = z - cz * cell_size_;

    bool found = false;
    for (int dz = -ring; dz <= ring; dz++) {
      const float gap_z = axisGap(oz, dz);
      const float gap_z_sq = gap_z * gap_z;
      if (gap_z_sq >= best) continue;
      for (int dy = -ring; dy <= ring; dy++) {
        const float gap_y = axisGap(oy, dy);
        const float gap_zy_sq = gap_z_sq + gap_y * gap_y;
        if (gap_zy_sq >= best) continue;
        // The shell is hollow: unless this row is on a face of the cube, only
        // its two ends are new, the rest having been read by an earlier ring.
        const bool row_on_face = (std::abs(dz) == ring || std::abs(dy) == ring);
        for (int dx = -ring; dx <= ring; dx += (row_on_face ? 1 : (ring ? 2 * ring : 1))) {
          const float gap_x = axisGap(ox, dx);
          if (gap_zy_sq + gap_x * gap_x >= best) continue;

          const int32_t gx = cx + dx;
          const int32_t gy = cy + dy;
          const int32_t gz = cz + dz;
          const Block * block = cache.get(blockIdOf(gx, gy, gz));
          if (!block) continue;
          for (int32_t i = block->head[localIndex(gx, gy, gz)]; i >= 0; i = next_[i]) {
            const float * q = &pts_[static_cast<size_t>(i) * 3];
            const float ex = q[0] - x;
            const float ey = q[1] - y;
            const float ez = q[2] - z;
            const float d_sq = ex * ex + ey * ey + ez * ez;
            if (d_sq < best) {
              best = d_sq;
              found = true;
            }
          }
        }
      }
    }
    return found;
  }

  std::unordered_map<BlockId, Block, BlockHash> blocks_;
  std::vector<float> pts_;
  std::vector<int32_t> next_;
  float scale_[3] = {1.0f, 1.0f, 1.0f};
  float cell_size_;
  float max_dist_;
  int max_ring_;
};

}  // namespace mcl_3dl

#endif  // MCL_3DL__NEAREST_POINT_GRID_H_
