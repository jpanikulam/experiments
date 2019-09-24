#pragma once

#include "eigen.hh"

#include <functional>
#include <map>

namespace planning {

struct GridCache2dConfig {
  double meters_per_voxel = 0.25;
  int edge_length_voxels = 256;
};

class GridCache2d {
 public:
  using CachedFunction = std::function<double(const jcc::Vec2& x)>;

  GridCache2d() = default;
  GridCache2d(const CachedFunction& fnc, const GridCache2dConfig& cfg = {});
  double get(const jcc::Vec2& x) const;

  double operator()(const jcc::Vec2& x) const {
    return get(x);
  }

 private:
  double direct_get(const jcc::Vec2i& p) const;

  std::vector<double> blocks_;

  jcc::Vec2i center_grid_voxels_;

  // std::map<uint64_t, double> vals_;
  GridCache2dConfig cfg_;
};

}  // namespace planning
