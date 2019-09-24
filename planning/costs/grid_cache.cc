#include "planning/costs/grid_cache.hh"

#include "util/eigen_clip.hh"

// TODO
#include <iostream>

namespace planning {

GridCache2d::GridCache2d(const CachedFunction& fnc, const GridCache2dConfig& cfg)
    : cfg_(cfg) {
  blocks_.resize(cfg_.edge_length_voxels * cfg_.edge_length_voxels, 0.0);
  center_grid_voxels_ =
      jcc::Vec2i(cfg_.edge_length_voxels / 2, cfg_.edge_length_voxels / 2);

  double max = 0.0;
  for (int i = 0; i < cfg_.edge_length_voxels; ++i) {
    for (int j = 0; j < cfg_.edge_length_voxels; ++j) {
      const jcc::Vec2i pos_grid_voxels(i, j);
      const jcc::Vec2 pos_world_voxels =
          (pos_grid_voxels - center_grid_voxels_).cast<double>();
      const jcc::Vec2 pos_world_m = pos_world_voxels * cfg_.meters_per_voxel;

      const double value = fnc(pos_world_m);
      blocks_[i + (cfg_.edge_length_voxels * j)] = value;

      max = std::max(max, value);
    }
  }
}

double GridCache2d::direct_get(const jcc::Vec2i& p) const {
  return blocks_[p.x() + (cfg_.edge_length_voxels * p.y())];
}

double GridCache2d::get(const jcc::Vec2& query_world_m) const {
  const jcc::Vec2 lower_bound(0, 0);
  const jcc::Vec2 upper_bound(cfg_.edge_length_voxels - 1, cfg_.edge_length_voxels - 1);

  const jcc::Vec2 query_world_voxels = query_world_m / cfg_.meters_per_voxel;
  const jcc::Vec2 query_grid_voxels =
      (query_world_voxels + center_grid_voxels_.cast<double>());

  // TODO : Allow sampling outside of the space
  const jcc::Vec2 sample_pt =
      jcc::eigen_clip(query_grid_voxels, lower_bound, upper_bound);

  const jcc::Vec2i q22 = sample_pt.array().ceil().cast<int>();
  const jcc::Vec2i q11 = sample_pt.array().floor().cast<int>();

  const jcc::Vec2i q12(q11.x(), q22.y());
  const jcc::Vec2i q21(q22.x(), q11.y());

  const double f11 = direct_get(q11);
  const double f21 = direct_get(q21);
  const double f12 = direct_get(q12);
  const double f22 = direct_get(q22);

  const MatNd<2, 2> Q = (MatNd<2, 2>() << f11, f12, f21, f22).finished();

  const jcc::Vec2 x_query(q22.x() - sample_pt.x(), sample_pt.x() - q11.x());
  const jcc::Vec2 y_query(q22.y() - sample_pt.y(), sample_pt.y() - q11.y());
  const double interpolated = x_query.dot(Q * y_query);
  return interpolated;
}

}  // namespace planning
