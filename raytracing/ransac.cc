#include "raytracing/ransac.hh"

#include "spatial_geometry/perp.hh"

namespace raytrace {
using Vec2 = Eigen::Vector2d;

// DirectedLineSegment: model
bool fit_model(const std::vector<Observation>&       observations,
               const std::vector<int>&               candidate_indices,
               Out<std::vector<DirectedLineSegment>> hypotheses) {
  if (candidate_indices.size() != 2) {
    // TODO: shit the bed
    return false;
  }

  const Observation& a = observations[candidate_indices[0]];
  const Observation& b = observations[candidate_indices[1]];

  const Vec2 center = (a.hit + b.hit) / 2.0;
  Vec2       normal = geometry::perp((a.hit - center).eval());

  // todo: determine incompatible observations
  const Vec2 direction = a.hit - a.origin;
  if (normal.dot(direction) > 0.0) {
    normal = -normal;
  }

  hypotheses->push_back(DirectedLineSegment(center, normal));

  return true;
}

// DirectedLineSegment: model
void find_inliers(const DirectedLineSegment&      line,
                  const std::vector<Observation>& points,
                  Out<std::vector<int>>           inlier_indices) {
}
}