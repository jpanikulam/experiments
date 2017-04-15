#include "out.hh"
#include "raytracing/geometry.hh"

#include <Eigen/Dense>

#include <vector>

namespace raytrace {
struct RansacConfiguration {
  int inliers_for_hypothesis = 2;
};

//
// Non-general ransac procedures
//

struct Observation {
  using Vec2 = Eigen::Vector2d;

  Vec2 origin;
  Vec2 hit;
};

// DirectedLineSegment: model
bool fit_model(const std::vector<Observation>&       points,
               const std::vector<int>&               candidate_indices,
               Out<std::vector<DirectedLineSegment>> hypotheses);

// DirectedLineSegment: model
void find_inliers(const DirectedLineSegment&      line,
                  const std::vector<Observation>& points,
                  Out<std::vector<int>>           inlier_indices);
}