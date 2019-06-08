#include "geometry/intersection/ray_closest_approach.hh"

namespace geometry {
namespace {

using Vec3 = Eigen::Vector3d;

struct ApproachParameters {
  double along_a;
  double along_b;
};

ApproachParameters ray_ray_closest_approach(const Ray& a, const Ray& b) {
  const Vec3 c = b.origin - a.origin;

  const double p = a.direction.dot(b.direction);
  const double q = a.direction.dot(c);
  const double r = b.direction.dot(c);
  const double s = a.direction.dot(a.direction);
  const double t = b.direction.dot(b.direction);

  const double p_sqr = p * p;
  const double st_min_p2 = (s * t) - p_sqr;
  const double inv_st_min_p2 = 1.0 / st_min_p2;

  const double d = ((-p * r) + (q * t)) * inv_st_min_p2;
  const double e = ((p * q) - (r * s)) * inv_st_min_p2;

  const ApproachParameters result({.along_a = d, .along_b = e});
  return result;
}
}  // namespace

jcc::Optional<LineApproachParameters> line_ray_closest_approach(
    const Ray& a, const shapes::LineSegment& b) {
  const jcc::Vec3 b_dir = (b.point_b - b.point_a);
  const double b_length = b_dir.norm();
  const Ray ray_b = Ray{b.point_a, b_dir / b_length};
  const auto approach_params = ray_ray_closest_approach(a, ray_b);

  if (approach_params.along_a < 0.0) {
    return {};
  }

  jcc::Vec3 pt_on_line;
  if (approach_params.along_b < 0.0) {
    pt_on_line = b.point_a;
  } else if (approach_params.along_b > b_length) {
    pt_on_line = b.point_b;
  } else {
    pt_on_line = ray_b(approach_params.along_b);
  }

  const double squared_distance = (a(approach_params.along_a) - pt_on_line).squaredNorm();

  // Computing this here, so that one day we can make this call faster

  const LineApproachParameters result{
      .along_ray = approach_params.along_a,  //
      .squared_distance = squared_distance,  //
      .on_line = pt_on_line                  //
  };
  return {result};
}

}  // namespace geometry