#include "estimation/calibration/nonlinear_camera_model.hh"

#include "numerics/gauss_newton.hh"

#include "eigen_helpers.hh"
#include "logging/assert.hh"

namespace estimation {
namespace {

jcc::Vec2 apply_distortion(const ProjectionCoefficients& proj,
                           const jcc::Vec3& world_point) {
  const jcc::Vec2 distorted_point = world_point.head<2>() / world_point.z();

  const double r2 = distorted_point.squaredNorm();
  const double x = distorted_point.x();
  const double y = distorted_point.y();
  const double xy = x * y;

  const double x_tan_offset = (2.0 * proj.p1 * xy) + (proj.p2 * (r2 + (2.0 * x * x)));
  const double y_tan_offset = (2.0 * proj.p2 * xy) + (proj.p1 * (r2 + (2.0 * y * y)));

  const double r4 = r2 * r2;
  const double r6 = r2 * r4;

  const double radial_distortion = (proj.k1 * r2) + (proj.k2 * r4) + (proj.k3 * r6);
  const double x_prime = x * (1.0 + radial_distortion) + x_tan_offset;
  const double y_prime = y * (1.0 + radial_distortion) + y_tan_offset;
  return jcc::Vec2(x_prime, y_prime);
}

jcc::Vec2 apply_projection(const ProjectionCoefficients& proj, const jcc::Vec2& p_prime) {
  const jcc::Vec2 focal_scaling = jcc::Vec2(proj.fx, proj.fy);
  const jcc::Vec2 principal_point = jcc::Vec2(proj.cx, proj.cy);
  const jcc::Vec2 projected = focal_scaling.cwiseProduct(p_prime) + principal_point;
  return projected;
}

jcc::Vec2 apply_projection_and_distortion(const ProjectionCoefficients& proj,
                                          const jcc::Vec3& world_point) {
  const jcc::Vec2 p_prime = apply_distortion(proj, world_point);
  const jcc::Vec2 projected = apply_projection(proj, p_prime);
  return projected;
}

}  // namespace

NonlinearCameraModel::NonlinearCameraModel(const ProjectionCoefficients& proj)
    : linear_model_(proj), proj_(proj) {
  JASSERT_GT(proj.fx, 0.0, "Focal length must be positive");
  JASSERT_GT(proj.fy, 0.0, "Focal length must be positive");
  JASSERT_GT(proj.cols, 0, "Cols must be positive");
  JASSERT_GT(proj.rows, 0, "Rows must be positive");
}

jcc::Vec2 NonlinearCameraModel::project_unchecked(const jcc::Vec3& camera_point) const {
  const jcc::Vec2 projection = apply_projection_and_distortion(proj_, camera_point);
  return projection;
}

jcc::Optional<jcc::Vec2> NonlinearCameraModel::project(
    const jcc::Vec3& camera_point) const {
  const jcc::Vec2 projection = apply_projection_and_distortion(proj_, camera_point);

  const bool in_image = (projection.x() >= 0 && projection.x() <= proj_.cols) &&  //
                        (projection.y() >= 0 && projection.y() <= proj_.rows);
  if (in_image) {
    return projection;
  } else {
    return {};
  }
}

jcc::Optional<geometry::Ray> NonlinearCameraModel::unproject(
    const jcc::Vec2& image_point) const {
  const jcc::Vec2 focal_scaling = jcc::Vec2(proj_.fx, proj_.fy);
  const jcc::Vec2 principal_point(proj_.cx, proj_.cy);

  const jcc::Vec2 p_prime =
      (image_point - principal_point).array() / focal_scaling.array();

  const jcc::Vec2 error = (image_point - apply_projection(proj_, p_prime));
  JASSERT_LT(error.norm(), 1e-3, "Jake made an arithmetic mistake");

  // Ok, now optimize over unit vectors such that...
  const auto error_fnc = [proj = proj_, p_prime](const jcc::Vec2& proposed) {
    const jcc::Vec3 proposed_world_pt = jcc::augment(proposed, 1.0);
    return apply_distortion(proj, proposed_world_pt) - p_prime;
  };

  const auto soln = numerics::gauss_newton_minimize<2, 2>(error_fnc, p_prime);

  if (soln.success && (soln.terminal_error.norm() < 1e-2)) {
    const jcc::Vec3 world_point = jcc::augment(soln.solution, 1.0);
    const geometry::Ray ray{jcc::Vec3::Zero(), world_point.normalized()};
    return {ray};
  } else {
    return {};
  }
}

int NonlinearCameraModel::rows() const {
  return proj_.rows;
}

int NonlinearCameraModel::cols() const {
  return proj_.cols;
}

}  // namespace estimation
