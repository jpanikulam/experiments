#include "vision/bootstrapping/bootstrap_pose.hh"

#include "geometry/ray_ray_approx_intersect.hh"
#include "vision/robust_estimator.hh"

namespace slam {

BootstrapResult Bootstrapper::compute_nonmetric_pose(const std::vector<Vec2>& image_points_a,
                                                     const std::vector<Vec2>& image_points_b,
                                                     const CameraModel&       camera_model,
                                                     const SE3&               initial_camera_from_object) {
  // Primer
  // To do this, we must assume a scene scale, and then simultaneously estimate *all* of the structure parameters, and
  // *all* of the pose parameters
  //
  // - Each observation constraints two degrees of freedom: x, y image-frame translation
  // - There are only five observable degrees of freedom: absolute rotation and direction of motion on S2.
  //
  //
  ////////////////////////////////////////////////
  // Given a camera pose and a correspondence, we can estimate a "best 3d point" by triangulation
  //
  // --> We can do this by computing "pose" as:
  // SE3(SO3::exp(r_params), SO3::exp(0.0, *t_params) * Vec3::UnitX())
  //

  const L2Cost    cost_fcn;
  BootstrapResult result;

  for (size_t k = 0; k < image_points_a.size(); ++k) {
    const Vec2& pt_a = image_points_a[k];
    const Vec2& pt_b = image_points_b[k];

    const Vec3 triangulated_point = geometry::ray_ray_approx_intersect(pt_a, pt_b);

    std::cout << triangulated_point.transpose() << std::endl;
  }

  // const auto cost_weight = cost_fcn()
  return {};
}
}