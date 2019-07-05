#include "estimation/vision/robust_pnp.hh"

#include "logging/assert.hh"
#include "numerics/numdiff.hh"
#include "vision/robust_estimator.hh"

// TODO
#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace estimation {

SolvePnpVisitor make_pnp_visitor(const estimation::NonlinearCameraModel& model,
                                 const std::vector<jcc::Vec3>& points_object_frame,
                                 const std::vector<jcc::Vec2>& image_points) {
  const auto view = viewer::get_window3d("Calibration");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const auto visitor = [view, geo, ui2d, model, points_object_frame,
                        image_points](const SolvePnpResult& result) {
    if (!result.success) {
      // return;
    }

    double max_weight = 0.0;
    for (const auto w : result.weights) {
      max_weight = std::max(w, max_weight);
    }

    for (std::size_t k = 0u; k < image_points.size(); ++k) {
      const jcc::Vec2 projected =
          model.project_unchecked(result.camera_from_object * points_object_frame[k]);

      const double normalized_weight = result.weights[k] / max_weight;

      ui2d->add_point({projected / model.rows(), jcc::Vec4(normalized_weight, 1.0, 0.5, 1.0), 8.0});
      ui2d->add_line({projected / model.rows(), image_points[k] / model.rows(),
                      jcc::Vec4(0.0, 0.8, 0.4, 1.0), 4.0});

      geo->add_point({result.camera_from_object * points_object_frame[k],
                      jcc::Vec4(0.0, 1.0, 0.5, 1.0)});
    }

    geo->flip();
    ui2d->flip();
    view->spin_until_step();
    geo->clear();
    ui2d->clear();
  };

  geo->clear();
  ui2d->clear();
  return visitor;
}

namespace {

SolvePnpResult compute_step(const estimation::NonlinearCameraModel& model,
                            const SE3& camera_from_object,
                            const std::vector<jcc::Vec3>& points_object_frame,
                            const std::vector<jcc::Vec2>& image_points,
                            const slam::RobustEstimator& est) {
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  Mat6 JtJ = Mat6::Zero();
  jcc::Vec6 Jtv = jcc::Vec6::Zero();

  SolvePnpResult result;
  result.weights.resize(points_object_frame.size());

  jcc::Vec2 total_reproj_error = jcc::Vec2::Zero();

  for (std::size_t k = 0; k < points_object_frame.size(); ++k) {
    const jcc::Vec3& observed_pt_board_frame = points_object_frame[k];
    const jcc::Vec2& observed_pt_image = image_points[k];

    const auto error_fcn = [&model, &camera_from_object, &observed_pt_image,
                            &observed_pt_board_frame](const jcc::Vec6& w) {
      const jcc::Vec3 pt_camera_frame =
          SE3::exp(w) * camera_from_object * observed_pt_board_frame;
      const jcc::Vec2 expected_pt_image = model.project_unchecked(pt_camera_frame);
      const jcc::Vec2 error = (observed_pt_image - expected_pt_image);
      return error;
    };
    const MatNd<2, 6> J_k =
        numerics::numerical_jacobian<2, 6>(jcc::Vec6::Zero(), error_fcn, 1e-6);
    const jcc::Vec2 v_k = error_fcn(jcc::Vec6::Zero());

    total_reproj_error += v_k;

    const auto cost_and_weight = est(v_k.transpose() * v_k);
    const double w = cost_and_weight.weight;
    result.weights[k] = w;

    result.average_error += v_k;
    Jtv += w * J_k.transpose() * v_k;
    JtJ += w * J_k.transpose() * J_k;
  }

  result.average_error /= image_points.size();

  std::cout << "Error: " << total_reproj_error.transpose() / points_object_frame.size()
            << std::endl;

  constexpr double MARQUARDT_LAMBDA = 1e-3;
  const Mat6 marquardt = (JtJ.diagonal() * MARQUARDT_LAMBDA).asDiagonal();
  const Eigen::LDLT<Mat6> ldlt(marquardt + JtJ);
  JASSERT_EQ(ldlt.info(), Eigen::Success, "LDLT must not fail");

  const jcc::Vec6 delta = -ldlt.solve(Jtv);
  result.camera_from_object = SE3::exp(delta) * camera_from_object;
  return result;
}
}  // namespace

SolvePnpResult robust_pnp(const estimation::NonlinearCameraModel& model,
                          const SE3& object_from_camera_init,
                          const std::vector<jcc::Vec3>& points_object_frame,
                          const std::vector<jcc::Vec2>& image_points,
                          const SolvePnpVisitor& visitor) {
  SolvePnpResult result;
  JASSERT_EQ(points_object_frame.size(), image_points.size(),
             "Must have the same number of object points and image points");

  result.camera_from_object =
      SE3::exp(jcc::Vec6::Random() * 0.1) * object_from_camera_init.inverse();
  result.weights.resize(points_object_frame.size(), 0.0);
  result.average_error.setZero();

  for (int iter = 0; iter < 20; ++iter) {
    if (visitor) {
      visitor(result);
    }

    double width;
    if (iter < 5) {
      width = 0.5;
    } else {
      width = 0.25;
    }
    const auto est = slam::HuberCost(width);
    result = compute_step(model, result.camera_from_object, points_object_frame,
                          image_points, est);
  }

  result.success = true;

  if (visitor) {
    visitor(result);
  }
  return result;
}
}  // namespace estimation