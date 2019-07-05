#include "estimation/calibration/calibrate_single_imu.hh"

#include "estimation/calibration/estimate_gravity_direction.hh"
#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "estimation/calibration/rotation_between_vector_series.hh"
#include "estimation/sensors/make_interpolator.hh"

#include "estimation/visualization/visualize_calibration.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace estimation {
namespace {
SO3 estimate_camera_from_gyro(
    const estimation::calibration::CalibrationMeasurements& measurements,
    const estimation::calibration::ImuCalibrationMeasurements& imu_measurements,
    bool visualize = true) {
  const auto interp = estimation::make_gyro_interpolator(imu_measurements.gyro_meas);

  std::vector<jcc::Vec3> w_cam_at_t;
  std::vector<jcc::Vec3> w_gyro_at_t;
  for (std::size_t i = 1u; i < measurements.fiducial_meas.size(); ++i) {
    const auto prev = measurements.fiducial_meas.at(i - 1);
    const auto next = measurements.fiducial_meas.at(i);

    const auto fiducial_from_camera_0 = prev.measurement.T_fiducial_from_camera;
    const auto fiducial_from_camera_1 = next.measurement.T_fiducial_from_camera;

    const auto t0 = prev.timestamp;
    const auto t1 = next.timestamp;

    const SE3 camera_1_from_camera_0 =
        fiducial_from_camera_1.inverse() * fiducial_from_camera_0;

    const double dt = estimation::to_seconds(t1 - t0);
    const jcc::Vec3 dR_dt = -camera_1_from_camera_0.so3().log() / dt;

    const estimation::TimePoint true_t = estimation::average(t1, t0);
    const auto maybe_w_gyro = interp(true_t);
    if (maybe_w_gyro && (dt < 0.2) && (dR_dt.norm() > 0.1)) {
      w_cam_at_t.push_back(dR_dt);
      w_gyro_at_t.push_back(*maybe_w_gyro);
    }
  }

  const auto visitor =
      visualize ? create_gyro_orientation_optimization_visitor(w_gyro_at_t, w_cam_at_t)
                : estimation::OptimizationVisitor{};

  const auto pp =
      estimation::rotation_between_vector_series(w_gyro_at_t, w_cam_at_t, visitor);

  return pp.b_from_a;
}

}  // namespace

SingleImuCalibration create_single_imu_model(
    const estimation::calibration::CalibrationMeasurements& all_cal_measurements,
    const estimation::calibration::ImuCalibrationMeasurements& imu_cal_measurements,
    const CreateSingleImuModelConfig& cfg) {
  jcc::Success() << "[IMU] Estimating intrinsics..." << std::endl;

  const auto imu_model =
      estimation::calibration::estimate_imu_intrinsics(imu_cal_measurements);
  jcc::Success() << "[IMU] Estimating sensor-frame direction of gravity..." << std::endl;
  const auto g_estimate = estimate_gravity_direction(
      all_cal_measurements, imu_cal_measurements, imu_model, {});

  //
  // IMU Visualization
  //
  if (cfg.visualize_imu_model || cfg.visualize_gyro || cfg.visualize_magnetometer) {
    jcc::Success() << "[IMU] Visualizing for validation..." << std::endl;
    const auto view = viewer::get_window3d("Calibration");
    const auto geo = view->add_primitive<viewer::SimpleGeometry>();
    const auto ui2d = view->add_primitive<viewer::Ui2d>();

    if (cfg.visualize_magnetometer) {
      visualize_mag_data(imu_model, imu_cal_measurements, geo, ui2d);

      view->add_toggle_hotkey("visualize_mag", true, 'P');
      const auto view_cb =
          [geo, ui2d, imu_model, imu_cal_measurements](const bool visualize_mag) {
            if (visualize_mag) {
              visualize_mag_data(imu_model, imu_cal_measurements, geo, ui2d);
            } else {
              //
            }
          };
      view->add_toggle_callback("visualize_mag", view_cb);

      view_cb(view->get_toggle("visualize_mag"));

      view->spin_until_step();
      geo->clear();
      ui2d->clear();
      view->clear_toggle_callbacks("visualize_mag");
    }

    if (cfg.visualize_gyro) {
      visualize_gyro_data(imu_model, imu_cal_measurements, geo, ui2d);
      visualize_fwd_difference_angular(all_cal_measurements, geo, ui2d);

      view->add_toggle_hotkey("visualize_gyro", true, 'P');
      const auto view_cb =
          [geo, ui2d, imu_model, imu_cal_measurements, all_cal_measurements](
              const bool visualize_gyro) {
            if (visualize_gyro) {
              visualize_gyro_data(imu_model, imu_cal_measurements, geo, ui2d);
            } else {
              visualize_fwd_difference_angular(all_cal_measurements, geo, ui2d);
            }
          };
      view->add_toggle_callback("visualize_gyro", view_cb);

      view_cb(view->get_toggle("visualize_gyro"));

      view->spin_until_step();
      geo->clear();
      ui2d->clear();
      view->clear_toggle_callbacks("visualize_gyro");
    }

    if (cfg.visualize_imu_model) {
      view->add_toggle_hotkey("visualize_imu_data", true, 'P');
      const auto view_cb =
          [geo, ui2d, imu_model, imu_cal_measurements, all_cal_measurements, g_estimate](
              const bool visualize_imu_data) {
            if (visualize_imu_data) {
              visualize_imu_data_with_intrinsics(
                  imu_model, imu_cal_measurements, g_estimate.direction, geo, ui2d);
            } else {
              // visualize_fwd_difference(imu_cal_measurements, geo, ui2d);
              visualize_fwd_acceleration(all_cal_measurements, geo, ui2d);
            }
          };
      view->add_toggle_callback("visualize_imu_data", view_cb);

      view_cb(view->get_toggle("visualize_imu_data"));
      view->spin_until_step();
      geo->clear();
      ui2d->clear();
    }
    jcc::Success() << "[IMU] Done Visualizing" << std::endl;
  }
  jcc::Success() << "[IMU] Done Calibration IMU" << std::endl;

  //
  // Calibrate camera->imu extrinsics
  //

  jcc::Success() << "[Camera->IMU] Estimating IMU->Camera Extrinsics (Rotation)"
                 << std::endl;
  const SO3 camera_from_gyro = estimate_camera_from_gyro(
      all_cal_measurements, imu_cal_measurements, cfg.visualize_gyro);

  return SingleImuCalibration{
      .imu_model = imu_model,               //
      .g_estimate = g_estimate,             //
      .camera_from_gyro = camera_from_gyro  //
  };
}
}  // namespace estimation