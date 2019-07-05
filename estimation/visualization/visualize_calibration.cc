#include "estimation/visualization/visualize_calibration.hh"

#include "viewer/window_3d.hh"

#include "eigen_helpers.hh"

namespace jet {
namespace filtering {

void visualize_mag_data(const estimation::calibration::ImuModel& imu_model,
                        const estimation::calibration::ImuCalibrationMeasurements& measurements,
                        const std::shared_ptr<viewer::SimpleGeometry> geo,
                        const std::shared_ptr<viewer::Ui2d> ui2d) {
  geo->clear();
  ui2d->clear();
  const auto first = measurements.first();

  viewer::LinePlotBuilder builder("Magnetometer");
  viewer::PlotRange range;
  builder.set_range(range);

  auto& mag_x = builder.make_subplot("mag_x", jcc::Vec4(1.0, 0.0, 0.0, 1.0), 0.5);
  auto& mag_y = builder.make_subplot("mag_y", jcc::Vec4(0.0, 1.0, 0.0, 1.0), 0.5);
  auto& mag_z = builder.make_subplot("mag_z", jcc::Vec4(0.0, 0.0, 1.0, 1.0), 0.5);
  auto& g_alignment = builder.make_subplot("g_alignment", jcc::Vec4(0.0, 1.0, 1.0, 1.0), 0.5);
  int k = 0;

  for (const auto& mag : measurements.mag_meas) {
    const auto t = estimation::to_seconds(mag.timestamp - first);
    const jcc::Vec3 mag_utesla = mag.measurement.observed_bfield;
    const jcc::Vec3 accel = measurements.accel_meas[k].measurement.observed_acceleration;

    const jcc::Vec3 corrected_mag = imu_model.correct_measured_mag(mag_utesla);
    const jcc::Vec3 corrected_accel = imu_model.correct_measured_accel(accel);

    geo->add_point({mag_utesla, jcc::Vec4(1.0, 0.1, 0.1, 1.0)});
    geo->add_point({corrected_mag, jcc::Vec4(0.1, 1.0, 0.1, 1.0)});

    mag_x << jcc::Vec2(t, corrected_mag.x());
    mag_y << jcc::Vec2(t, corrected_mag.y());
    mag_z << jcc::Vec2(t, corrected_mag.z());

    const double angle = std::acos(corrected_mag.normalized().dot(corrected_accel.normalized()));
    g_alignment << jcc::Vec2(t, angle);
    ++k;
  }

  ui2d->add_lineplot(builder);
  geo->add_sphere({jcc::Vec3::Zero(), 1.0});

  geo->flip();
  ui2d->flip();
}

void visualize_gyro_data(const estimation::calibration::ImuModel& imu_model,
                         const estimation::calibration::ImuCalibrationMeasurements& measurements,
                         const std::shared_ptr<viewer::SimpleGeometry> geo,
                         const std::shared_ptr<viewer::Ui2d> ui2d) {
  geo->clear();
  ui2d->clear();
  const auto first = measurements.first();

  viewer::LinePlotBuilder builder("Gyro Readings");
  viewer::PlotRange range;
  range.x_min = 0.0;
  range.x_max = estimation::to_seconds(measurements.last() - first);

  range.y_min = -2.0;
  range.y_max = 2.0;
  builder.set_range(range);

  auto& gyro_x = builder.make_subplot("gyro_x", jcc::Vec4(1.0, 0.0, 0.0, 0.0), 0.5);
  auto& gyro_y = builder.make_subplot("gyro_y", jcc::Vec4(0.0, 1.0, 0.0, 0.0), 0.5);
  auto& gyro_z = builder.make_subplot("gyro_z", jcc::Vec4(0.0, 0.0, 1.0, 0.0), 0.5);
  auto& norm = builder.make_subplot("norm", jcc::Vec4(0.0, 1.0, 1.0, 0.8), 0.5);

  for (const auto& gyro_meas : measurements.gyro_meas) {
    const jcc::Vec3 observed_w = gyro_meas.measurement.observed_w;
    geo->add_point({observed_w, jcc::Vec4(1.0, 0.1, 0.1, 1.0)});

    const double t = estimation::to_seconds(gyro_meas.timestamp - first);

    gyro_x << jcc::Vec2(t, observed_w.x());
    gyro_y << jcc::Vec2(t, observed_w.y());
    gyro_z << jcc::Vec2(t, observed_w.z());
    norm << jcc::Vec2(t, observed_w.norm());
  }

  ui2d->add_lineplot(builder);
  ui2d->flip();

  geo->flush();
}

void visualize_fwd_difference_angular(const estimation::calibration::CalibrationMeasurements& measurements,
                                      const std::shared_ptr<viewer::SimpleGeometry> geo,
                                      const std::shared_ptr<viewer::Ui2d> ui2d) {
  geo->clear();
  ui2d->clear();
  const auto first = measurements.first();

  viewer::LinePlotBuilder builder("w (rad/s)");
  viewer::PlotRange range;
  range.x_min = 0.0;
  range.x_max = estimation::to_seconds(measurements.last() - first);
  range.y_min = -2.0;
  range.y_max = 2.0;
  builder.set_range(range);

  auto& w_x = builder.make_subplot("w_x", jcc::Vec4(1.0, 0.0, 0.0, 0.0), 0.4, true);
  auto& w_y = builder.make_subplot("w_y", jcc::Vec4(0.0, 1.0, 0.0, 0.0), 0.4, true);
  auto& w_z = builder.make_subplot("w_z", jcc::Vec4(0.0, 0.0, 1.0, 0.0), 0.4, true);
  // auto& dt_plt = builder.make_subplot("dt", jcc::Vec4(0.0, 1.0, 1.0, 0.7), 0.4, false);
  auto& norm = builder.make_subplot("norm", jcc::Vec4(0.0, 1.0, 1.0, 0.8), 0.5);

  for (std::size_t i = 1u; i < measurements.fiducial_meas.size(); ++i) {
    const auto prev = measurements.fiducial_meas.at(i - 1);
    const auto next = measurements.fiducial_meas.at(i);

    const auto fiducial_from_camera_0 = prev.measurement.T_fiducial_from_camera;
    const auto fiducial_from_camera_1 = next.measurement.T_fiducial_from_camera;

    const auto t0 = prev.timestamp;
    const auto t1 = next.timestamp;

    const SE3 camera_1_from_camera_0 = fiducial_from_camera_1.inverse() * fiducial_from_camera_0;

    const double dt = estimation::to_seconds(t1 - t0);
    const jcc::Vec3 dR_dt = camera_1_from_camera_0.so3().log() / dt;

    const double t = estimation::to_seconds(t0 - first);

    constexpr double MAX_DT_SEC = 0.2;
    // dt_plt << jcc::Vec2(t, dt);
    if (dt < MAX_DT_SEC) {
      w_x << jcc::Vec2(t, dR_dt.x());
      w_y << jcc::Vec2(t, dR_dt.y());
      w_z << jcc::Vec2(t, dR_dt.z());
      norm << jcc::Vec2(t, dR_dt.norm());
    }
  }

  ui2d->add_lineplot(builder);
  ui2d->flip();

  geo->flush();
}

void visualize_imu_data_with_intrinsics(const estimation::calibration::ImuModel& imu_model,
                                        const estimation::calibration::ImuCalibrationMeasurements& measurements,
                                        const geometry::UnitVector3& g_direction,
                                        const std::shared_ptr<viewer::SimpleGeometry> geo,
                                        const std::shared_ptr<viewer::Ui2d> ui2d) {
  geo->clear();
  ui2d->clear();
  const auto first = measurements.first();

  viewer::LinePlotBuilder builder("IMU Readings");
  viewer::PlotRange range;
  range.x_min = 0.0;
  range.x_max = estimation::to_seconds(measurements.last() - first);
  range.y_min = -1.5;
  range.y_max = 1.5;

  builder.set_range(range);

  auto& imu_x = builder.make_subplot("imu_x", jcc::Vec4(1.0, 0.0, 0.0, 0.0), 0.5);
  auto& imu_y = builder.make_subplot("imu_y", jcc::Vec4(0.0, 1.0, 0.0, 0.0), 0.5);
  auto& imu_z = builder.make_subplot("imu_z", jcc::Vec4(0.0, 0.0, 1.0, 0.0), 0.5);
  auto& norm_uncorrected = builder.make_subplot("norm_uncorrected", jcc::Vec4(0.0, 1.0, 1.0, 0.8), 0.5);
  auto& norm = builder.make_subplot("norm", jcc::Vec4(0.0, 1.0, 1.0, 0.8), 0.5);

  for (const auto& accel_meas : measurements.accel_meas) {
    const jcc::Vec3 observed_acceleration = accel_meas.measurement.observed_acceleration;
    geo->add_point({observed_acceleration, jcc::Vec4(1.0, 0.1, 0.1, 1.0)});

    const jcc::Vec3 corrected = imu_model.correct_measured_mag(observed_acceleration);
    geo->add_point({corrected, jcc::Vec4(0.1, 1.0, 0.1, 1.0)});

    const double t = estimation::to_seconds(accel_meas.timestamp - first);

    imu_x << jcc::Vec2(t, corrected.x());
    imu_y << jcc::Vec2(t, corrected.y());
    imu_z << jcc::Vec2(t, corrected.z());
    // norm << jcc::Vec2(t, corrected.norm());

    const jcc::Vec3 g_absent = (corrected - (9.81 * corrected.normalized()));

    norm << jcc::Vec2(t, g_absent.norm());
    // norm_uncorrected << jcc::Vec2(t, g_absent);
  }

  ui2d->add_lineplot(builder);
  ui2d->flip();

  constexpr double G_MPSS = 9.81;
  geo->add_sphere({jcc::Vec3::Zero(), G_MPSS});
  geo->add_line({jcc::Vec3::Zero(), g_direction.vector() * G_MPSS, jcc::Vec4(0.2, 0.7, 0.9, 1.0)});

  geo->flush();
}

void visualize_fwd_difference(const estimation::calibration::CalibrationMeasurements& measurements,
                              const std::shared_ptr<viewer::SimpleGeometry> geo,
                              const std::shared_ptr<viewer::Ui2d> ui2d) {
  geo->clear();
  ui2d->clear();
  const auto first = measurements.first();

  viewer::LinePlotBuilder builder("Velocity");
  viewer::PlotRange range;
  range.x_min = 0.0;
  range.x_max = estimation::to_seconds(measurements.last() - first);
  builder.set_range(range);

  auto& vel_x = builder.make_subplot("vel_x", jcc::Vec4(1.0, 0.0, 0.0, 0.7), 0.4, true);
  auto& vel_y = builder.make_subplot("vel_y", jcc::Vec4(0.0, 1.0, 0.0, 0.7), 0.4, true);
  auto& vel_z = builder.make_subplot("vel_z", jcc::Vec4(0.0, 0.0, 1.0, 0.7), 0.4, true);
  auto& dt_plt = builder.make_subplot("dt", jcc::Vec4(0.0, 1.0, 1.0, 0.7), 0.4, false);

  for (std::size_t i = 1u; i < measurements.fiducial_meas.size(); ++i) {
    const auto prev = measurements.fiducial_meas.at(i - 1);
    const auto next = measurements.fiducial_meas.at(i);

    const auto fiducial_from_camera_0 = prev.measurement.T_fiducial_from_camera;
    const auto fiducial_from_camera_1 = next.measurement.T_fiducial_from_camera;

    const auto t0 = prev.timestamp;
    const auto t1 = next.timestamp;

    const SE3 camera_1_from_camera_0 = fiducial_from_camera_1.inverse() * fiducial_from_camera_0;

    const double dt = estimation::to_seconds(t1 - t0);
    const jcc::Vec3 dx_dt = camera_1_from_camera_0.translation() / dt;

    const double t = estimation::to_seconds(t0 - first);

    constexpr double MAX_DT_SEC = 0.2;
    dt_plt << jcc::Vec2(t, dt);
    if (dt < MAX_DT_SEC) {
      vel_x << jcc::Vec2(t, dx_dt.x());
      vel_y << jcc::Vec2(t, dx_dt.y());
      vel_z << jcc::Vec2(t, dx_dt.z());
    }
  }

  ui2d->add_lineplot(builder);
  ui2d->flip();

  geo->flush();
}

void visualize_fwd_acceleration(const estimation::calibration::CalibrationMeasurements& measurements,
                                const std::shared_ptr<viewer::SimpleGeometry> geo,
                                const std::shared_ptr<viewer::Ui2d> ui2d) {
  geo->clear();
  ui2d->clear();
  const auto first = measurements.first();

  viewer::LinePlotBuilder builder("Velocity");
  viewer::PlotRange range;
  range.x_min = 0.0;
  range.x_max = estimation::to_seconds(measurements.last() - first);
  range.y_min = -1.5;
  range.y_max = 1.5;
  builder.set_range(range);

  auto& accel_x = builder.make_subplot("accel_x", jcc::Vec4(1.0, 0.0, 0.0, 0.0), 0.4, true);
  auto& accel_y = builder.make_subplot("accel_y", jcc::Vec4(0.0, 1.0, 0.0, 0.0), 0.4, true);
  auto& accel_z = builder.make_subplot("accel_z", jcc::Vec4(0.0, 0.0, 1.0, 0.0), 0.4, true);
  auto& norm = builder.make_subplot("norm", jcc::Vec4(0.0, 1.0, 1.0, 0.8), 0.5);

  // auto& dt_plt = builder.make_subplot("dt", jcc::Vec4(0.0, 1.0, 1.0, 0.7), 0.4, false);

  for (std::size_t i = 1u; i < measurements.fiducial_meas.size() - 1; ++i) {
    const auto prev = measurements.fiducial_meas.at(i - 1);
    const auto cur = measurements.fiducial_meas.at(i);
    const auto next = measurements.fiducial_meas.at(i + 1);

    const auto fiducial_from_camera_prev = prev.measurement.T_fiducial_from_camera;
    const auto fiducial_from_camera_cur = cur.measurement.T_fiducial_from_camera;
    const auto fiducial_from_camera_next = next.measurement.T_fiducial_from_camera;

    const auto t_prev = prev.timestamp;
    const auto t_cur = cur.timestamp;
    const auto t_next = next.timestamp;

    jcc::Vec3 dx_dt_1 = jcc::Vec3::Zero();
    {
      const SE3 camera_cur_from_camera_prev = fiducial_from_camera_cur.inverse() * fiducial_from_camera_prev;
      const double dt = estimation::to_seconds(t_cur - t_prev);
      dx_dt_1 = camera_cur_from_camera_prev.translation() / dt;
    }

    jcc::Vec3 dx_dt_2 = jcc::Vec3::Zero();
    {
      const SE3 camera_next_from_camera_cur = fiducial_from_camera_next.inverse() * fiducial_from_camera_cur;
      const double dt = estimation::to_seconds(t_next - t_cur);
      dx_dt_2 = camera_next_from_camera_cur.translation() / dt;
    }

    const auto t1 = estimation::average(t_prev, t_cur);
    const auto t2 = estimation::average(t_cur, t_next);
    const double accel_dt = estimation::to_seconds(t2 - t1);
    const double t = estimation::to_seconds(t_cur - first);

    const jcc::Vec3 d2x_dt2 = (dx_dt_2 - dx_dt_1) / accel_dt;

    constexpr double MAX_DT_SEC = 0.3;
    if (accel_dt < MAX_DT_SEC) {
      accel_x << jcc::Vec2(t, d2x_dt2.x());
      accel_y << jcc::Vec2(t, d2x_dt2.y());
      accel_z << jcc::Vec2(t, d2x_dt2.z());
      norm << jcc::Vec2(t, d2x_dt2.norm());
    }
  }

  ui2d->add_lineplot(builder);
  ui2d->flip();

  geo->flush();
}

void visualize_camera_distortion(const std::shared_ptr<viewer::Ui2d>& ui2d,
                                 const ImageMeasurement& image,
                                 const estimation::NonlinearCameraModel& model) {
  const auto bottom_left = *model.unproject(jcc::Vec2(0.0, 0.0));
  const auto bottom_right = *model.unproject(jcc::Vec2(image.image.cols, 0.0));
  const auto top_left = *model.unproject(jcc::Vec2(0.0, image.image.rows));
  const auto top_right = *model.unproject(jcc::Vec2(image.image.cols, image.image.rows));

  const double z_dist = 3.0;
  std::vector<jcc::Vec2> mesh_vertices;

  const int num_rows = 32;
  const double scaling = 1.4;

  const double x_min = bottom_left(z_dist).x() * scaling;
  const double x_max = bottom_right(z_dist).x() * scaling;
  const double y_min = bottom_left(z_dist).y() * scaling;
  const double y_max = top_left(z_dist).y() * scaling;

  const double range_m = x_max - x_min;
  const double space_per_increment = range_m / num_rows;

  for (double x = x_min; x <= x_max; x += space_per_increment) {
    for (double y = y_min; y <= y_max; y += space_per_increment) {
      const jcc::Vec3 camera_frame_pt(x, y, z_dist);
      const jcc::Vec2 projected = model.project_unchecked(camera_frame_pt);
      mesh_vertices.push_back(projected / image.image.rows);
    }
  }

  ui2d->add_grid_mesh({num_rows, mesh_vertices});
}

void visualize_camera_frustum(const std::shared_ptr<viewer::SimpleGeometry>& geo,
                              const std::shared_ptr<viewer::Ui2d>& ui2d,
                              const ImageMeasurement& image,
                              const estimation::NonlinearCameraModel& model) {
  const std::vector<jcc::Vec2> image_pts = {
      jcc::Vec2(0.0, image.image.rows),               //
      jcc::Vec2(image.image.cols, image.image.rows),  //
  };

  constexpr double DIST_Z = 4.0;
  for (const auto& image_pt : image_pts) {
    const auto unproj = model.unproject(image_pt);
    JASSERT(static_cast<bool>(unproj), "Could not deproject a point");
    {
      geo->add_ray(*unproj, DIST_Z, jcc::Vec4::Ones());
      ui2d->add_point({image_pt / image.image.rows, jcc::Vec4(1.0, 1.0, 0.2, 1.0), 5.0});
    }
  }

  {
    const jcc::Vec2 bottom_right(image.image.cols, 0.0);
    const auto unproj = model.unproject(bottom_right);
    JASSERT(static_cast<bool>(unproj), "Could not deproject a point");
    {
      geo->add_ray(*unproj, DIST_Z, jcc::Vec4(1.0, 0.1, 0.1, 1.0));
      ui2d->add_point({bottom_right / image.image.rows, jcc::Vec4(1.0, 0.0, 0.0, 1.0), 5.0});
    }
  }

  {
    const jcc::Vec2 bottom_left(0.0, 0.0);
    const auto unproj = model.unproject(bottom_left);
    JASSERT(static_cast<bool>(unproj), "Could not deproject a point");
    {
      geo->add_ray(*unproj, DIST_Z, jcc::Vec4(0.1, 1.0, 0.1, 1.0));
      ui2d->add_point({bottom_left / image.image.rows, jcc::Vec4(0.0, 1.0, 0.0, 1.0), 5.0});
    }
  }
}

void visualize_fiducial_detection(
    const std::shared_ptr<viewer::SimpleGeometry>& geo,
    const std::shared_ptr<viewer::Ui2d>& ui2d,
    const estimation::NonlinearCameraModel& model,
    const SE3& fiducial_from_camera,
    const std::vector<ObjectImageAssociations>& obj_img_associations) {
  // Draw as though the camera is the world frame
  const auto camera_from_fiducial = fiducial_from_camera.inverse();

  const auto maybe_fiducial_center = model.project(camera_from_fiducial.translation());
  if (maybe_fiducial_center) {
    geo->add_line({camera_from_fiducial.translation(), jcc::Vec3::Zero(), jcc::Vec4(0.2, 0.9, 0.2, 1.0), 5.0});

    const jcc::Vec2 fiducial_center_image = *maybe_fiducial_center;
    ui2d->add_point({fiducial_center_image / model.rows(), jcc::Vec4(0.0, 1.0, 0.0, 1.0), 5.0});

    {
      //
      // Draw projection of the 3D Axes
      //
      const auto test_c0 = model.project(camera_from_fiducial * jcc::Vec3(0.1, 0.0, 0.0));
      if (test_c0) {
        ui2d->add_line(
            {fiducial_center_image / model.rows(), *test_c0 / model.rows(), jcc::Vec4(1.0, 0.1, 0.0, 1.0), 2.0});
      }
      const auto test_c1 = model.project(camera_from_fiducial * jcc::Vec3(0.0, 0.1, 0.0));
      if (test_c1) {
        ui2d->add_line(
            {fiducial_center_image / model.rows(), *test_c1 / model.rows(), jcc::Vec4(0.0, 1.0, 0.1, 1.0), 2.0});
      }
      const auto test_cz = model.project(camera_from_fiducial * jcc::Vec3(0.0, 0.0, 0.1));
      if (test_cz) {
        ui2d->add_line(
            {fiducial_center_image / model.rows(), *test_cz / model.rows(), jcc::Vec4(0.1, 0.0, 1.0, 1.0), 2.0});
      }
    }

    const auto deproj = model.unproject(fiducial_center_image);
    if (deproj) {
      geo->add_ray(*deproj, 5.0);
    }
  } else {
    geo->add_line({camera_from_fiducial.translation(), jcc::Vec3::Zero(), jcc::Vec4(0.9, 0.9, 0.9, 1.0), 1.0});
  }

  geo->add_axes({camera_from_fiducial});

  for (const auto& assoc : obj_img_associations) {
    const jcc::Vec4 image_point_color(0.0, 0.0, 0.8, 1.0);
    const jcc::Vec4 world_point_color(1.0, 0.0, 0.2, 1.0);

    ui2d->add_point(
        {jcc::Vec2(assoc.point_image) / model.rows(), image_point_color, 5.0});

    const jcc::Vec3 point_fiducial_frame =
        jcc::augment(jcc::Vec2(assoc.point_object_frame), 0.0);
    const jcc::Vec3 point_camera_frame = camera_from_fiducial * point_fiducial_frame;
    geo->add_point({point_camera_frame, world_point_color, 5.0});

    const auto maybe_projected_point = model.project(point_camera_frame);
    if (maybe_projected_point) {
      ui2d->add_point({*maybe_projected_point / model.rows(), world_point_color, 7.0});
    }
  }
}

estimation::OptimizationVisitor create_gyro_orientation_optimization_visitor(const std::vector<jcc::Vec3>& w_gyro_at_t,
                                                                             const std::vector<jcc::Vec3>& w_cam_at_t) {
  const auto view = viewer::get_window3d("Calibration");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();

  const auto visitor = [view, ui2d, w_cam_at_t, w_gyro_at_t](const estimation::RotationBetweenVectorSeries& result) {
    viewer::LinePlotBuilder builder("Gyro Optimization");

    const double xyz_alpha = 1.0;
    auto& gyro_x = builder.make_subplot("gyro_x", jcc::Vec4(1.0, 0.0, 0.0, xyz_alpha), 0.3);
    auto& gyro_y = builder.make_subplot("gyro_y", jcc::Vec4(0.0, 1.0, 0.0, xyz_alpha), 0.3);
    auto& gyro_z = builder.make_subplot("gyro_z", jcc::Vec4(0.0, 0.0, 1.0, xyz_alpha), 0.3);
    auto& camera_w_x = builder.make_subplot("camera_w_x", jcc::Vec4(1.0, 0.0, 0.0, xyz_alpha), 0.3, true);
    auto& camera_w_y = builder.make_subplot("camera_w_y", jcc::Vec4(0.0, 1.0, 0.0, xyz_alpha), 0.3, true);
    auto& camera_w_z = builder.make_subplot("camera_w_z", jcc::Vec4(0.0, 0.0, 1.0, xyz_alpha), 0.3, true);

    auto& dot_product = builder.make_subplot("dot_product", jcc::Vec4(1.0, 0.0, 1.0, 1.0), 0.3);
    auto& weight = builder.make_subplot("weight", jcc::Vec4(0.0, 1.0, 1.0, 1.0), 0.9);

    for (std::size_t k = 0; k < w_cam_at_t.size(); ++k) {
      const auto& gyro_w = w_gyro_at_t.at(k);
      const auto& cam_w = w_cam_at_t.at(k);
      const double t = static_cast<double>(k) / 20.0;

      const jcc::Vec3 gyro_w_camera_frame = result.b_from_a * gyro_w;

      gyro_x << jcc::Vec2(t, gyro_w_camera_frame.normalized().x());
      gyro_y << jcc::Vec2(t, gyro_w_camera_frame.normalized().y());
      gyro_z << jcc::Vec2(t, gyro_w_camera_frame.normalized().z());
      camera_w_x << jcc::Vec2(t, cam_w.normalized().x());
      camera_w_y << jcc::Vec2(t, cam_w.normalized().y());
      camera_w_z << jcc::Vec2(t, cam_w.normalized().z());
      weight << jcc::Vec2(t, result.weights[k]);

      dot_product << jcc::Vec2(t, gyro_w_camera_frame.normalized().dot(cam_w.normalized()));
    }

    ui2d->add_lineplot(builder);
    ui2d->flip();
    const double average_error = std::sqrt(result.average_error.norm());
    if (result.success) {
      if (average_error > 0.5) {
        jcc::Warning() << "[Camera->IMU] Average error: " << average_error << std::endl;
      } else {
        jcc::Success() << "[Camera->IMU] Aligned Successfully" << std::endl;
      }
    }
    std::cout << "Error: " << average_error << std::endl;
    view->spin_until_step();
    ui2d->clear();
  };
  return visitor;
}

}  // namespace filtering
}  // namespace jet
