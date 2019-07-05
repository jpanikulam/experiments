#pragma once

#include "estimation/calibration/calibration_dataset.hh"
#include "estimation/calibration/estimate_imu_intrinsics.hh"
#include "estimation/calibration/nonlinear_camera_model.hh"
#include "estimation/calibration/rotation_between_vector_series.hh"
#include "estimation/vision/robust_pnp.hh"

#include "geometry/types/unit_vector.hh"

#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace jet {
namespace filtering {

void visualize_mag_data(
    const estimation::calibration::ImuModel& imu_model,
    const estimation::calibration::ImuCalibrationMeasurements& measurements,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const std::shared_ptr<viewer::Ui2d> ui2d);

void visualize_gyro_data(
    const estimation::calibration::ImuModel& imu_model,
    const estimation::calibration::ImuCalibrationMeasurements& measurements,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const std::shared_ptr<viewer::Ui2d> ui2d);

void visualize_fwd_difference_angular(
    const estimation::calibration::CalibrationMeasurements& measurements,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const std::shared_ptr<viewer::Ui2d> ui2d);

void visualize_imu_data_with_intrinsics(
    const estimation::calibration::ImuModel& imu_model,
    const estimation::calibration::ImuCalibrationMeasurements& measurements,
    const geometry::UnitVector3& g_direction,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const std::shared_ptr<viewer::Ui2d> ui2d);

void visualize_fwd_difference(
    const estimation::calibration::CalibrationMeasurements& measurements,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const std::shared_ptr<viewer::Ui2d> ui2d);

void visualize_fwd_acceleration(
    const estimation::calibration::CalibrationMeasurements& measurements,
    const std::shared_ptr<viewer::SimpleGeometry> geo,
    const std::shared_ptr<viewer::Ui2d> ui2d);

void visualize_camera_distortion(const std::shared_ptr<viewer::Ui2d>& ui2d,
                                 const ImageMeasurement& image,
                                 const estimation::NonlinearCameraModel& model);

void visualize_camera_frustum(const std::shared_ptr<viewer::SimpleGeometry>& geo,
                              const std::shared_ptr<viewer::Ui2d>& ui2d,
                              const ImageMeasurement& image,
                              const estimation::NonlinearCameraModel& model);

void visualize_fiducial_detection(
    const std::shared_ptr<viewer::SimpleGeometry>& geo,
    const std::shared_ptr<viewer::Ui2d>& ui2d,
    const estimation::NonlinearCameraModel& model,
    const SE3& fiducial_from_camera,
    const std::vector<ObjectImageAssociations>& obj_pt_associations);

estimation::OptimizationVisitor create_gyro_orientation_optimization_visitor(
    const std::vector<jcc::Vec3>& w_gyro_at_t, const std::vector<jcc::Vec3>& w_cam_at_t);

}  // namespace filtering
}  // namespace jet