#include "estimation/vision/simulate_camera.hh"

#include "estimation/demos/make_model.hh"
#include "estimation/vision/sample_bilinear.hh"
#include "geometry/plane.hh"

namespace estimation {

cv::Mat render_at_pose(const SE3& camera_from_fiducial, const cv::Mat& fiducial_image) {
  const geometry::Plane im_plane_fiducial_frame{
      .origin = jcc::Vec3::Zero(),        //
      .normal = geometry::Unit3::UnitZ()  //
  };

  const auto im_plane_camera_frame = camera_from_fiducial * im_plane_fiducial_frame;

  const auto model = make_model();

  cv::Mat out_image(cv::Size(480, 270), CV_8UC1, cv::Scalar(0));

  const double width_per_height = out_image.cols / static_cast<double>(out_image.rows);

  jcc::Vec3 intersection_camera_frame;
  for (int u = 0; u < out_image.cols; ++u) {
    for (int v = 0; v < out_image.rows; ++v) {
      const auto optl_ray = model.unproject(jcc::Vec2(u + 0.5, v + 0.5));
      // Could not produce a projection
      if (!optl_ray) {
        continue;
      }

      const bool intersected =
          im_plane_camera_frame.intersect(*optl_ray, out(intersection_camera_frame));

      // Did not hit the plane
      if (!intersected) {
        continue;
      }

      const jcc::Vec3 intersection_fiducial_frame =
          camera_from_fiducial.inverse() * intersection_camera_frame;

      const bool out_x = intersection_fiducial_frame.x() < 0.0 ||
                         intersection_fiducial_frame.x() >= width_per_height;
      const bool out_y =
          intersection_fiducial_frame.y() < 0.0 || intersection_fiducial_frame.y() >= 1.0;
      // Out of the frame
      if (out_x || out_y) {
        continue;
      }

      const jcc::Vec2 pt_fiducial_image_frame(intersection_fiducial_frame.x(),
                                              intersection_fiducial_frame.y());

      const double result = interpolate_bilinear(
          fiducial_image, pt_fiducial_image_frame * fiducial_image.rows);

      out_image.at<uint8_t>(v, u) = static_cast<uint8_t>(result);
    }
  }

  return out_image;
}
}  // namespace estimation