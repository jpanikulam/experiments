#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "estimation/calibration/nonlinear_camera_model.hh"
#include "estimation/vision/sample_bilinear.hh"

#include "estimation/visualization/visualize_camera_frustum.hh"

#include "geometry/plane.hh"

#include "viewer/primitives/image.hh"
#include "viewer/primitives/scene_tree.hh"

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace estimation {
namespace {

constexpr float FIDUCIAL_WIDTH_METERS = 99.0 / 1000;
constexpr float FIDUCIAL_GAP_WIDTH_METERS = 50.0 / 1000;

inline cv::Ptr<cv::aruco::Dictionary> get_aruco_dictionary() {
  return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
}

inline cv::Ptr<cv::aruco::GridBoard> get_aruco_board() {
  return cv::aruco::GridBoard::create(4, 4, FIDUCIAL_WIDTH_METERS,
                                      FIDUCIAL_GAP_WIDTH_METERS, get_aruco_dictionary());
}

NonlinearCameraModel make_model() {
  estimation::ProjectionCoefficients proj_coeffs;
  proj_coeffs.fx = 279.76;
  proj_coeffs.fy = 280.034;
  proj_coeffs.cx = 235.992;
  proj_coeffs.cy = 141.951;
  proj_coeffs.k1 = 0.0669332;
  proj_coeffs.k2 = -0.224151;
  proj_coeffs.p1 = 0.00993584;
  proj_coeffs.p2 = 0.00848696;
  proj_coeffs.k3 = 0.10179;
  proj_coeffs.rows = 270;
  proj_coeffs.cols = 480;

  const NonlinearCameraModel model(proj_coeffs);
  return model;
}

cv::Mat render_at_pose(const SE3& camera_from_fiducial, const cv::Mat& fiducial_image) {
  constexpr double HEIGHT_M = 1.0;

  const geometry::Plane im_plane_fiducial_frame{
      .origin = jcc::Vec3::Zero(),        //
      .normal = geometry::Unit3::UnitZ()  //
  };

  const auto im_plane_camera_frame = camera_from_fiducial * im_plane_fiducial_frame;

  const auto view = viewer::get_window3d("Calibration");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  geo->add_axes({camera_from_fiducial});

  const auto model = make_model();

  cv::Mat out_image(cv::Size(480, 270), CV_8UC1, cv::Scalar(0));
  // cv::Mat out_image(cv::Size(48, 27), CV_8UC1);

  const double width_per_height = out_image.cols / static_cast<double>(out_image.rows);

  // ui2d->add_image(fiducial_image, 1.0);

  jcc::Vec3 intersection_camera_frame;
  for (int u = 0; u < out_image.cols; ++u) {
    for (int v = 0; v < out_image.rows; ++v) {
      const auto optl_ray = model.unproject(jcc::Vec2(u, v));
      // Could not produce a projection
      if (!optl_ray) {
        std::cout << "Lost a ray at : " << u << ", " << v << std::endl;
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

      ui2d->add_point({pt_fiducial_image_frame, jcc::Vec4(0.0, 1.0, 0.0, 1.0), 1.0});

      // const double result =
      //     interpolate_nearest(fiducial_image, pt_fiducial_image_frame * model.rows());
      const double result =
          interpolate_bilinear(fiducial_image, pt_fiducial_image_frame * model.rows());

      out_image.at<uint8_t>(v, u) = static_cast<uint8_t>(result);

      if ((v % 10 == 0) && (u % 10 == 0)) {
        geo->add_ray(*optl_ray, 1.0);
        geo->flush();
      }
    }
    if (u % 50 == 0) {
      ui2d->add_image(out_image, 1.0);
      ui2d->flip();
      view->spin_until_step();
    }
  }
  ui2d->add_image(out_image, 1.0);
  ui2d->flip();
  view->spin_until_step();

  return out_image;
}

}  // namespace

void go() {
  const auto view = viewer::get_window3d("Calibration");
  const auto ui2d = view->add_primitive<viewer::Ui2d>();
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  visualize_camera_frustum(*geo, *ui2d, make_model());
  geo->flip();
  ui2d->flip();

  cv::Mat board_image;
  get_aruco_board()->draw(cv::Size(900, 900), board_image, 50, 1);
  const SE3 camera_from_fiducial(SO3(), jcc::Vec3(0.0, 0.0, 1.0));

  {
    const auto image_tree = view->add_primitive<viewer::SceneTree>();

    const auto board_texture_image =
        std::make_shared<viewer::Image>(board_image, 1.0, 1.0);

    image_tree->add_primitive("root", camera_from_fiducial, "fiducial",
                              board_texture_image);
  }

  render_at_pose(camera_from_fiducial, board_image);

  // OK: Now render some good bullshit
  jcc::Success() << "Finished rendering." << std::endl;
  view->flip();
  view->spin_until_step();
  jcc::Success() << "Done." << std::endl;
}
}  // namespace estimation

int main() {
  estimation::go();
}