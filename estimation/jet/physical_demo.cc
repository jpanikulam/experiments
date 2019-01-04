#include "util/environment.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/scene_tree.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <opencv2/opencv.hpp>
#include "estimation/vision/fiducial_pose.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

struct ProjectionModel {
  double fx;
  double fy;
  double cx;
  double cy;

  double p1;
  double p2;

  double k1;
  double k2;
  double k3;
};

jcc::Vec2 distort(const ProjectionModel& proj, const jcc::Vec3& world_point) {
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

void setup() {
  const auto view = viewer::get_window3d("FiducialDebug");

  view->set_azimuth(0.0);
  view->set_elevation(0.0);
  view->set_zoom(1.0);

  // view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
  // jcc::Vec3(-1.0, 0.0, -1.0)));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};

  background->add_plane({ground, 0.1});
  background->flip();
}

void setup2() {
  const auto view = viewer::get_window3d("ImageView");

  // 1920, 1080

  view->set_azimuth(0.0);
  view->set_elevation(0.0);
  view->set_zoom(1.0);

  // view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
  // jcc::Vec3(-1.0, 0.0, -1.0)));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};

  background->add_plane({ground, 0.1});
  background->flip();
}

}  // namespace

void run() {
  setup();
  setup2();

  const auto view = viewer::get_window3d("FiducialDebug");
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  const std::string fiducial_path = jcc::Environment::asset_path() + "fiducial.jpg";
  const cv::Mat fiducial_tag = cv::imread(fiducial_path);
  const auto fiducial_image = view->add_primitive<viewer::Image>(fiducial_tag, 0.28, 1);
  /*
  const auto image = std::make_shared<viewer::Image>(camera_frame, 1, 1);
  {
    const auto tree = view->add_primitive<viewer::SceneTree>();
    // const SE3 world_from_image(SO3::exp(jcc::Vec3(M_PI * 0.5, 0.0, 0.0)),
                               // jcc::Vec3::Zero());
    const SE3 world_from_image;
    tree->add_primitive("root", world_from_image, "image", image);
  }
  */

  const auto im_view = viewer::get_window3d("ImageView");
  cv::Mat camera_frame = cv::Mat::zeros(cv::Size(640, 640), CV_8UC3);
  const auto image = im_view->add_primitive<viewer::Image>(camera_frame);

  auto cap = cv::VideoCapture(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
  cap.set(cv::CAP_PROP_AUTO_WB, 0);

  // cap.set(cv::CAP_PROP_AUTO_WB, 0);
  // cap.set(cv::CAP_PROP_AUTO_WB, 0);
  // cap.set(cv::CAP_PROP_AUTO_WB, 0);

  while (!view->should_close()) {
    if (cap.read(camera_frame)) {
      cv::Mat cf_blurred;
      // blur = cv2.blur(img,(5,5))
      cv::GaussianBlur(camera_frame, cf_blurred, cv::Size(5, 5), 1.0, 1.0);

      const auto result = vision::detect_markers(cf_blurred);

      for (const auto& detection : result) {
        const SE3 world_from_camera = detection.marker_center_from_camera;
        geo->add_axes({world_from_camera});

        for (const auto& pt : detection.image_points) {
          cv::circle(camera_frame, cv::Point2f(pt.x(), pt.y()), 5.0,
                     cv::Scalar(255, 15, 15), 5.0);
        }
      }
      geo->flip();
      image->update_image(camera_frame);
    }
  }

  view->spin_until_step();
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::run();
}