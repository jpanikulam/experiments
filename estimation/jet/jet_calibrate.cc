//%ignore

#include "viewer/primitives/image.hh"
#include "viewer/primitives/scene_tree.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <opencv2/opencv.hpp>
#include "estimation/vision/calibration.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace estimation {
namespace jet_filter {
namespace {

const cv::Size get_chess_size() {
  constexpr int BOARD_WIDTH = 9;
  constexpr int BOARD_HEIGHT = 6;

  const auto board_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);
  return board_size;
}

void setup() {
  const auto view = viewer::get_window3d("ImageView");

  view->set_azimuth(0.0);
  view->set_elevation(0.0);
  view->set_zoom(1.0);

  view->set_target_from_world(SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)),
                                  jcc::Vec3(-1.0, 0.0, -1.0)));
  view->set_continue_time_ms(10);

  const auto background = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();
}

std::vector<cv::Point2f> chess_corners(const cv::Mat& image) {
  cv::Mat gray;
  std::vector<cv::Point2f> corners;

  // constexpr float SQUARE_SIZE = .1;

  const auto board_size = get_chess_size();

  cv::cvtColor(image, gray, CV_BGR2GRAY);
  bool found = cv::findChessboardCorners(
      gray, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
  if (found) {
    cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                 cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001));
  }
  return corners;
}
/*
[265.9604351267379, 0, 323.3841822012849;
 0, 302.7743119963964, 177.7795703229708;
 0, 0, 1]
-----
[
  -0.0881294556831833,
  0.08627577358744372,
  -0.006574803742454203,
  0.01200680448589873,
  -0.02887266477084746
]
*/
}  // namespace

void run() {
  setup();

  const auto view = viewer::get_window3d("ImageView");
  cv::Mat camera_frame = cv::Mat::zeros(cv::Size(640, 640), CV_8UC3);

  const auto image = std::make_shared<viewer::Image>(camera_frame, 1, 1);
  {
    const auto tree = view->add_primitive<viewer::SceneTree>();
    const SE3 world_from_image(SO3::exp(jcc::Vec3(M_PI * 0.5, 0.0, 0.0)),
                               jcc::Vec3::Zero());
    tree->add_primitive("root", world_from_image, "image", image);
  }

  auto cap = cv::VideoCapture(0);

  const auto board_size = get_chess_size();

  vision::CalibrationManager mgr;
  constexpr int NUM_STEPS = 1000;

  int n_since_last_discovered = 0;

  for (int k = 0; k < NUM_STEPS && !view->should_close(); ++k) {
    if (cap.read(camera_frame)) {
      const auto found_corners = chess_corners(camera_frame);
      if (!found_corners.empty()) {
        n_since_last_discovered += 1;
        if (n_since_last_discovered < 10) {
        } else {
          n_since_last_discovered = 0;

          std::cout << "Found" << std::endl;
          constexpr bool FOUND_CHESSBOARD = true;
          cv::drawChessboardCorners(camera_frame, board_size, found_corners,
                                    FOUND_CHESSBOARD);
          mgr.add_camera_image(camera_frame);
        }
      }
      image->update_image(camera_frame);
    }
  }

  std::cout << "Calibrating" << std::endl;
  const auto result = mgr.calibrate();
  std::cout << result.K << std::endl;

  std::cout << "-----" << std::endl;
  std::cout << result.D << std::endl;

  // geo->flip();

  view->spin_until_step();
}

}  // namespace jet_filter
}  // namespace estimation

int main() {
  estimation::jet_filter::run();
}