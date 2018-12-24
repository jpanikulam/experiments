#include "estimation/vision/fiducial_pose.hh"
#include "eigen.hh"
//%deps(opencv)

namespace estimation {
namespace vision {

constexpr int BOARD_WIDTH = 9;
constexpr int BOARD_HEIGHT = 6;
constexpr float SQUARE_SIZE = 1.0 / 10;

std::vector<MarkerDetection> detect_markers(cv::Mat inputImage) {
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(inputImage, dictionary, corners, ids);
  // cv::aruco::drawDetectedMarkers(inputImage, corners, ids);
  std::vector<cv::Vec3d> rvecs, tvecs;

  cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 320, 0, 320, 0, 320, 320, 0, 0, 1);

  // webcam
  // cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 509.55744588, 0, 331.84201483,
  //                                            0, 512.03515271, 250.87030742,
  //                                            0, 0, 1);
  // cv::Mat distortionCoefficients = (cv::Mat1d(1, 8) << -0.0953, 0.108,
  // 0.0455, -0.0312, -0.032);
  cv::Mat distortionCoefficients = (cv::Mat1d(1, 8) << 0, 0, 0, 0, 0);

  cv::aruco::estimatePoseSingleMarkers(corners, 0.49375, cameraMatrix,
                                       distortionCoefficients, rvecs, tvecs);
  // The returned transformation is the one that transforms points from each
  // marker
  // coordinate system to the camera coordinate system. The marker corrdinate
  // system
  // is centered on the middle of the marker, with the Z axis perpendicular to
  // the marker plane.
  std::vector<MarkerDetection> detections;
  // draw axis for each marker
  for (int i = 0; i < ids.size(); i++) {
    cv::aruco::drawAxis(inputImage, cameraMatrix, distortionCoefficients,
                        rvecs[i], tvecs[i], 1);
    auto camera_from_marker_center =
        SE3(SO3::exp(jcc::Vec3(rvecs[i][0], rvecs[i][1], rvecs[i][2])),
            jcc::Vec3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));

    std::cout << "raw tvec " << tvecs[i][0] << " " << tvecs[i][1] << " "
              << tvecs[i][2] << " " << std::endl;
    MarkerDetection detection;
    detection.marker_center_from_camera = camera_from_marker_center.inverse();
    detection.id = ids[i];
    detections.push_back(detection);
  }

  return detections;
}

std::vector<MarkerInWorld> get_world_from_marker_centers(
    cv::Mat camera_image, SE3 world_from_opengl_camera) {
  std::vector<MarkerDetection> marker_detections =
      estimation::vision::detect_markers(camera_image);
  std::vector<MarkerInWorld> result;
  for (auto const& image_detection : marker_detections) {
    auto opengl_camera_from_opencv_camera =
        SE3(SO3::exp(Eigen::Vector3d(M_PI, 0, 0)), Eigen::Vector3d(0, 0, 0)) *
        SE3(SO3::exp(Eigen::Vector3d(0, 0, M_PI / 2 * 0)),
            Eigen::Vector3d(0, 0, 0));

    const auto marker_center_from_opencv_camera =
        image_detection.marker_center_from_camera;
    const auto opencv_camera_from_marker_center =
        marker_center_from_opencv_camera.inverse();
    const auto world_from_marker_frame =
        (world_from_opengl_camera * opengl_camera_from_opencv_camera *
         opencv_camera_from_marker_center);
    const MarkerInWorld world_space_detection{
        .world_from_marker = world_from_marker_frame, .id = image_detection.id};
    result.push_back(world_space_detection);
  }
  return result;
}

void CalibrationManager::add_camera_image(cv::Mat image) {
  all_camera_images.push_back(image);
}

int CalibrationManager::num_images_collected() {
  return all_camera_images.size();
}

std::pair<cv::Mat, cv::Mat> CalibrationManager::calibrate() {
  // based on
  // https://github.com/sourishg/stereo-calibration/blob/master/calib_intrinsic.cpp
  assert(all_camera_images.size() > 0);

  auto board_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);
  auto criteria =
      (cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.001);

  std::vector<std::vector<cv::Point3f> > object_points;
  std::vector<std::vector<cv::Point2f> > image_points;

  std::vector<cv::Point3f>
      obj;  // simply a description of the board's intersection points
  for (int i = 0; i < BOARD_HEIGHT; i++)
    for (int j = 0; j < BOARD_WIDTH; j++)
      obj.push_back(
          cv::Point3f((float)j * SQUARE_SIZE, (float)i * SQUARE_SIZE, 0));

  for (auto const& image : all_camera_images) {
    cv::Mat gray;
    std::vector<cv::Point2f> corners;

    cv::cvtColor(image, gray, CV_BGR2GRAY);
    bool found = cv::findChessboardCorners(
        gray, board_size, corners,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if (found) {
      cornerSubPix(
          gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      image_points.push_back(corners);
      object_points.push_back(obj);
    }
  }

  cv::Mat K;
  cv::Mat D;
  std::vector<cv::Mat> rvecs, tvecs;
  int flag = 0;
  flag |= CV_CALIB_FIX_K4;
  flag |= CV_CALIB_FIX_K5;
  cv::calibrateCamera(object_points, image_points, all_camera_images[0].size(),
                      K, D, rvecs, tvecs, flag);

  std::cout << "K" << K;
  std::cout << "D" << D;
  return std::make_pair(K, D);
}

}  // namespace vision
}  // namespace estimation