#include "estimation/vision/fiducial_pose.hh"

//%deps(opencv)

namespace fiducials {
void detect_markers(cv::Mat mat) {
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}
}  // namespace fiducials