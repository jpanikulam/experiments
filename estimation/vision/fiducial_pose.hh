#pragma once

//%deps(opencv)

#include <opencv2/aruco.hpp>

namespace fiducials {
void detect_markers(cv::Mat mat);
}  // namespace fiducials