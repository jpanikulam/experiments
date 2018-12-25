#pragma once
#include <opencv2/opencv.hpp>
//%deps(opencv)
#include <vector>
#include "sophus.hh"
#include <opencv2/aruco.hpp>

namespace fiducials {
struct MarkerDetection{
	SE3 camera_to_marker_center;
	int id;
};
std::vector< MarkerDetection > detect_markers(cv::Mat mat);
}  // namespace fiducials