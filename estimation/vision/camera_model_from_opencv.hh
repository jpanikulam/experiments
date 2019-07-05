#pragma once

#include "eigen.hh"

#include "estimation/calibration/camera_model.hh"

#include <opencv2/opencv.hpp>

namespace estimation {

CameraModel camera_model_from_opencv(const cv::Mat camera_matrix,
                                     const cv::Mat distortion_coefficients);

}  // namespace estimation