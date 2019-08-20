#pragma once

#include "estimation/calibration/nonlinear_camera_model.hh"
#include "estimation/vision/image_size.hh"

#include <opencv2/opencv.hpp>

namespace jcc {

estimation::NonlinearCameraModel make_model();
estimation::NonlinearCameraModel nice_model(const ImageSize& image_size);

cv::Mat create_ray_lut(const estimation::NonlinearCameraModel &model,
                       const int cols,
                       const int rows);

}  // namespace jcc