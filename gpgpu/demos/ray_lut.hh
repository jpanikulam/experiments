#include "estimation/calibration/nonlinear_camera_model.hh"

#include <opencv2/opencv.hpp>

namespace jcc {

estimation::NonlinearCameraModel make_model();

cv::Mat create_ray_lut(const estimation::NonlinearCameraModel &model,
                       const int cols,
                       const int rows);

}  // namespace jcc