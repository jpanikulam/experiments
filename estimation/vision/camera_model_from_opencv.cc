#include "estimation/vision/camera_model_from_opencv.hh"

#include <opencv2/core/eigen.hpp>

namespace estimation {

CameraModel camera_model_from_opencv(const cv::Mat camera_matrix,
                                     const cv::Mat distortion_coefficients) {
  MatNd<3, 3> K;
  cv::cv2eigen(camera_matrix, K);

  const CameraModel model(K);
  return model;
}

}  // namespace estimation