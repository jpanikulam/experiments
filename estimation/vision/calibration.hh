#pragma once

#include <opencv2/opencv.hpp>

//%deps(opencv)

#include <vector>

#include "sophus.hh"

namespace estimation {
namespace vision {


struct CameraIntrinsics{
	cv::Mat K;
	cv::Mat D;
};

class CalibrationManager {
 private:
  std::vector<cv::Mat> all_camera_images;

 public:
  void add_camera_image(cv::Mat image);

  int num_images_collected();

  const CameraIntrinsics calibrate();
};

}  // namespace vision
}  // namespace estimation