#include "features.hh"
#include "block_transform.hh"

#include "testing/gtest.hh"

#include <string>

namespace slam {

cv::Mat normalize(const cv::Mat &grayscale_image) {
  // Compute a block normalization of an grayscale_image

  cv::Mat_<float> float_gs_image;
  grayscale_image.convertTo(float_gs_image, CV_32FC1);

  double min, max;
  cv::minMaxLoc(float_gs_image, &min, &max);

  return (float_gs_image - min) / (max - min);
}

TEST(Convolve, convolves) {
  const std::string image_path = "/home/jacob/repos/slam/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png";
  const cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);

  const cv::Rect section(400, 400, 70, 70);
  const cv::Mat kernel = image(section);

  cv::imshow("image", image);
  cv::waitKey(0);

  const cv::Mat corr = ncc(image, kernel);

  cv::imshow("kern", kernel);
  cv::imshow("image", normalize(corr));
  cv::waitKey(0);
}
}
