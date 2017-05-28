#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "pyramid.hh"
#include "features.hh"
#include "images.hh"
#include "io.hh"

using Vec2 = Eigen::Vector2f;

struct MappingContext {
  cv::Mat rgb_image;
  std::vector<Vec2> points;
};

void visualize(const MappingContext &context) {
  cv::Mat visualization_image = context.rgb_image;

  for (const auto &point : context.points) {
    constexpr int RADIUS = 1;
    const cv::Scalar COLOR(0, 255, 0);
    cv::circle(visualization_image, cv::Point(point.x(), point.y()), RADIUS, COLOR);
  }

  cv::imshow("Visualization", visualization_image);
}

cv::Mat block_normalize(const cv::Mat &grayscale_image) {
  // Compute a block normalization of an grayscale_image

  cv::Mat_<float> float_gs_image;
  grayscale_image.convertTo(float_gs_image, CV_32FC1); // or CV_32F works (too)

  double min, max;
  cv::minMaxLoc(float_gs_image, &min, &max);

  return (float_gs_image - min) / (max - min);
}

std::vector<Vec2> extract_harris_features(const cv::Mat &image) {
  cv::Mat half_image;
  cv::resize(image, half_image, cv::Size(0, 0), 0.5, 0.5);
  const cv::Mat normalized = block_normalize(half_image);
  const float mean = cv::mean(normalized)[0];
  cv::Mat mean_subtracted = normalized - mean;

  cv::imshow("Gombo", normalized - mean);

  float const *const pixel_ptr = (float *)mean_subtracted.data;

  std::vector<Vec2> locations;
  for (int col = 0; col < mean_subtracted.cols; ++col) {
    for (int row = 0; row < mean_subtracted.rows; ++row) {
      const int index = row * (mean_subtracted.cols) + col;
      const float pixel_val = pixel_ptr[index];

      constexpr float NORMALIZED_THRESHOLD = 0.05;
      if (pixel_val > NORMALIZED_THRESHOLD) {
        locations.emplace_back(col * 2.0, row * 2.0);
      }
    }
  }
  return locations;
}

int main() {
  const auto image_and_depth_locations = slam::get_both();

  for (const auto &location_pair : image_and_depth_locations) {
    const cv::Mat image = cv::imread(location_pair.image);
    const cv::Mat depth = cv::imread(location_pair.depth, CV_LOAD_IMAGE_GRAYSCALE);

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);

    constexpr int BLOCK_SIZE = 2;
    constexpr int K_SIZE = 5;
    constexpr double FREE_PARAMETER = 0.005;

    const cv::Mat_<float> normalized_gray_image = block_normalize(gray_image);

    cv::Mat harris_image;
    cornerHarris(normalized_gray_image, harris_image, BLOCK_SIZE, K_SIZE, FREE_PARAMETER);
    cv::Mat drawable_image = image;

    double min, max;
    cv::minMaxLoc(harris_image, &min, &max);
    cv::imshow("Harris image", (harris_image - min) / (max - min));

    const auto pts = extract_harris_features(harris_image);

    MappingContext context;
    context.points = pts;
    context.rgb_image = image;
    visualize(context);

    const int key = cv::waitKey(0);

    if (key == 113) {
      break;
    }
  }
}
