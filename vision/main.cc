#include <iostream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "features.hh"
#include "images.hh"
#include "io.hh"
#include "pyramid.hh"

using Vec2 = Eigen::Vector2f;

struct MappingContext {
  cv::Mat           rgb_image;
  std::vector<Vec2> points;
};

void visualize(const MappingContext &context) {
  cv::Mat visualization_image = context.rgb_image;

  for (const auto &point : context.points) {
    constexpr int    RADIUS = 1;
    const cv::Scalar COLOR(0, 255, 0);
    cv::circle(visualization_image, cv::Point(point.x(), point.y()), RADIUS, COLOR);
  }

  cv::imshow("Visualization", visualization_image);
}

cv::Mat normalize(const cv::Mat &grayscale_image) {
  // Compute a block normalization of an grayscale_image

  cv::Mat_<float> float_gs_image;
  grayscale_image.convertTo(float_gs_image, CV_32FC1);

  double min, max;
  cv::minMaxLoc(float_gs_image, &min, &max);

  return (float_gs_image - min) / (max - min);
}

float variance(const cv::Mat &image) {
  float  mean   = cv::mean(image)[0];
  double sum_sq = 0.0;

  for (int col = 0; col < image.cols; ++col) {
    for (int row = 0; row < image.rows; ++row) {
      const double pixel_val = image.at<float>(row, col);
      sum_sq += (pixel_val - mean) * (pixel_val - mean);
    }
  }

  const double num_elements = image.total();
  const double variance     = sum_sq / num_elements;
  return variance;
}

cv::Mat block_normalize(const cv::Mat &input_image) {
  //

  // OpenCV API sucks
  cv::Mat output = input_image.clone();

  // std::cout << slam::type2str(output.type()) << std::endl;

  constexpr int BLOCK_SIZE = 100;
  for (int start_col = 0; (start_col + BLOCK_SIZE) < output.cols; start_col += BLOCK_SIZE) {
    for (int start_row = 0; (start_row + BLOCK_SIZE) < output.rows; start_row += BLOCK_SIZE) {
      const auto    rect      = cv::Rect(start_col, start_row, BLOCK_SIZE, BLOCK_SIZE);
      const cv::Mat sub_image = output(rect);

      const float  var    = variance(sub_image);
      const float  stddev = var * var;
      const double mean   = cv::mean(sub_image)[0];

      std::cout << stddev << ", " << mean << std::endl;

      output(rect) = normalize((sub_image - mean) / stddev);
    }
  }
  cv::imshow("Merk", output);
  cv::waitKey(0);
  return output;
}

std::vector<Vec2> extract_harris_features(const cv::Mat &image) {
  constexpr double RELATIVE_SCALE     = 1.0;
  constexpr double INV_RELATIVE_SCALE = 1.0 / RELATIVE_SCALE;

  cv::Mat scaled_image;
  cv::resize(image, scaled_image, cv::Size(0, 0), RELATIVE_SCALE, RELATIVE_SCALE);
  const cv::Mat normalized      = normalize(scaled_image);
  const float   mean            = cv::mean(normalized)[0];
  cv::Mat       mean_subtracted = normalized - mean;

  cv::imshow("Gombo", normalized - mean);

  float const *const pixel_ptr = (float *)mean_subtracted.data;

  std::vector<Vec2> locations;
  for (int col = 0; col < mean_subtracted.cols; ++col) {
    for (int row = 0; row < mean_subtracted.rows; ++row) {
      const int   index     = row * (mean_subtracted.cols) + col;
      const float pixel_val = pixel_ptr[index];

      constexpr float NORMALIZED_THRESHOLD = 0.05;
      if (pixel_val > NORMALIZED_THRESHOLD) {
        locations.emplace_back(col * INV_RELATIVE_SCALE, row * INV_RELATIVE_SCALE);
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
    const cv::Mat_<float> normalized_gray_image = normalize(gray_image);

    constexpr int    BLOCK_SIZE     = 2;
    constexpr int    K_SIZE         = 5;
    constexpr double FREE_PARAMETER = 0.001;

    block_normalize(normalized_gray_image);

    cv::Mat harris_image;
    cornerHarris(normalized_gray_image, harris_image, BLOCK_SIZE, K_SIZE, FREE_PARAMETER);
    cv::Mat drawable_image = image;

    const auto pts = extract_harris_features(harris_image);

    MappingContext context;
    context.points    = pts;
    context.rgb_image = image;
    visualize(context);

    const int key = cv::waitKey(0);
    std::cout << key << std::endl;

    if (key == 113 || key == 1048689) {
      break;
    }
  }
}
