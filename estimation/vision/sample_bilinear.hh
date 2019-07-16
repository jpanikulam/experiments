#pragma once

#include "eigen.hh"

#include <opencv2/opencv.hpp>

namespace estimation {

double interpolate_bilinear(const cv::Mat& img, const jcc::Vec2& sample_pt);

uint8_t interpolate_nearest(const cv::Mat& img, const jcc::Vec2& image_point);

}  // namespace estimation