#pragma once

#include "eigen.hh"

//%ignore

#include <opencv2/opencv.hpp>

namespace estimation {

double interpolate(const cv::Mat& img, const jcc::Vec2& sample_pt);

}  // namespace estimation