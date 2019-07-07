#include "estimation/vision/sample_bilinear.hh"

namespace estimation {

double interpolate(const cv::Mat& img, const jcc::Vec2& sample_pt) {
  const jcc::Vec2i q22 = sample_pt.ceil().cast<int>;
  const jcc::Vec2i q11 = sample_pt.floor().cast<int>;

  const jcc::Vec2i q12(q11.x(), q22.y());
  const jcc::Vec2i q21(q22.x(), q11.y());

  const double f11 = img.at<uint8_t>(q11.x(), q11.y());
  const double f21 = img.at<uint8_t>(q21.x(), q21.y());
  const double f12 = img.at<uint8_t>(q12.x(), q12.y());
  const double f22 = img.at<uint8_t>(q22.x(), q22.y());

  Eigen::Matrix<double, 2, 2> Q;
  Q << f11, f12, f21, f22;

  const jcc::Vec2 x_qry(q22.x() - sample_pt.x(), sample_pt.x() - q11.x());
  const jcc::Vec2 y_qry(q22.y() - sample_pt.y(), sample_pt.y() - q11.y());

  constexpr double PX_WIDTH = 1.0;
  const double interpolated = (1.0 / PX_WIDTH) * (x_qry.dot(Q * y_qry));
  return interpolated;
}

}  // namespace estimation