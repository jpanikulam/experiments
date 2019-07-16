#include "estimation/vision/sample_bilinear.hh"

#include "util/eigen_clip.hh"

namespace estimation {

double interpolate_bilinear(const cv::Mat& img, const jcc::Vec2& sample_pt_unclamped) {
  const jcc::Vec2 lower_bound(0, 0);
  const jcc::Vec2 upper_bound(img.cols - 1, img.rows - 1);
  const jcc::Vec2 sample_pt = jcc::eigen_clip(sample_pt_unclamped, lower_bound, upper_bound);

  const jcc::Vec2i q22 = sample_pt.array().ceil().cast<int>();
  const jcc::Vec2i q11 = sample_pt.array().floor().cast<int>();

  const jcc::Vec2i q12(q11.x(), q22.y());
  const jcc::Vec2i q21(q22.x(), q11.y());

  const double f11 = img.at<uint8_t>(q11.y(), q11.x());
  const double f21 = img.at<uint8_t>(q21.y(), q21.x());
  const double f12 = img.at<uint8_t>(q12.y(), q12.x());
  const double f22 = img.at<uint8_t>(q22.y(), q22.x());

  const MatNd<2, 2> Q = (MatNd<2, 2>() << f11, f12, f21, f22).finished();

  const jcc::Vec2 x_qry(q22.x() - sample_pt.x(), sample_pt.x() - q11.x());
  const jcc::Vec2 y_qry(q22.y() - sample_pt.y(), sample_pt.y() - q11.y());

  constexpr double PX_WIDTH = 1.0;
  const double interpolated = (1.0 / PX_WIDTH) * (x_qry.dot(Q * y_qry));
  return interpolated;
}

uint8_t interpolate_nearest(const cv::Mat& img, const jcc::Vec2& image_point) {
  const int icol = static_cast<int>(image_point.x());
  const int irow = static_cast<int>(image_point.y());

  if (icol < img.cols && irow < img.rows && icol >= 0 && irow >= 0) {
    return img.at<uint8_t>(irow, icol);
  } else {
    return 0u;
  }
}

}  // namespace estimation