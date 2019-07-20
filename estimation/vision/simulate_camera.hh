#pragma once

namespace estimation {
cv::Mat render_at_pose(const SE3& camera_from_fiducial, const cv::Mat& fiducial_image);
//
}  // namespace estimation
