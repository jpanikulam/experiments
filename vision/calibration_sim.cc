#include "image_align.hh"

#include "out.hh"
#include "util/clamp.hh"

#include "viewer/primitives/frame.hh"
#include "viewer/primitives/image.hh"
#include "viewer/primitives/image_frame.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"
#include "viewer/window_manager.hh"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <ostream>

using Vec2   = Eigen::Vector2d;
using Vec3   = Eigen::Vector3d;
using Vec32i = Eigen::Matrix<uint8_t, 32, 1>;

constexpr double DISTANCE = 500.0;

namespace slam {

std::vector<Vec32i> eigenize_descriptors(const cv::Mat& descriptors) {
  std::vector<Vec32i> eigen_descriptors(descriptors.rows);
  for (int k = 0; k < descriptors.rows; ++k) {
    for (int j = 0; j < descriptors.cols; ++j) {
      eigen_descriptors[k](j) = descriptors.at<uint8_t>(k, j);
    }
  }
  return eigen_descriptors;
}

std::vector<Vec32i> compute_orb_features(const cv::Mat& image, Out<std::vector<cv::KeyPoint>> key_points) {
  constexpr int   N_FEATURES     = 500;
  constexpr float SCALE_FACTOR   = 1.2f;
  constexpr int   N_LEVELS       = 8;
  constexpr int   EDGE_THRESHOLD = 12;
  constexpr int   FIRST_LEVEL    = 0;
  const cv::ORB   orb_detector(N_FEATURES, SCALE_FACTOR, N_LEVELS, EDGE_THRESHOLD, FIRST_LEVEL);

  cv::Mat descriptors;
  orb_detector(image, cv::Mat(), is_out(*key_points), is_out(descriptors));
  return eigenize_descriptors(descriptors);
}

void try_align(const std::vector<cv::KeyPoint>& observed_pts,
               const std::vector<cv::KeyPoint>& calibration_pts,
               const cv::Mat&                   calibration_image_color) {
  constexpr double FX = 1.0;
  constexpr double FY = 1.0;
  constexpr double CX = -1600 * 0.5;
  constexpr double CY = -1067 * 0.5;

  const CameraModel model(FX, FY, CX, CY);

  if (observed_pts.size() < 5u) {
    return;
  }

  std::vector<Vec2> observed;
  std::vector<Vec3> object;
  std::vector<Vec2> observed_effective;

  // const SE3 distortion_from_object(SO3::exp(Vec3(0.0, 0.0, 0.5)), Vec3(0.0, 1.5, 0.4));
  const SE3 distortion_from_object(SO3::exp(Vec3(0.0, 0.0, -0.0)), Vec3(-500.0, -250.6, -100.0));

  for (const auto& calibration_pt : calibration_pts) {
    object.emplace_back(calibration_pt.pt.x, calibration_pt.pt.y, DISTANCE);
    observed.emplace_back(model.project(distortion_from_object * object.back()));
    observed_effective.emplace_back(model.project(distortion_from_object * object.back()) - Vec2(CX, CY));
  }

  ImageAligner              aligner;
  const Sophus::SE3<double> initial;
  const auto                result             = aligner.standard_align(model, initial, observed, object);
  const SE3                 camera_from_object = result.delta.inverse();

  auto win  = gl_viewer::get_window3d("Window A");
  auto geom = std::make_shared<gl_viewer::SimpleGeometry>();
  geom->add_axes({SE3(), 10.0});

  // Draw the object
  {
    auto contained_geom = std::make_shared<gl_viewer::SimpleGeometry>();
    auto image_frame    = std::make_shared<gl_viewer::ImageFrame>(SE3(), 1067, 1600, 1.0);

    contained_geom->add_points({object, Eigen::Vector4d(0.0, 0.8, 0.1, 0.7), 25.0});
    image_frame->add_primitive(contained_geom);
    win->add_primitive(image_frame);
  }

  // Draw a representation of what we see
  {
    auto contained_geom = std::make_shared<gl_viewer::SimpleGeometry>();
    contained_geom->add_points2d({observed_effective, Eigen::Vector4d(0.7, 0.2, 0.1, 0.7), 25.0, FX});
    auto image_frame = std::make_shared<gl_viewer::ImageFrame>(camera_from_object, 1067, 1600, 1.0);
    image_frame->add_primitive(contained_geom);
    win->add_primitive(image_frame);

    // That's image scale sqd
    auto calibration_img_primitive = std::make_shared<gl_viewer::Image>(calibration_image_color, 0.002, 0.4);
    auto calibration_img_primitive_frame =
        std::make_shared<gl_viewer::Frame>(SE3(SO3(), Vec3(0.0, 0.0, FX + 0.01)) * camera_from_object);
    calibration_img_primitive_frame->add_primitive(calibration_img_primitive);
    win->add_primitive(calibration_img_primitive_frame);
  }

  if (result.success) {
    geom->add_axes({camera_from_object});

    for (const Vec2& pt : observed) {
      const Vec3 ray_direction          = model.unproject(pt);
      const Vec3 ray_direction_adjusted = Vec3(ray_direction.x(), ray_direction.y(), ray_direction.z());

      geom->add_ray({camera_from_object.translation(), camera_from_object.so3() * ray_direction_adjusted, 1500.0,
                     Eigen::Vector4d(1.0, 0.0, 0.0, 0.6)});
    }

    win->add_primitive(geom);
    win->spin_until_step();
  }
}
}

int main() {
  const std::string calibration_image_filename =
      "/home/jacob/repos/slam/data/calibration/domestic_goat_kid_in_capeweed.jpg";

  const cv::Mat calibration_image_color = cv::imread(calibration_image_filename);
  std::cout << calibration_image_color.size() << std::endl;
  cv::Mat calibration_image;
  cv::cvtColor(calibration_image_color, is_out(calibration_image), CV_BGR2GRAY);

  auto win = gl_viewer::get_window3d("Window A");

  {
    auto calibration_img_primitive = std::make_shared<gl_viewer::Image>(calibration_image_color, 1.0);
    auto calibration_img_primitive_frame =
        std::make_shared<gl_viewer::Frame>(SE3(SO3(), Vec3(0.0, 0.0, DISTANCE + 0.1)));
    calibration_img_primitive_frame->add_primitive(calibration_img_primitive);
    win->add_primitive(calibration_img_primitive_frame);
  }

  std::vector<cv::KeyPoint> calibration_key_points;
  const auto calibration_descriptors = slam::compute_orb_features(calibration_image, out(calibration_key_points));
  slam::try_align(calibration_key_points, calibration_key_points, calibration_image_color);
}