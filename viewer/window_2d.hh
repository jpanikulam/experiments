#pragma once

#include "simple_window.hh"

#include <Eigen/Dense>
#include <sophus/se2.hpp>

#include <vector>

namespace gl_viewer {
namespace {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
}

struct Line {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2 start;
  Vec2 end;
  Vec4 color = Vec4::Ones();
};

struct Ray {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2 origin;
  Vec2 direction;
  Vec4 color = Vec4::Ones();
};

class Window2D final : public SimpleWindow {
  using so2 = Sophus::SO2<double>;
  using se2 = Sophus::SE2<double>;

 public:
  Window2D() {
    // view_.camera_pose = se2::exp(Vec3(0.0, ))
    view_.camera_height = 3.0;
  }

  void on_key(int key, int scancode, int action, int mods) override;
  void render() override;

  void add_line(const Line& line) {
    renderables_.lines.push_back(line);
  }

  void add_ray(const Ray& ray) {
    renderables_.rays.push_back(ray);
  }

 private:
  struct Renderables {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Line> lines;
    std::vector<Ray>  rays;
  };

  struct View2D {
    // default identity
    se2 camera_pose;

    // Height from view plane
    double camera_height  = 0.0;
    double dcamera_height = 0.0;

    double last_update_time = 0.0;

    // Tangent vector : left tangent space
    Vec3 velocity;

    // Apply the transformation
    void apply();

    void simulate();
  };

  void apply_keys_to_view();

  View2D view_;

  Renderables renderables_;

  void draw_renderables(const Renderables& renderables) const;
};
}
