#pragma once

#include "gl_size.hh"
#include "projection.hh"
#include "simple_window.hh"

#include <Eigen/Dense>
#include <sophus/se2.hpp>

#include <vector>

namespace gl_viewer {
namespace {
using Vec2    = Eigen::Vector2d;
using Vec3    = Eigen::Vector3d;
using Vec4    = Eigen::Vector4d;
using Vec2Map = Eigen::Map<const Eigen::Vector2d>;
using Vec3Map = Eigen::Map<const Eigen::Vector3d>;
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

struct Circle {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec2   center;
  double radius;
  Vec4   color = Vec4::Ones();
};

class Window2D final : public SimpleWindow {
  using so2 = Sophus::SO2<double>;
  using se2 = Sophus::SE2<double>;

 public:
  Window2D() {
    view_.camera_height = 3.0;
  }

  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const WindowPoint& mouse_pos) override;
  void on_scroll(const double amount) override;

  void resize(const GlSize& gl_size) override;

  void render() override;

  void add_line(const Line& line) {
    renderables_.lines.push_back(line);
  }

  void add_ray(const Ray& ray) {
    renderables_.rays.push_back(ray);
  }

  void add_circle(const Circle& circle) {
    renderables_.circles.push_back(circle);
  }

  void clear() {
    renderables_.clear();
  }

 private:
  struct Renderables {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Line>   lines;
    std::vector<Ray>    rays;
    std::vector<Circle> circles;

    void clear() {
      lines.clear();
      rays.clear();
      circles.clear();
    }
  };

  struct View2D {
    // Default identity
    se2 camera_pose;

    // Height from view plane
    double camera_height  = 5.0;
    double dcamera_height = 0.0;

    double zoom  = 0.1;
    double dzoom = 0.0;

    // Time
    double last_update_time = 0.0;

    // Tangent vector; Left tangent space or gtfo
    Vec3 velocity;

    // Apply the transformation
    void apply();

    void simulate();
  };

  void apply_keys_to_view();

  void draw_renderables(const Renderables& renderables) const;

  Vec2 mouse_direction_ = Vec2::Zero();

  //
  // Track some window properties
  //

  View2D     view_;
  Projection projection_;

  Renderables renderables_;

  GlSize gl_size_ = GlSize(640, 640);
};
}
