//%ignore

#include "geojson/fast_json.hh"

#include "eigen_helpers.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

namespace geojson {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

void visualize(const std::vector<Feature>& features) {
  //
  auto viewer = viewer::get_window3d("Poopy Sand");
  viewer->set_target_from_world(SE3(SO3::exp(Vec3(-3.1415 * 0.5, 0.0, 0.0)), Vec3::Zero()));
  auto geom = viewer->add_primitive<viewer::SimpleGeometry>();

  Vec2 sum = Vec2::Zero();
  for (const auto& ftr : features) {
    if (!ftr.points_ll.empty()) {
      sum += ftr.points_ll.front();
    }
  }


  const Vec2 mean = sum / features.size();
  for (const auto& ftr : features) {
    viewer::Polygon polygon;

    const Vec3 color = (Vec3::Random() * 0.5).array() + 0.5;
    polygon.color = jcc::augment(color, 1.0);
    polygon.height = std::sqrt(ftr.area) * 0.01;

    for (const Eigen::Vector2d& pt : ftr.points_ll) {
      const Vec2 point_mean_subtracted = pt - mean;
      const Vec3 point_3d = jcc::augment(point_mean_subtracted, 0.0);

      polygon.points.push_back(point_3d);
    }
    geom->add_polygon(polygon);
  }

  viewer->spin_until_step();
}

}  // namespace geojson

int main() {
  const std::string path = "/home/jacob/repos/reduced.json";

  const std::vector<geojson::Feature> features = geojson::read_json(path);
  geojson::visualize(features);
}