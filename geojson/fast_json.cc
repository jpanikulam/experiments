#include "fast_json.hh"

#include <tao/json.hpp>

namespace geojson {
namespace {
using Vec2 = Eigen::Vector2d;
}  // namespace

std::vector<Feature> read_json(const std::string& path) {
  const tao::json::value json_data = tao::json::parse_file(path);

  std::vector<Feature> features;
  features.reserve(json_data.get_array().size());

  int k = 0;
  for (const auto& val : json_data.get_array()) {
    ++k;

    features.push_back({});
    Feature& feature = features.back();

    if (val.at("geometry").at("type") != "Polygon") {
      continue;
    }

    const auto& coordinates = val.at("geometry").at("coordinates").get_array();
    feature.points_ll.reserve(coordinates.size());

    for (const auto& point : coordinates.at(0).get_array()) {
      const auto& point_vec = point.get_array();
      const Vec2 vec(point_vec.at(0).as<double>(), point_vec.at(1).as<double>());
      feature.points_ll.push_back(vec);
    }
  }

  return features;
}
}  // namespace geojson