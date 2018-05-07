#include "eigen.hh"

#include <string>
#include <vector>

namespace geojson {

struct Feature {
  std::string global_id;
  std::string pin;

  std::vector<Eigen::Vector2d> points_ll;
};

std::vector<Feature> read_json(const std::string& path);

}  // namespace geojson