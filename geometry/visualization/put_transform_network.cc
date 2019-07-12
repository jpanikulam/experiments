#include "geometry/visualization/put_transform_network.hh"

#include "geometry/kinematics/transform_network.hh"
#include "viewer/primitives/simple_geometry.hh"

#include <set>
#include <stack>

#include "eigen_helpers.hh"

namespace geometry {

void put_transform_network(viewer::SimpleGeometry& geo,
                           const TransformNetwork& tfn,
                           const std::string& root_node) {
  std::stack<std::string> to_visit;
  std::set<std::string> visited;
  to_visit.push(root_node);
  visited.insert(root_node);

  std::map<std::string, SE3> root_from_node_map;
  root_from_node_map[root_node] = SE3();

  const auto& edge_map = tfn.edges_from_node_tag();
  while (!to_visit.empty()) {
    const auto parent = to_visit.top();

    to_visit.pop();
    for (const auto& edge : edge_map.at(parent)) {
      const SE3& parent_from_node = edge.second.source_from_destination;
      const auto& root_from_parent = root_from_node_map.at(parent);
      const SE3 root_from_node = root_from_parent * parent_from_node;
      if (std::find(visited.begin(), visited.end(), edge.first) == visited.end()) {
        to_visit.push(edge.first);
        visited.insert(edge.first);

        root_from_node_map[edge.first] = root_from_node;

        geo.add_line({root_from_parent.translation(), root_from_node.translation(),
                      jcc::Vec4(1.0, 1.0, 0.0, 0.8)});
        geo.add_axes({root_from_node});
      } else {
        if (edge.first != parent) {
          geo.add_line({root_from_parent.translation(), root_from_node.translation(),
                        jcc::Vec4(0.0, 1.0, 1.0, 0.8)});
        }
      }
    }
  }
}

}  // namespace geometry
