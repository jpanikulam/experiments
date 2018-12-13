#include "geometry/visualization/put_collada.hh"

#include <stack>

namespace geometry {
namespace visualization {

void put_collada(viewer::SimpleGeometry& geo,
                 const geometry::import::ColladaModel& model,
                 const SE3 world_from_root) {
  std::map<std::string, SE3> world_from_node;
  world_from_node[model.root()] = world_from_root;

  std::stack<std::string> to_visit;
  to_visit.push(model.root());

  const auto& adj = model.adjacency();
  const auto& meshes = model.meshes();

  while (!to_visit.empty()) {
    const auto key = to_visit.top();
    to_visit.pop();

    // geo.add_triangle_mesh({mesh, world_from_node[key], color, true, 3.0, false});
    const auto& mesh = meshes.at(key);
    const jcc::Vec4 color(0.7, 0.7, 0.7, 0.7);
    geo.add_triangle_mesh({mesh, world_from_node.at(key), color, true, 3.0, false});

    if (adj.count(key) == 0) {
      continue;
    }

    const SE3 world_from_parent = world_from_node.at(key);

    for (const auto& edge : adj.at(key)) {
      world_from_node[edge.child_name] =
          world_from_parent * edge.child_from_parent.inverse();
      to_visit.push(edge.child_name);
    }
  }
}
}  // namespace visualization
}  // namespace geometry