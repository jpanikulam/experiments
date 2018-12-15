#include "planning/jet/jet_model.hh"

#include "geometry/visualization/put_collada.hh"
#include "viewer/primitives/simple_geometry.hh"

namespace planning {
namespace jet {

void JetModel::insert(viewer::SceneTree& tree) const {
  // const SE3 world_from_jet = SE3(jet.R_world_from_body, jet.x);
  /*
    std::stack<std::string> to_visit;
    to_visit.push(model_.root());

    const auto& adj = model_.adjacency();
    const auto& meshes = model_.meshes();
    const auto& colors = model_.colors();

    for (const auto& col : colors) {
      std::cout << col.first << " < " << col.second.transpose() << " > " << std::endl;
    }

    while (!to_visit.empty()) {
      const auto key = to_visit.top();
      to_visit.pop();

      if (adj.count(key) == 0) {
        continue;
      }

      const std::string target_key = key == model_.root() ? "root" : key;
      for (const auto& edge : adj.at(key)) {
        to_visit.push(edge.child_name);

        const auto geo = std::make_shared<viewer::SimpleGeometry>();

        const auto& mesh = meshes.at(key);
        jcc::Vec4 color = jcc::Vec4::Ones() * 0.7;
        std::cout << "key: " << key << std::endl;
        if (colors.count(key)) {
          std::cout << "\t Inserting" << std::endl;
          color = colors.at(key);
        }
        geo->add_triangle_mesh({mesh, SE3(), color, true, 3.0, false});
        geo->add_point({edge.child_from_parent.inverse().translation()});

        // TODO WHEN COLLADA IS SWITCHED TO FRAMETREE, THIS MUST BE FIXED
        const auto& parent_from_child = edge.child_from_parent.inverse();
        // std::cout << "Adding primitive: " << target_key << " -> " << edge.child_name <<
    std::endl;

        geo->flip();
        tree.add_primitive(target_key, parent_from_child, edge.child_name, geo);

      }
    }*/

  const SE3 world_from_jet = SE3();

  const auto ggeo = std::make_shared<viewer::SimpleGeometry>();
  auto& geo = *ggeo;
  tree.add_primitive("root", SE3(), "poopy", ggeo);

  std::map<std::string, SE3> world_from_node;
  world_from_node[model_.root()] = world_from_jet;

  std::stack<std::string> to_visit;
  to_visit.push(model_.root());

  const auto& adj = model_.adjacency();
  const auto& meshes = model_.meshes();
  const auto& colors = model_.colors();

  while (!to_visit.empty()) {
    const auto key = to_visit.top();
    to_visit.pop();

    const auto& mesh = meshes.at(key);
    if (colors.count(key)) {
      const jcc::Vec4 color = colors.at(key);
      geo.add_triangle_mesh({mesh, world_from_node.at(key), color, true, 3.0, false});
    }
    geo.flush();

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
  geo.flush();
}

}  // namespace jet
}  // namespace planning