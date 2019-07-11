#include "viewer/colors/colors.hh"
#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/kinematics/transform_network.hh"

#include <stack>
#include <set>

#include "eigen_helpers.hh"

namespace geometry {

void draw() {
  const auto view = viewer::get_window3d("Beams Baby");
  const auto bgnd = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  bgnd->add_plane({ground, 1.0, jcc::Vec4(0.8, 0.8, 0.8, 0.4)});
  bgnd->flip();

  view->add_toggle_hotkey("phase", false, 'P');

  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto text = view->add_primitive<viewer::Ui2d>();

  TransformNetwork tfn;
  tfn.add_edge("world", "fiducial", SE3(SO3(), jcc::Vec3(1.0, 0.0, 0.1)));

  tfn.add_edge("fiducial", "camera", SE3(SO3(), jcc::Vec3(1.0, 0.0, 1.1)));
  tfn.add_edge("jet", "camera",
               SE3(SO3::exp(jcc::Vec3(0.2, 0.2, -0.5)), jcc::Vec3(1.0, 0.0, -1.0)));

  tfn.add_edge("camera", "IMU-1",
               SE3(SO3::exp(jcc::Vec3(1.2, 0.2, -0.5)), jcc::Vec3(0.0, 1.0, -1.0)));
  tfn.add_edge("camera", "IMU-2",
               SE3(SO3::exp(jcc::Vec3(1.2, 0.2, -0.5)), jcc::Vec3(0.0, -1.0, -1.0)));

  const SE3 imu2_from_imu1 = tfn.source_from_destination("IMU-2", "IMU-1");
  tfn.add_edge("IMU-2", "IMU-1", imu2_from_imu1);

  tfn.add_edge("jet", "servos",
               SE3(SO3::exp(jcc::Vec3(-0.2, 0.2, -0.5)), jcc::Vec3(1.0, 1.0, -1.0)));


  std::stack<std::string> to_visit;
  std::set<std::string> visited;

  const std::string root_node = "world";
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

        geo->add_line({root_from_parent.translation(), root_from_node.translation(),
                       jcc::Vec4(1.0, 1.0, 0.0, 0.8)});
        geo->add_axes({root_from_node});
      } else {
        if (edge.first != parent) {
          geo->add_line({root_from_parent.translation(), root_from_node.translation(),
                         jcc::Vec4(0.0, 1.0, 1.0, 0.8)});
        }
      }
    }
  }
  geo->flip();
  view->spin_until_step();
}
}  // namespace geometry

int main() {
  geometry::draw();
}
