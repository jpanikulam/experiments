#include "planning/visualization/put_body.hh"

#include "geometry/visualization/put_circle.hh"
#include "geometry/shapes/circle.hh"

#include "eigen.hh"
#include "sophus.hh"

#include <queue>
#include <unordered_map>

namespace planning {

void put_body(viewer::SimpleGeometry& geo, const Body& body, const jcc::Vec4& color) {
  std::queue<int> q;
  q.emplace(-1);

  std::unordered_map<int, SE3> world_from_joint;
  world_from_joint[-1] = SE3();

  while (!q.empty()) {
    const int parent = q.front();
    q.pop();

    const auto world_from_parent = world_from_joint.at(parent);

    constexpr double JOINT_RADIUS_M = 0.1;
    if (body.parent_to_children().count(parent) == 0) {
      const jcc::Vec4 color(1.0, 0.1, 0.1, 0.8);
      geo.add_sphere({world_from_parent.translation(), JOINT_RADIUS_M, color});
      continue;
    } else {
      const jcc::Vec4 color(0.1, 1.0, parent == -1 ? 0.8 : 0.1, 0.8);
      geo.add_sphere({world_from_parent.translation(), JOINT_RADIUS_M, color});
    }

    for (const auto& child : body.parent_to_children().at(parent)) {
      const auto& joint = body.joints().at(child.joint);

      const jcc::Vec3 log_joint = jcc::Vec3(0.0, 0.0, joint.angle);
      const SE3 joint_place_from_joint = SE3(SO3::exp(log_joint), jcc::Vec3::Zero());

      world_from_joint[child.joint] =
          world_from_parent * child.parent_from_joint * joint_place_from_joint.inverse();

      geo.add_line({world_from_parent.translation(),
                    world_from_joint[child.joint].translation(), color});

      const geometry::shapes::Circle joint_circle{
          world_from_joint[child.joint].translation(),
          world_from_joint[child.joint].so3() * jcc::Vec3::UnitZ(), JOINT_RADIUS_M};
      geometry::visualization::put_circle(geo, joint_circle);

      q.emplace(child.joint);
    }
  }
}

}  // namespace planning