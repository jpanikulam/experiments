#include "eigen.hh"

#include "geometry/intersection/sphere_plane.hh"
#include "geometry/shapes/circle.hh"
#include "geometry/shapes/halfspace.hh"
#include "geometry/shapes/sphere.hh"
#include "geometry/visualization/put_circle.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include <map>
#include <queue>
#include <unordered_map>
#include <utility>

namespace planning {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

struct Joint {
  double damping = 0.0;
  double angle = 0.0;
  double velocity = 0.0;
};

struct Link {
  int joint;
  SE3 parent_from_joint;
};

// Compute force at each constrained location by estimating lagrange multiplier
//
// TODO:
// - ? Make all joints rotate -- so invert everything
//    - But that would not be ideal for a few reasons
//
// - Give every joint a "torque" capability
//   - This way, we can apply the force of gravity
//
class Body {
 public:
  using LinkGraph = std::unordered_map<int, std::vector<Link>>;
  Body(const SE3& root_from_world) {
    root_from_world_ = root_from_world;
  }

  //
  // Body Manipulation
  //

  int attach_link(int parent, const SE3& child_from_parent, const Joint& joint) {
    joints_[current_id_] = joint;
    parent_to_children_[parent].push_back({current_id_, child_from_parent});

    return current_id_++;
  }

  //
  // Simulation
  //

  // Simulate, purely kinematic
  void coarse_simulate(const double dt) {
    for (auto& joint : joints_) {
      joint.second.angle += joint.second.velocity * dt;
      joint.second.velocity *= std::pow(1.0 - joint.second.damping, dt);
    }
  }

  //
  // Accessors
  //

  const std::unordered_map<int, Joint>& joints() const {
    return joints_;
  }
  const LinkGraph& parent_to_children() const {
    return parent_to_children_;
  }

  SE3 root_from_world() const {
    return root_from_world_;
  }

 private:
  std::unordered_map<int, Joint> joints_;
  LinkGraph parent_to_children_;
  int current_id_ = 1;

  SE3 root_from_world_;
};

Body make_walker() {
  Body body(SE3(SO3(), Vec3(1.0, 0.0, 1.0)));

  const SE3 joint_from_body_1(SO3::exp(Vec3(0.9, 0.0, 0.0)), Vec3(1.0, 0.0, 0.0));
  const SE3 joint_from_body_2(SO3::exp(Vec3(0.0, 0.8, 0.0)), Vec3(1.0, 2.0, 0.0));

  int n = body.attach_link(0, joint_from_body_1, {0.0, 0.0, 0.1});
  {
    n = body.attach_link(n, joint_from_body_2, {0.0, 0.0, 0.1});
    body.attach_link(n, joint_from_body_2, {0.0, 0.0, 0.0});
  }

  {
    n = body.attach_link(0, joint_from_body_2, {0.0, 0.0, 0.3});
    body.attach_link(n, joint_from_body_2, {0.0, 0.0, 0.0});
  }

  return body;
}

void put_body(viewer::SimpleGeometry& geo, const Body& body) {
  std::queue<int> q;
  q.emplace(0);

  std::map<int, SE3> world_from_joint;
  world_from_joint[0] = SE3();

  while (!q.empty()) {
    const int parent = q.front();
    q.pop();

    const auto world_from_parent = world_from_joint.at(parent);

    constexpr double JOINT_RADIUS_M = 0.1;
    if (body.parent_to_children().count(parent) == 0) {
      geo.add_sphere(
          {world_from_parent.translation(), JOINT_RADIUS_M, Vec4(1.0, 0.1, 0.1, 0.8)});
      continue;
    } else {
      geo.add_sphere(
          {world_from_parent.translation(), JOINT_RADIUS_M, Vec4(0.1, 1.0, 0.1, 0.8)});
    }
    for (const auto& child : body.parent_to_children().at(parent)) {
      const auto& joint = body.joints().at(child.joint);

      const SE3 joint_place_from_joint =
          SE3(SO3::exp(Vec3(0.0, 0.0, joint.angle)), Vec3::Zero());

      world_from_joint[child.joint] =
          world_from_parent * child.parent_from_joint * joint_place_from_joint.inverse();

      geo.add_line(
          {world_from_parent.translation(), world_from_joint[child.joint].translation()});

      const geometry::shapes::Circle joint_circle{
          world_from_joint[child.joint].translation(),
          world_from_joint[child.joint].so3() * Vec3::UnitZ(), JOINT_RADIUS_M};
      geometry::visualization::put_circle(geo, joint_circle);

      q.emplace(child.joint);
    }
  }
}

void walk() {
  const auto view = viewer::get_window3d("Mr. Walks, walks");
  view->set_target_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
  const auto geo = view->add_primitive<viewer::SimpleGeometry>();
  view->set_continue_time_ms(20);

  constexpr double dt = 0.1;
  auto walker = make_walker();

  for (double t = 0.0; t < 100.0; t += dt) {
    const geometry::shapes::Plane ground{Vec3::UnitZ(), 0.0};
    geo->add_plane({ground});

    const geometry::shapes::Sphere sphere{Vec3(5.0, 5.0, std::cos(t)), 1.0};
    geo->add_sphere({sphere.center, sphere.radius});

    const auto intersection =
        geometry::intersection::sphere_plane_intersection(sphere, ground);

    if (intersection) {
      geometry::visualization::put_circle(*geo, *intersection);
    }

    put_body(*geo, walker);
    walker.coarse_simulate(dt);

    geo->flip();
    view->spin_until_step();
  }
}

}  // namespace planning

int main() {
  planning::walk();
}