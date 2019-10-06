#include "viewer/colors/colors.hh"
#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "viewer/window_3d.hh"

#include "eigen_helpers.hh"
#include "numerics/gauss_newton.hh"
#include "util/between.hh"

namespace jcc {

struct SimulationState {
  std::vector<jcc::Vec2> points;
};

VecXd to_vector(const SimulationState& mesh) {
  VecXd vec(mesh.points.size() * 2);
  for (std::size_t k = 0; k < mesh.points.size(); ++k) {
    vec.segment(k * 2, 2) = mesh.points[k];
  }
  return vec;
}

SimulationState from_vector(const VecXd& vec) {
  SimulationState out_mesh;
  out_mesh.points.resize(vec.rows() / 2);
  for (int k = 0; k < (vec.rows() / 2); ++k) {
    out_mesh.points[k] = vec.segment(k * 2, 2);
  }
  return out_mesh;
}

class StressSimulation {
 public:
  StressSimulation(const SimulationState& state0,
                   const std::vector<std::pair<int, int>>& edges) {
    initial_state_ = state0;
    edges_ = edges;
  }

  jcc::Vec2 compute_edge_force(const SimulationState& state, const int edge_id) const {
    constexpr double E = 1000.0;

    const int a = edges_[edge_id].first;
    const int b = edges_[edge_id].second;
    const jcc::Vec2 p_a_now = state.points[a];
    const jcc::Vec2 p_a_0 = initial_state_.points[a];

    const jcc::Vec2 p_b_now = state.points[b];
    const jcc::Vec2 p_b_0 = initial_state_.points[b];

    const jcc::Vec2 direction = (p_a_now - p_b_now).normalized();
    const double l_0 = (p_a_0 - p_b_0).norm();
    const double l_now = (p_a_now - p_b_now).norm();

    const double deformation = ((l_now - l_0) / l_0);

    return (direction * (E * deformation));
  }

  VecXd compute_error(const SimulationState& state,
                      const std::vector<jcc::Vec2>& exogeneous_force) const {
    std::vector<jcc::Vec2> net_forces(state.points.size(), jcc::Vec2::Zero());
    for (std::size_t k = 0; k < state.points.size(); ++k) {
      net_forces[k] += exogeneous_force[k];
    }

    for (std::size_t k = 0; k < edges_.size(); ++k) {
      if (edges_[k].second == -1) {
        continue;
      }
      const jcc::Vec2 force = compute_edge_force(state, k);
      net_forces[edges_[k].first] += force;
      net_forces[edges_[k].second] -= force;
    }

    for (std::size_t k = 0; k < edges_.size(); ++k) {
      if (edges_[k].second == -1) {
        net_forces[edges_[k].first] = 1e4 * (state.points[edges_[k].first] -
                                             initial_state_.points[edges_[k].first]);
        continue;
      }
    }

    VecXd error(state.points.size() * 2);
    // error.segment(0, 2) = 1000.0 * (state.points[0] - initial_state_.points[0]);

    for (std::size_t k = 0; k < net_forces.size(); ++k) {
      error.segment(k * 2, 2) = net_forces[k];
    }

    /*for (std::size_t k = 1; k < edges_.size(); ++k) {
      const int a = edges_[k].first;
      const int b = edges_[k].second;
      const jcc::Vec2 p_a_now = state.points[a];
      const jcc::Vec2 p_a_0 = initial_state_.points[a];

      const jcc::Vec2 p_b_now = state.points[b];
      const jcc::Vec2 p_b_0 = initial_state_.points[b];

      const double l_0 = (p_a_0 - p_b_0).norm();
      const double l_now = (p_a_now - p_b_now).norm();
      const double strain = l_now / l_0;

      error[k] = (E * strain) - (exogeneous_force[k] * AREA);
    }*/
    return error;
  }

 private:
  std::vector<std::pair<int, int>> edges_;
  SimulationState initial_state_;
  int n_edges_ = 0;
};

void put_sim(viewer::SimpleGeometry& geo,
             const SimulationState& state,
             const std::vector<std::pair<int, int>>& edges,
             const std::vector<jcc::Vec2>& forces) {
  constexpr double Z_HEIGHT = 1.0;

  int k = 0;
  for (const auto& edge : edges) {
    const int i = edge.first;
    const int j = edge.second;

    jcc::Vec4 point_color(0.0, 1.0, 0.0, 1.0);
    if (j != -1) {
      // const jcc::Vec2 dir = (state.points[i] - state.points[j]).normalized();
      // const double force = forces[k].dot(dir) / 1.0;
      // const jcc::Vec4 force_color(
      // force > 0.0 ? force : 0.0, force < 0.0 ? -force : 0.0, 0.0, 1.0);

      geo.add_line({jcc::augment(state.points[i], Z_HEIGHT),
                    jcc::augment(state.points[j], Z_HEIGHT)});
    } else {
      point_color = jcc::Vec4(1.0, 0.0, 0.0, 1.0);
    }
    geo.add_point({jcc::augment(state.points[i], Z_HEIGHT), point_color, 4.0});
    ++k;
  }

  for (std::size_t j = 0; j < forces.size(); ++j) {
    geo.add_line({jcc::augment(state.points[j], Z_HEIGHT),
                  jcc::augment(jcc::Vec2(state.points[j] + forces[j]), Z_HEIGHT),
                  jcc::Vec4(1.0, 0.0, 0.0, 1.0)});
  }

  geo.flip();
}

int lex(int x, int y, int row_size) {
  return (x + (y * row_size));
}

jcc::Vec2i delex(int u, int row_size) {
  jcc::Vec2i xy;
  xy.x() = (u % row_size);
  xy.y() = u - xy.x();
  return xy;
}

void go() {
  const auto view = viewer::get_window3d("Mr. Stress");

  const auto bgnd = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{Vec3::UnitZ(), 0.0};
  bgnd->add_plane({ground});
  bgnd->flip();

  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  /*  state.points.push_back(jcc::Vec2(1.0, 1.0));
    state.points.push_back(jcc::Vec2(3.0, 1.0));
    state.points.push_back(jcc::Vec2(5.0, 1.0));
    state.points.push_back(jcc::Vec2(4.0, 1.0));
    state.points.push_back(jcc::Vec2(5.0, 1.0));
    state.points.push_back(jcc::Vec2(6.0, 1.0));

    std::vector<std::pair<int, int>> edges;
    edges.push_back({0, -1});
    edges.push_back({0, 1});
    edges.push_back({1, 2});
    edges.push_back({2, 3});
    edges.push_back({3, 4});
    edges.push_back({4, 5});

    std::vector<jcc::Vec2> forces;
    forces.push_back(jcc::Vec2(15.0, 0.0));
    forces.push_back(jcc::Vec2(15.0, 0.0));
    forces.push_back(jcc::Vec2(15.0, 0.0));
    forces.push_back(jcc::Vec2(1.0, 0.0));
    forces.push_back(jcc::Vec2(1.0, 0.0));
    forces.push_back(jcc::Vec2(1.0, 0.0));
  */

  SimulationState state;
  std::vector<std::pair<int, int>> edges;
  std::vector<jcc::Vec2> forces;

  constexpr int min = 0;
  constexpr int max = 8;

  int ct = (max - min);

  for (int i = min; i < max; ++i) {
    for (int j = min; j < max; ++j) {
      state.points.push_back(jcc::Vec2(i, j));

      if (jcc::between(i, min, max + 1)) {
        edges.push_back({lex(i - 1, j, ct), lex(i, j, ct)});
      } else {
        edges.push_back({lex(i, j, ct), -1});
      }
      if (jcc::between(j, min, max + 1)) {
        edges.push_back({lex(i, j - 1, ct), lex(i, j, ct)});
      } else {
        edges.push_back({lex(i, j, ct), -1});
      }

      if (i == 3 && j == 3) {
        forces.push_back(jcc::Vec2(-51.0, -51.0));
      } else {
        forces.push_back(jcc::Vec2(0.0, 0.0));
      }
    }
  }

  put_sim(*geo, state, edges, forces);
  view->spin_until_step();

  const StressSimulation stress_sim(state, edges);
  const auto error_fnc = [stress_sim, forces](const VecXd& vec) {
    const SimulationState now = from_vector(vec);
    return stress_sim.compute_error(now, forces);
  };

  std::cout << forces.size() << std::endl;
  std::cout << edges.size() << std::endl;

  const auto soln = numerics::gauss_newton_minimize<50, 50>(error_fnc, to_vector(state));
  std::cout << "Solved" << std::endl;

  std::vector<jcc::Vec2> final_forces(state.points.size(), jcc::Vec2::Zero());
  for (std::size_t k = 0; k < edges.size(); ++k) {
    if (edges[k].second == -1) {
      continue;
    }
    const jcc::Vec2 f = stress_sim.compute_edge_force(from_vector(soln.solution), k);

    final_forces[edges[k].first] += f;
    final_forces[edges[k].second] -= f;

    // std::cout << final_forces[k].transpose() << std::endl;
  }
  std::cout << "Putting" << std::endl;
  put_sim(*geo, from_vector(soln.solution), edges, final_forces);
  view->spin_until_step();
}
}  // namespace jcc

int main() {
  jcc::go();
}
