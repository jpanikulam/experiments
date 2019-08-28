#pragma once

#include "planning/drifter/drifter_planner.hh"
#include "planning/simulation/sim_viewer_types.hh"

#include "logging/assert.hh"

namespace jcc {
namespace simulation {

struct SimulationState {
  planning::drifter::State state;
};

class SimulationIntegrator {
 public:
  SimulationIntegrator() {
  }

  void add_avoider(const jcc::Vec3& pos, const double radius) {
    desires_.avoids.push_back({pos, radius});
  }

  void clear_avoiders() {
    desires_.avoids.clear();
  }

  void set_target(const jcc::Vec3& target) {
    // TODO: Implement this
    desires_.target = target;
  }

  void toggle_pause() {
    paused_ = !paused_;
  }

  void reset(const Robot& robot) {
    states_.clear();
    scrub_ = 0;

    planning::drifter::State x0;
    x0.x_world = robot.world_from_robot.translation();
    x0.R_world_from_body = robot.world_from_robot.so3();

    // x0.x_world = jcc::Vec3(1.08539, 1.08715, 0);
    // x0.phi = 0.00108898;
    // x0.x_vel = 0.0733424;
    // x0.R_world_from_body = SO3::exp(jcc::Vec3(0, 0, 1.42077));

    // x0 x_vel 0.0733424 phi 0.00108898 R_world_from_body 0 0 1.42077 x_world
    // 1.08539 1.08715 0

    states_.push_back({x0});
  }

  void set_timestep(double step) {
    JASSERT_LT(step, static_cast<int>(states_.size()),
               "Cannot scrub beyond created states");
    JASSERT_GE(step, 0, "Cannot scrub before the start of simulation");
    step_ = step;
  }

  void set_max_speed(double vmax) {
    desires_.vmax = vmax;
  }

  void scrub_to(const int scrub_time) {
    scrub_ = scrub_time;
  }

  int scrub() const {
    return scrub_;
  }

  int max_scrub() const {
    return states_.size() - 1;
  }

  planning::drifter::BehaviorPrimitives desires() const {
    return desires_;
  }

  std::vector<planning::drifter::StateControl> step() {
    const auto x0 = states_[scrub_];

    const auto rst = planning::drifter::plan(x0.state, desires_);

    const auto& x1 = rst[traj_step_size_].state;
    // Only do this when unpaused!

    if (!paused_) {
      while (static_cast<int>(states_.size()) - 1 > scrub_) {
        states_.pop_back();
      }
      states_.push_back({x1});
      scrub_ += 1;
    }
    return rst;
  }

  planning::drifter::State get() const {
    return states_[scrub_].state;
  }

 private:
  double step_ = 0.1;
  int scrub_ = 0;

  int traj_step_size_ = 1;
  int record_rate_ = 1;

  bool paused_ = false;

  std::vector<SimulationState> states_;

  planning::drifter::BehaviorPrimitives desires_;
};

}  // namespace simulation
}  // namespace jcc
