#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "planning/simulation/sim_viewer.hh"
#include "planning/simulation/sim_viewer_placement.hh"
#include "util/environment.hh"

#include "geometry/import/read_stl.hh"
#include "geometry/plane.hh"
#include "viewer/window_manager.hh"

#include "eigen_helpers.hh"
#include "viewer/colors/viridis.hh"

#include "third_party/imgui/imgui.h"

#include "planning/drifter/drifter_ui_elements.hh"

// TODO
#include "geometry/visualization/put_bounding_box.hh"
#include "util/heap.hh"

namespace jcc {
namespace simulation {

namespace {

jcc::Vec4 get_color(const Element &element) {
  const std::map<ElementType, jcc::Vec4> color_map = {
      {ElementType::NoGoZone, jcc::Vec4(1.0, 0.0, 0.0, 1.0)},
      {ElementType::GoZone, jcc::Vec4(0.0, 1.0, 0.0, 1.0)},
      {ElementType::LookZone, jcc::Vec4(0.0, 0.0, 1.0, 1.0)}};

  const std::map<SelectionState, jcc::Vec4> scaling = {
      {SelectionState::Selected, jcc::Vec4(1.0, 1.0, 1.0, 1.0)},
      {SelectionState::Hovered, jcc::Vec4(0.8, 0.8, 0.8, 0.8)},
      {SelectionState::None, jcc::Vec4(0.7, 0.7, 0.7, 0.8)}};

  const jcc::Vec4 color = color_map.at(element.element_type).array() *
                          scaling.at(element.selection_state).array();
  return color;
}

}  // namespace

SimViewer::SimViewer() {
  const auto background = this->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};
  background->add_plane({ground});
  background->flip();

  set_show_axes(false);

  //
  // Load assets
  //

  assets_["One Shelf+X"] =
      build_asset(Environment::repo_path() + "planning/simulation/models/one_shelf.stl");
  assets_["One Shelf-X"] = build_asset(Environment::repo_path() +
                                       "planning/simulation/models/one_shelf_back.stl");
  assets_["Drifter"] = build_asset(Environment::repo_path() +
                                   "planning/simulation/models/drifter_low_poly.stl");
  assets_["Bossa Nova"] = build_asset(Environment::repo_path() +
                                      "planning/simulation/models/bossa_nova_robot.stl");
  assets_["Scrubber"] =
      build_asset(Environment::repo_path() + "planning/simulation/models/scrubber.stl");
  assets_["Store"] =
      build_asset(Environment::repo_path() + "planning/simulation/models/store.stl");

  jcc::Success() << "Loaded assets" << std::endl;

  geometry::spatial::RayCasterConfig cfg;

  for (double x = -0.1; x <= 0.5; x += 0.1) {
    for (double y = -0.5; y <= 0.5; y += 0.1) {
      const geometry::Unit3 dir(y, 1.0, x);

      const geometry::Ray ray{jcc::Vec3::UnitZ() * 0.2, dir.vec()};
      cfg.rays.push_back({ray});
      const geometry::Ray ray2{jcc::Vec3::UnitZ() * 0.2, -dir.vec()};
      cfg.rays.push_back({ray2});
    }
  }

  ray_caster_.init(cfg);
  jcc::Warning() << "Building Bounding Volume Hierarchies" << std::endl;
  // ray_caster_.add_mesh(assets_["One Shelf+X"].mesh);
  // ray_caster_.add_mesh(assets_["One Shelf-X"].mesh);
  // ray_caster_.add_mesh(assets_["Store"].mesh);
  ray_caster_.add_volume(assets_["Store"].get_bvh());

  jcc::Success() << "Built Bounding Volume Hierarchies" << std::endl;
}

void SimViewer::on_key(int key, int scancode, int action, int mods) {
  Window3D::on_key(key, scancode, action, mods);

  if (action == GLFW_PRESS) {
    if ((key == GLFW_KEY_Z) && (mods == GLFW_MOD_CONTROL)) {
      editor_commands_.push({EditorCommand::Undo});
    } else if ((key == GLFW_KEY_Y) && (mods == GLFW_MOD_CONTROL)) {
      editor_commands_.push({EditorCommand::Redo});
    } else if ((key == GLFW_KEY_SPACE)) {
      integrator_.toggle_pause();
    }
  }
}
void SimViewer::on_mouse_button(int button, int action, int mods) {
  Window3D::on_mouse_button(button, action, mods);

  // const auto &proj = projection();
  // const auto ray = proj.unproject(mouse_pos());

  const bool right_clicking = right_mouse_held();
  if (action == GLFW_PRESS) {
    if (editor_state_.hovered != -1) {
      auto &element = editor_state_.elements[editor_state_.hovered];
      const auto prev_state = element.selection_state;

      SelectionState new_state = SelectionState::None;
      if (right_clicking) {
        if (prev_state == SelectionState::Selected) {
          new_state = SelectionState::None;
        } else {
          new_state = SelectionState::Selected;
        }
      } else {
        new_state = SelectionState::Hovered;
      }
      element.selection_state = new_state;

      if (new_state != prev_state) {
        view_state_change_ = true;
      }
    }
  }
}
void SimViewer::on_mouse_move(const viewer::WindowPoint &mouse_pos) {
  Window3D::on_mouse_move(mouse_pos);
}
void SimViewer::on_scroll(const double amount) {
  Window3D::on_scroll(amount);
}

bool SimViewer::update_editor_state() {
  bool view_state_change = false;
  const auto &proj = projection();
  const auto ray = proj.unproject(mouse_pos());

  //
  // Decide what is hovered
  //

  const auto meets = interactable_geo_.all_intersections(ray);
  const int prev_hovered = editor_state_.hovered;
  if (!meets.empty()) {
    const auto &inter = meets[0];
    editor_state_.hovered = inter.intersected_id;
  } else {
    editor_state_.hovered = -1;
  }
  if (editor_state_.hovered != prev_hovered) {
    view_state_change = false;
    // Consider changing view state if we can cache things more effectively
  }

  //
  // Plane intersection
  //

  const geometry::Plane plane{Vec3::Zero(), geometry::Unit3::UnitZ()};

  jcc::Vec3 intersection;
  const bool intersected = plane.intersect(ray, out(intersection));
  if (intersected) {
    editor_state_.world_hovered = true;
    editor_state_.world_click_pos = intersection;

    const jcc::Vec3 draw_loc =
        editor_state_.config.snap_to_grid ? snap_to_grid(intersection) : intersection;
    const jcc::Vec3 size(1.0, 1.0, 1.0);
    tmp_geo_.lines.push_back({draw_loc, draw_loc + jcc::Vec3::UnitX()});
    tmp_geo_.lines.push_back({draw_loc + jcc::Vec3::UnitX(),
                              draw_loc + jcc::Vec3::UnitY() + jcc::Vec3::UnitX()});
    tmp_geo_.lines.push_back({draw_loc + jcc::Vec3::UnitY() + jcc::Vec3::UnitX(),
                              draw_loc + jcc::Vec3::UnitY()});
    tmp_geo_.lines.push_back({draw_loc, draw_loc + jcc::Vec3::UnitY()});

  } else {
    editor_state_.world_hovered = false;
  }

  //
  // Main Menu UI Availability
  //
  editor_state_.can_undo = cmd_queue_.can_undo();
  editor_state_.can_redo = cmd_queue_.can_redo();

  return view_state_change;
}

void SimViewer::draw_robot(const planning::drifter::State &state) {
  if (!editor_state_.any_robot_placed) {
    return;
  }

  const auto &robot = editor_state_.robot;
  const auto &asset = assets_.at(robot.model);

  const SE3 world_from_robot(state.R_world_from_body, state.x_world);

  geometry::spatial::BoundingBox<3> bbox;
  for (const auto &pt : asset.bbox.points()) {
    bbox.expand(world_from_robot * pt);
  }

  const auto meets = interactable_geo_.all_intersections(bbox);
  if (!meets.empty()) {
    geometry::put_bounding_box(tmp_geo_, bbox, jcc::Vec4(1.0, 1.0, 1.0, 1.0));
  }

  robot_geo_.tri_meshes[0].world_from_mesh = world_from_robot;

  const SE3 robot_from_camera = jcc::exp_z(M_PI * 0.5);

  const SE3 world_from_camera = world_from_robot * robot_from_camera;

  // robot_geo_.polygons.clear();
  // viewer::Polygon pgon;
  // pgon.points.push_back(world_from_camera * jcc::Vec3(0.0, 0.0, 0.1));
  // pgon.points.push_back(world_from_camera * jcc::Vec3(9.0, +1.75, 0.1));
  // pgon.points.push_back(world_from_camera * jcc::Vec3(9.0, -1.75, 0.1));
  // pgon.color = jcc::Vec4(0.2, 1.0, 0.2, 0.3);
  // robot_geo_.polygons.push_back(pgon);
}

void SimViewer::draw_element(int id, const Element &element) {
  const jcc::Vec3 size = element.properties.at("size");

  const jcc::Vec3 lower = element.world_from_element *
                          (jcc::Vec3::Zero() + jcc::Vec3(size.x(), size.y(), size.z()));
  const jcc::Vec3 upper = element.world_from_element.translation();

  // const auto bbox = element.model != ""? assets_.at(element.model).bbox :
  geometry::spatial::BoundingBox<3> bbox;

  if (element.model != "") {
    for (const auto &pt : assets_.at(element.model).bbox.points()) {
      bbox.expand(element.world_from_element * pt);
    }
  } else {
    bbox.expand(lower);
    bbox.expand(upper);
  }

  interactable_geo_.add_intersectible(id, bbox);

  if (element.element_type != ElementType::Object) {
    const jcc::Vec4 color = get_color(element);
    geometry::put_bounding_box(geo_, bbox, color);
  }
  if (element.model != "") {
    const auto asset = assets_.at(element.model);
    // const bool outline = element.selection_state == SelectionState::Selected;
    const bool outline = false;
    const bool filled = true;

    const jcc::Vec4 color = element.selection_state == SelectionState::Selected
                                ? jcc::Vec4(0.7, 0.9, 0.7, 1.0)
                                : jcc::Vec4(0.6, 0.6, 0.6, 1.0);

    const double outline_width = 3.0;

    geo_.tri_meshes.push_back(
        {asset.mesh, element.world_from_element, color, filled, outline_width, outline});
  }
}

void SimViewer::draw() {
  const bool prev_had_robots = editor_state_.any_robot_placed;

  tmp_geo_.clear();
  bool view_state_change = false;
  view_state_change |= update_editor_state();

  editor_state_.filename.copy(main_menu_state_.filename_entry_buffer,
                              editor_state_.filename.size() + 1);
  main_menu_state_.sim_speed = editor_state_.config.sim_speed;
  create_main_menu(editor_state_, out(main_menu_state_));

  editor_state_.filename = std::string(main_menu_state_.filename_entry_buffer);
  editor_state_.config.sim_speed = main_menu_state_.sim_speed;
  editor_state_.config.snap_to_grid = main_menu_state_.snap_to_grid;
  editor_commands_.push(main_menu_state_.cmd_queue_update);

  // Mega-hack
  if (main_menu_state_.cmd_queue_update == EditorCommand::Open) {
    editor_state_.filename = main_menu_state_.new_file_name;
  }

  editor_state_.time_scrub = integrator_.scrub();
  if (ImGui::Begin("Time Control")) {
    if (ImGui::Button("Pause / Play")) {
    }

    if (ImGui::SliderInt(
            "Time (steps)", &editor_state_.time_scrub, 0, integrator_.max_scrub())) {
      integrator_.scrub_to(editor_state_.time_scrub);
    }
  }
  ImGui::End();

  //
  // Flush the queue of editor commands
  //

  while (!editor_commands_.empty()) {
    const auto cmd = editor_commands_.top();
    if (cmd == EditorCommand::Undo) {
      cmd_queue_.undo(out(editor_state_));
      view_state_change |= true;
    } else if (cmd == EditorCommand::Redo) {
      cmd_queue_.redo(out(editor_state_));
      view_state_change |= true;
    } else if (cmd == EditorCommand::Save) {
      JASSERT_NE(editor_state_.filename, "", "Cannot save without a set filename");
      const std::string path = jcc::Environment::repo_path() + "sims/";

      jcc::Warning() << "Saving as " << path + editor_state_.filename << std::endl;
      cmd_queue_.save(path + editor_state_.filename);
    } else if (cmd == EditorCommand::Open) {
      JASSERT_NE(editor_state_.filename, "", "Cannot save without a set filename");
      const std::string path = jcc::Environment::repo_path() + "sims/";

      jcc::Warning() << "Opening " << path + editor_state_.filename << std::endl;
      cmd_queue_.load(path + editor_state_.filename, out(editor_state_));
      view_state_change |= true;
    }

    editor_commands_.pop();
  }

  //
  // Find and execute new commands
  //

  const auto maybe_command = create_task_popup(editor_state_, out(task_popup_state_));
  if (maybe_command) {
    cmd_queue_.commit(*maybe_command, out(editor_state_));
    view_state_change |= true;
  }

  if (prev_had_robots == false && editor_state_.any_robot_placed) {
    integrator_.reset(editor_state_.robot);

    const bool outline = false;
    const bool filled = true;
    const jcc::Vec4 color(0.8, 0.8, 0.8, 1.0);
    const double outline_width = 1.0;

    std::cout << "Loading robot: " << editor_state_.robot.model << std::endl;
    const auto &asset = assets_.at(editor_state_.robot.model);
    robot_geo_.tri_meshes.push_back({asset.mesh, editor_state_.robot.world_from_robot,
                                     color, filled, outline_width, outline});
  } else {
    if (!editor_state_.any_robot_placed) {
      // view_state_change = true;
      robot_geo_.clear();

      plan_geo_.clear();
    }
  }

  const auto t_now = jcc::now();

  if (editor_state_.any_robot_placed) {
    show_menu(out(integrator_.config()));
  }

  if (editor_state_.any_robot_placed &&
      (t_now - jcc::to_duration(editor_state_.config.sim_speed)) > last_step_time_) {
    integrator_.clear_avoiders();

    bool any_go_zones = false;
    for (const auto &el : editor_state_.elements) {
      const auto &element = el.second;
      const jcc::Vec3 element_center = element.world_from_element.translation() +
                                       (element.properties.at("size") * 0.5);
      if (element.element_type == ElementType::NoGoZone) {
        const double element_radius = element.properties.at("size").norm();
        integrator_.add_avoider(element_center, element_radius);
      }
      if ((ElementType::GoZone == element.element_type) && !any_go_zones) {
        any_go_zones = true;
        integrator_.set_target(element_center);
      }
    }

    if (!any_go_zones) {
      integrator_.set_target(jcc::Vec3(0.0, 0.0, 0.0));
    }

    if (editor_state_.robot.model == "Bossa Nova") {
      integrator_.set_max_speed(0.05);
      // integrator_.set_target(jcc::Vec3(0.0, 4.5, 0.0));
    } else if (editor_state_.robot.model == "Scrubber") {
      integrator_.set_max_speed(2.0);
    } else {
      integrator_.set_max_speed(15.5);
    }

    last_step_time_ = t_now;
    plan_geo_.clear();
    const auto plan = integrator_.step();

    if (integrator_.config().debug.show_trajectory) {
      for (std::size_t k = 1u; k < plan.size(); ++k) {
        const auto xp = plan[k - 1];
        const auto xn = plan[k];
        plan_geo_.lines.push_back({xn.state.x_world, xp.state.x_world});
      }
    }

    const auto &desires = integrator_.desires();
    const auto compute_cost =
        planning::drifter::make_drifter_cost(integrator_.config(), desires);

    const int test_state = integrator_.config().debug.visualize_last_state
                               ? planning::drifter::HORIZON - 1
                               : 0;

    const auto x0 = plan[0].state;
    const auto utest = plan[0].control;
    auto xtest = x0;
    const double spacing = 0.25;

    if (integrator_.config().debug.show_cost) {
      viewer::ColoredPoints colored_points;
      const jcc::Vec3 center = (x0.x_world.array() / spacing).floor() * spacing;
      for (double x = -5.5; x < 5.5; x += spacing) {
        for (double y = -5.5; y < 5.5; y += spacing) {
          const jcc::Vec3 offset(x, y, 0.0);
          xtest.x_world = center + offset;
          const double cost = compute_cost(
              xtest, planning::drifter::Controls::to_vector(utest), test_state);

          const jcc::Vec3 viz_pt = xtest.x_world + jcc::Vec3::UnitZ() * 0.1 * cost;
          const Vec4 color = jcc::augment(viewer::colors::viridis(cost / 8.0), 0.9);
          colored_points.points.push_back(viz_pt);
          colored_points.colors.push_back(color);
          colored_points.sizes.push_back(3.0);
        }
      }

      if (integrator_.config().debug.enable_look_target) {
        const jcc::Vec3 look_target = integrator_.desires().look_target;
        plan_geo_.lines.push_back({look_target, look_target + jcc::Vec3::UnitZ(),
                                   jcc::Vec4(0.5, 0.9, 0.4, 1.0)});
        plan_geo_.raw_points.push_back({look_target, jcc::Vec4(1.0, 1.0, 0.0, 0.6)});
        plan_geo_.raw_points.push_back(
            {look_target + jcc::Vec3::UnitZ(), jcc::Vec4(0.2, 1.0, 0.0, 0.6)});
      }

      plan_geo_.colored_points.push_back(colored_points);
    }
  }

  //
  // Draw things
  //

  // TODO: Add "tentative" objects that are not in queue and drawn w/ diff colors
  if (view_state_change || view_state_change_) {
    geo_.clear();
    interactable_geo_.clear();

    for (const auto &element : editor_state_.elements) {
      draw_element(element.first, element.second);
    }

    view_state_change_ = false;
  }

  if (editor_state_.any_robot_placed) {
    draw_robot(integrator_.get());
  }

  viewer::draw_geometry_buffer(geo_);
  viewer::draw_geometry_buffer(tmp_geo_);
  viewer::draw_geometry_buffer(robot_geo_);
  viewer::draw_geometry_buffer(plan_geo_);
  viewer::draw_geometry_buffer(lidar_geo_);
  viewer::draw_geometry_buffer(bgnd_geo_);
}

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title) {
  const viewer::GlSize gl_size(640, 640);
  auto window = std::make_shared<SimViewer>();
  viewer::WindowManager::register_window(gl_size, window, title);
  return window;
}

}  // namespace simulation
}  // namespace jcc
