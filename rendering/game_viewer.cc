#include "rendering/game_viewer.hh"

#include "viewer/rendering/pre_render.hh"
#include "viewer/window_manager.hh"

#include "geometry/types/angle.hh"

#include <thread>

#include "third_party/imgui/imgui.h"

// TODO
#include <iostream>
namespace jcc {

// Compute the perspective projection matrix
//
// Principle:
//
// f = cot(fovy / 2)
// (f / aspect), 0, 0, 0
// 0, f, 0, 0
// 0, 0, (zFar + zNear) / (zNear - zFar), (2 × zFar × zNear) / (zNear - zFar)
// 0, 0, -1, 0
//
// [1] https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluPerspective.xml
MatNf<4, 4> create_perspective_from_camera(const geometry::Angle &fovy,
                                           double aspect,
                                           double z_near,
                                           double z_far) {
  const double f = 1.0 / std::tan(fovy.rads() / 2.0);

  MatNf<4, 4> perspective_from_camera;
  perspective_from_camera.row(0) << (f / aspect), 0, 0, 0;
  perspective_from_camera.row(1) << 0, f, 0, 0;
  perspective_from_camera.row(2) << 0, 0, (z_far + z_near) / (z_near - z_far),
      (2 * z_far * z_near) / (z_near - z_far);
  perspective_from_camera.row(3) << 0, 0, -1, 0.0;

  return perspective_from_camera;
}

MatNf<4, 4> create_perspective_from_camera(const viewer::GlSize &gl_size) {
  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 1000.0;
  const geometry::Angle fov = geometry::Angle::from_degrees(60.0);
  const double aspect_ratio =
      (static_cast<double>(gl_size.width) / static_cast<double>(gl_size.height));
  return create_perspective_from_camera(fov, aspect_ratio, NEAR_CLIP, FAR_CLIP);
}

GameViewer::GameViewer() {
  gv_state_.view.camera.set_anchor_from_world(
      SE3(SO3::exp(Eigen::Vector3d(-3.1415 * 0.5, 0.0, 0.0)), Eigen::Vector3d::Zero()));
}

void GameViewer::init(const viewer::GlSize &gl_size) {
  imgui_mgr_.init(get_window(), {true});

  resize(gl_size);

  test_shader_ =
      load_shaders("/home/jacob/repos/experiments/rendering/shaders/phong.vert",
                   "/home/jacob/repos/experiments/rendering/shaders/phong.frag");
  test_asset_ =
      load_voxel_asset("/home/jacob/repos/experiments/rendering/assets/spaceship1.vox");
}

void GameViewer::on_key(int key, int scancode, int action, int mods) {
  if (imgui_mgr_.want_capture()) {
    return;
  }
}
void GameViewer::on_mouse_button(int button, int action, int mods) {
  if (imgui_mgr_.want_capture()) {
    return;
  }

  const auto current_mouse_pos = mouse_pos();
  gv_state_.interaction.mouse_pos_last_click = current_mouse_pos;
}
void GameViewer::on_mouse_move(const viewer::WindowPoint &mouse_pos) {
  if (imgui_mgr_.want_capture()) {
    return;
  }

  const bool shift = held_keys().count(GLFW_KEY_LEFT_SHIFT) == 1
                         ? held_keys().at(GLFW_KEY_LEFT_SHIFT)
                         : false;
  const bool left = left_mouse_held() && !shift;
  const bool right = right_mouse_held() || (shift && left_mouse_held());

  gv_state_.view.camera.apply_mouse(mouse_pos, gv_state_.interaction.mouse_pos_last_click,
                                    left, right);
  if (left || right) {
    gv_state_.interaction.mouse_pos_last_click = mouse_pos;
  }
}

void GameViewer::on_scroll(const double amount) {
  if (imgui_mgr_.want_capture()) {
    return;
  }
  gv_state_.view.camera.apply_scroll(amount);
}

void GameViewer::resize(const viewer::GlSize &gl_size) {
  glViewport(0, 0, gl_size.width, gl_size.height);

  SimpleWindow::resize(gl_size);
}

void GameViewer::draw_scene() {
  const geometry::shapes::Plane ground{jcc::Vec3::UnitZ(), 0.0};

  {
    test_shader_.use();
    const MatNf<4, 4> perspective_from_camera = create_perspective_from_camera(gl_size());
    test_shader_.set("perspective_from_camera", perspective_from_camera);

    const MatNf<4, 4> camera_from_world =
        gv_state_.view.camera.camera_from_world().matrix().cast<float>();
    test_shader_.set("camera_from_world", camera_from_world);

    const auto vertices = test_asset_.vertices();
    const auto normals = test_asset_.normals();

    std::vector<jcc::Vec3f> colors;
    colors.reserve(vertices.size());
    for (std::size_t k = 0; k < vertices.size(); ++k) {
      colors.push_back(jcc::Vec3f(0.7f, 0.6f, 0.7f));
    }

    auto ship_vao = test_shader_.generate_vao();
    ship_vao.bind();

    const auto indices = test_asset_.faces();

    GLuint element_buffer;
    glGenBuffers(1, &element_buffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * (3 * 4), indices.data(),
                 GL_STATIC_DRAW);

    ship_vao.set("vertex_color", colors);
    ship_vao.set("vertex_world", vertices);
    ship_vao.set("vertex_normal", normals);

    ship_vao.bind();
    glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, (void *)0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glLineWidth(3.0);
    glDrawArrays(GL_LINES, 0, vertices.size());

    // glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    ship_vao.destroy();

    auto light_vao = test_shader_.generate_vao();
    light_vao.bind();
    const std::vector<jcc::Vec3f> light_colors = {jcc::Vec3f(1.0, 1.0, 1.0),  //
                                                  jcc::Vec3f(1.0, 1.0, 1.0),  //
                                                  jcc::Vec3f(1.0, 1.0, 1.0)};
    const std::vector<jcc::Vec3f> light_vertices = {jcc::Vec3f(10.0, 1.0, 1.0),  //
                                                    jcc::Vec3f(9.0, 1.0, 0.0),   //
                                                    jcc::Vec3f(9.0, 0.0, 1.0)};
    const std::vector<jcc::Vec3f> light_normals = {
        jcc::Vec3f(1.0, 1.0, 1.0),  //
        jcc::Vec3f(1.0, 1.0, 1.0),  //
        jcc::Vec3f(1.0, 1.0, 1.0)   //
    };

    light_vao.set("vertex_color", light_colors);
    light_vao.set("vertex_world", light_vertices);
    light_vao.set("vertex_normal", light_normals);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    light_vao.destroy();
  }
}

void GameViewer::render() {
  viewer::prepare_to_render();

  const auto win_size = gl_size();
  const jcc::Vec4i viewport_dims(0, 0, win_size.width, win_size.height);
  gv_state_.view.projection = viewer::Projection(
      create_perspective_from_camera(win_size).cast<double>(),
      gv_state_.view.camera.camera_from_world().matrix(), viewport_dims);
  // TODO: Generate perspective projection -- we cannot query the GPU for this!

  imgui_mgr_.new_frame();
  if (ImGui::Begin("Sand")) {
    if (ImGui::Button("Sand Button")) {
      std::cout << "Sand" << std::endl;
    }
  }
  ImGui::End();

  imgui_mgr_.render();

  draw_scene();

  const double t_now = glfwGetTime();

  const double dt = std::max(t_now - gv_state_.sim.last_update_time, 0.02);
  gv_state_.view.camera.apply_keys(held_keys(), dt);
  gv_state_.view.camera = gv_state_.view.camera.simulate(dt);
  gv_state_.sim.last_update_time = t_now;

  glFinish();
}

void GameViewer::go() const {
  while (!should_close()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(23));
  }
}

std::shared_ptr<GameViewer> create_gameviewer(const GameViewerConfig &cfg) {
  const viewer::GlSize gl_size(640, 640);
  auto window = std::make_shared<GameViewer>();

  const int win_ver_maj = 4;
  const int win_ver_min = 6;
  viewer::WindowManager::register_window(gl_size, window, cfg.title, win_ver_maj,
                                         win_ver_min);
  return window;
}

}  // namespace jcc