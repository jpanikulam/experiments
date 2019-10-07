#include "rendering/game_viewer.hh"

#include "rendering/buffers/element_buffer.hh"
#include "rendering/buffers/framebuffer.hh"
#include "rendering/buffers/texture.hh"
#include "rendering/buffers/texture_manager.hh"
#include "rendering/imgui_elements/game_ui_elements.hh"

#include "geometry/types/angle.hh"
#include "logging/assert.hh"
#include "viewer/rendering/pre_render.hh"
#include "viewer/window_manager.hh"

#include "third_party/imgui/imgui.h"

#include <thread>

// TODO
#include "third_party/backward/backward.hpp"
// %deps(bfd, dl)
backward::SignalHandling sh;
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

MatNf<4, 4> create_ortho_from_camera(const viewer::GlSize &gl_size) {
  constexpr double NEAR_CLIP = 0.001;
  constexpr double FAR_CLIP = 10.0;

  const double f = FAR_CLIP;
  const double n = NEAR_CLIP;

  const double b = 0.0;
  const double l = 0.0;
  const double r = gl_size.width * 0.001;
  const double t = gl_size.height * 0.001;

  MatNf<4, 4> ortho;
  ortho.row(0) << 2 / (r - l), 0.0, 0.0, 0.0;
  ortho.row(1) << 0.0, 2 / (t - b), 0.0, 0.0;
  ortho.row(2) << 0.0, 0.0, -2.0 / (f - n), 0.0;
  ortho.row(3) << -(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1.0;
  return ortho;
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

  std::cout << "===\n" << std::endl;

  shadow_shader_ =
      load_shaders("/home/jacob/repos/experiments/rendering/shaders/shadow.vert",
                   "/home/jacob/repos/experiments/rendering/shaders/shadow.frag");

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
  const bool left = left_mouse_held();
  const bool right = right_mouse_held();

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

void GameViewer::draw_elements() {
  // test_shader_.use();

  const MatNf<4, 4> perspective_from_camera = create_perspective_from_camera(gl_size());
  test_shader_.set("u_perspective_from_camera", perspective_from_camera);

  const MatNf<4, 4> camera_from_world =
      gv_state_.view.camera.camera_from_world().matrix().cast<float>();
  test_shader_.set("u_camera_from_world", camera_from_world);

  // const jcc::Vec3f light_pos_world(1.0, 1.0, 1.0);
  // test_shader_.set("u_light_pos_world", light_pos_world);

  auto ship_vao = test_shader_.generate_vao();
  ship_vao.bind();
  const auto indices = test_asset_.faces();

  const ElementBuffer element_buffer(indices);

  ship_vao.set("vertex_color", test_asset_.colors());
  ship_vao.set("vertex_world", test_asset_.vertices());
  ship_vao.set("vertex_normal", test_asset_.normals());

  const auto mode = ui_cfg_.debug.wireframe ? GL_LINE : GL_FILL;
  glPolygonMode(GL_FRONT_AND_BACK, mode);
  element_buffer.draw_elements();

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
}

void GameViewer::draw_scene() {
  //
  // Create shadow buffer
  //

  // glEnable(GL_TEXTURE_2D);

  // TextureManager manager;

  // TODO
  Framebuffer fbo;

  auto &depth_texture = tex_mgr_.create_texture("depth");
  // Texture depth_texture;
  depth_texture.tex_image_2d(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, {4096, 4096}, 0,
                             GL_DEPTH_COMPONENT, GL_FLOAT);
  fbo.attach_depth_texture(depth_texture);

  auto &normals_texture = tex_mgr_.create_texture("normals");
  // Texture normals_texture;
  normals_texture.tex_image_2d(GL_TEXTURE_2D, 0, GL_RGBA16F, {4096, 4096}, 0, GL_RGBA,
                               GL_FLOAT);
  fbo.attach_color_texture(normals_texture);

  auto &pos_texture = tex_mgr_.create_texture("position");
  pos_texture.tex_image_2d(GL_TEXTURE_2D, 0, GL_RGBA16F, {4096, 4096}, 0, GL_RGBA,
                           GL_FLOAT);
  fbo.attach_color_texture(pos_texture);

  auto &color_texture = tex_mgr_.create_texture("color");
  color_texture.tex_image_2d(GL_TEXTURE_2D, 0, GL_RGBA8, {4096, 4096}, 0, GL_RGBA,
                             GL_FLOAT);
  fbo.attach_color_texture(color_texture);

  fbo.draw_buffers();

  fbo.bind();

  glViewport(0, 0, 4096, 4096);

  JASSERT_EQ(glCheckFramebufferStatus(GL_FRAMEBUFFER),
             static_cast<std::size_t>(GL_FRAMEBUFFER_COMPLETE), "Incomplete Framebuffer");

  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  // const MatNf<4, 4> perspective_from_light = create_ortho_from_camera({4096, 4096});
  const MatNf<4, 4> perspective_from_light = create_perspective_from_camera(gl_size());

  const jcc::Vec6 log_light_from_world =
      (jcc::Vec6() << ui_cfg_.debug.d * 5.9518, ui_cfg_.debug.d * -2.35115,
       ui_cfg_.debug.d * -4.51881, -0.419103, 1.38883, 2.21413)
          .finished();
  const SE3 light_from_world =
      SE3(SO3::exp(jcc::Vec3::UnitZ() * ui_cfg_.debug.theta), jcc::Vec3::Zero()) *
      SE3::exp(log_light_from_world);
  const MatNf<4, 4> light_from_world_mat = light_from_world.matrix().cast<float>();

  {
    shadow_shader_.use();

    shadow_shader_.set("u_light_from_world", light_from_world_mat);
    shadow_shader_.set("u_perspective_from_light", perspective_from_light);

    auto ship_vao = shadow_shader_.generate_vao();
    ship_vao.bind();
    const auto indices = test_asset_.faces();
    const ElementBuffer element_buffer(indices);

    ship_vao.set("vertex_color", test_asset_.colors());
    ship_vao.set("vertex_world", test_asset_.vertices());
    ship_vao.set("vertex_normal", test_asset_.normals());

    // Cull front faces as a weird way of hiding depth errors
    // glCullFace(GL_FRONT);

    // glBlendFunc(GL_ONE, GL_ONE);
    // glEnable(GL_BLEND);
    glDisable(GL_BLEND);

    // Check depth when rendering
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 1.0f);

    element_buffer.draw_elements();
    glDrawElements(GL_TRIANGLES, indices.size() * 3, GL_UNSIGNED_INT, (void *)0);
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // I do not fully grasp what SRGB is doing here, I expected it to have no effect on the
  // visual

  if (ui_cfg_.shading.srgb) {
    glEnable(GL_FRAMEBUFFER_SRGB);
  } else {
    glDisable(GL_FRAMEBUFFER_SRGB);
  }

  viewer::prepare_to_render();
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  resize(gl_size());

  test_shader_.use();

  const jcc::Vec3f light_pos_world =
      light_from_world.inverse().translation().cast<float>();

  test_shader_.set("u_light_pos_world", light_pos_world);
  test_shader_.set("u_shadow_normals_dist", normals_texture);
  test_shader_.set("u_shadow_pos", pos_texture);
  test_shader_.set("u_shadow_colors", color_texture);
  test_shader_.set("u_light_from_world", light_from_world_mat);
  test_shader_.set("u_perspective_from_light", perspective_from_light);

  test_shader_.set_bool("u_dbg_misc", ui_cfg_.shading.misc_debug);
  test_shader_.set_bool("u_dbg_shadows", ui_cfg_.shading.enable_shadows);
  test_shader_.set_bool("u_dbg_use_rsm", ui_cfg_.shading.use_rsm);
  test_shader_.set_bool("u_dbg_show_light_probes", ui_cfg_.shading.show_light_probes);
  draw_elements();
  GLenum err;
  err = glGetError();
  JASSERT_EQ(err, GL_NO_ERROR, "There was  an opengl error");

  imgui_mgr_.new_frame();
  tex_mgr_.show_ui();
  show_menu(out(ui_cfg_));
  imgui_mgr_.render();
  tex_mgr_.clear();
}

void GameViewer::render() {
  viewer::prepare_to_render();

  const auto win_size = gl_size();
  const jcc::Vec4i viewport_dims(0, 0, win_size.width, win_size.height);
  gv_state_.view.projection = viewer::Projection(
      create_perspective_from_camera(win_size).cast<double>(),
      gv_state_.view.camera.camera_from_world().matrix(), viewport_dims);
  // TODO: Generate perspective projection -- we cannot query the GPU for this!

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