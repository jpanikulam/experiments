#include "rendering/buffers/texture_manager.hh"

#include "eigen.hh"
#include "logging/assert.hh"
#include "util/clamp.hh"
#include "util/eigen_clip.hh"

#include "third_party/imgui/imgui.h"
namespace jcc {

namespace {
void show_tooltip(
    void* tex_id, double draw_w, double draw_h, float region_sz, double zoom) {
  const ImGuiIO& io = ImGui::GetIO();
  const ImVec2 cursor_pos = ImGui::GetCursorScreenPos();

  // Position of mouse in screen coordinates
  const jcc::Vec2 mouse_from_screen(io.MousePos.x, io.MousePos.y);
  const jcc::Vec2 image_bl_from_screen(cursor_pos.x, cursor_pos.y);

  const jcc::Vec2 image_origin_from_screen =
      image_bl_from_screen - jcc::Vec2(0.0, draw_w);
  const jcc::Vec2 mouse_from_image = mouse_from_screen - image_origin_from_screen;

  const jcc::Vec2 region_size(region_sz, region_sz);
  const jcc::Vec2 image_size(draw_w, draw_h);

  const jcc::Vec2 region_min_unclipped = mouse_from_image;

  const jcc::Vec2 region_max_unclipped = mouse_from_image + region_size;

  const jcc::Vec2 origin = jcc::Vec2::Zero();
  const jcc::Vec2 region_min = eigen_clip(region_min_unclipped, origin, image_size);
  const jcc::Vec2 region_max = eigen_clip(region_max_unclipped, origin, image_size);

  const jcc::Vec2 uv0 = region_min.array() / image_size.array();
  const jcc::Vec2 uv1 = region_max.array() / image_size.array();

  // TODO: Implement a picker
  ImGui::BeginTooltip();

  ImGui::Text("Min: (%.2f, %.2f)", region_min.x(), region_min.y());
  ImGui::SameLine();
  ImGui::Text("Max: (%.2f, %.2f)", region_max.x(), region_max.y());
  ImGui::Text("Zoom: %.2f", zoom);

  const ImVec2 im_uv0 = ImVec2(uv0.x(), uv0.y());
  const ImVec2 im_uv1 = ImVec2(uv1.x(), uv1.y());

  ImGui::Image(tex_id,
               ImVec2(region_sz * zoom, region_sz * zoom),
               im_uv0,
               im_uv1,
               ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
               ImVec4(1.0f, 1.0f, 1.0f, 0.5f));
  ImGui::EndTooltip();
}
}  // namespace

TextureManager::TextureManager() {
}

Texture& TextureManager::create_texture(const std::string& name) {
  JASSERT_LT(
      textures_.size(), 25u, "More than 25 textures allocated, probably a memory leak");
  return textures_[name];
}

void TextureManager::clear_textures() {
  textures_.clear();
}

void TextureManager::show_ui() {
  if (textures_.empty()) {
    return;
  }

  ImGui::Begin("Textures", nullptr, ImGuiWindowFlags_NoMove);
  ImGui::SetWindowPos(ImVec2(0.0, 0.0));
  ImGui::SetWindowSize(ImVec2(250, 500));

  for (const auto& tex_pair : textures_) {
    const auto& name = tex_pair.first;
    const auto& tex = tex_pair.second;

    if (!ImGui::CollapsingHeader(name.c_str())) {
      continue;
    }

    // texture_zoom_

    ImGui::Text("%s", name.c_str());

    const int width = tex.size().width;
    const int height = tex.size().height;
    if (width != 0 && height != 0) {
      const auto tex_id = (void*)(intptr_t)tex.get_id();

      const double draw_w = 250.0;
      const double draw_h = 250.0;

      ImGui::Image(tex_id, ImVec2(draw_w, draw_h));

      if (ImGui::IsItemHovered()) {
        if (0 == texture_zoom_.count(name)) {
          texture_zoom_[name] = 3.0;
        }

        show_tooltip(tex_id, draw_w, draw_h, 64.0f, texture_zoom_[name]);

        const ImGuiIO& io = ImGui::GetIO();
        const double zoom_change = io.MouseWheel;

        texture_zoom_[name] *= std::exp(zoom_change * 0.1);
        texture_zoom_[name] = jcc::clamp(texture_zoom_[name], 1.0, 10.0);
      }
    }
  }

  if (ImGui::CollapsingHeader("Picker")) {
    const ImGuiIO& io = ImGui::GetIO();

    constexpr int ORIGIN_X = 0;
    constexpr int ORIGIN_Y = 0;
    constexpr int FORMAT = GL_BGR;

    GLint m_viewport[4];
    glGetIntegerv(GL_VIEWPORT, m_viewport);

    const GLint width_px = m_viewport[2];
    const GLint height_px = m_viewport[3];
    ImGui::Text("Resolution: %dx%d", m_viewport[2], m_viewport[3]);

    ImGui::Text("Mouse Location: (%.2f, %.2f)", io.MousePos.x, io.MousePos.y);

    constexpr int TYPE = GL_UNSIGNED_BYTE;
    uint8_t* out_data = new uint8_t[width_px * height_px * 3];
    glReadPixels(ORIGIN_X, ORIGIN_Y, width_px, height_px, FORMAT, TYPE, out_data);

    const int col_dist = jcc::clamp(static_cast<int>(io.MousePos.x), 0, width_px);
    const int row_dist =
        jcc::clamp(height_px - static_cast<int>(io.MousePos.y), 0, height_px);

    constexpr int CELLS_PER_PX = 3;
    const int cells_per_row = CELLS_PER_PX * width_px;
    const int cells_per_col = CELLS_PER_PX;

    const int cell = (cells_per_col * col_dist) + (cells_per_row * row_dist);
    const uint8_t px_r = out_data[cell + 0];
    const uint8_t px_g = out_data[cell + 1];
    const uint8_t px_b = out_data[cell + 2];
    ImGui::Text("Pick Location Info: (%d, %d, %d)", col_dist, row_dist, cell);
    ImGui::Text("Color @ Pick: (%u, %u, %u)", px_r, px_g, px_b);

    delete out_data;
  }

  ImGui::End();
}

}  // namespace jcc