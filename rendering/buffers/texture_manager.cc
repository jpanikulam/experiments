#include "rendering/buffers/texture_manager.hh"

#include "third_party/imgui/imgui.h"
#include "util/clamp.hh"
#include "util/eigen_clip.hh"

// TODO
#include "eigen.hh"

namespace jcc {

namespace {
void show_tooltip(void* tex_id, double draw_w, double draw_h, float region_sz) {
  const ImGuiIO& io = ImGui::GetIO();
  const ImVec2 cursor_pos = ImGui::GetCursorScreenPos();

  // Position of mouse in screen coordinates
  const jcc::Vec2 mouse_from_screen(io.MousePos.x, io.MousePos.y);
  const jcc::Vec2 image_bl_from_screen(cursor_pos.x, cursor_pos.y);

  const jcc::Vec2 image_origin_from_screen =
      image_bl_from_screen - jcc::Vec2(0.0, draw_w);
  const jcc::Vec2 mouse_from_image = mouse_from_screen - image_origin_from_screen;

  const jcc::Vec2 region_size(32, 32);
  const jcc::Vec2 image_size(draw_w, draw_h);

  const jcc::Vec2 region_min_unclipped = mouse_from_image - (region_size * 0.5);
  const jcc::Vec2 region_max_unclipped = mouse_from_image + (region_size * 0.5);

  const jcc::Vec2 region_min =
      eigen_clip(region_min_unclipped, jcc::Vec2::Zero(), image_size);
  const jcc::Vec2 region_max =
      eigen_clip(region_max_unclipped, jcc::Vec2::Zero(), image_size);

  const jcc::Vec2 uv0 = region_min.array() / image_size.array;
  const jcc::Vec2 uv1 = region_max.array() / image_size.array;

  // TODO: Implement a picker
  ImGui::BeginTooltip();

  ImGui::Text("MfromI %.2f, %.2f", mouse_from_image.x(), mouse_from_image.y());

  /*  float region_x = io.MousePos.x - cursor_pos.x - (region_sz * 0.5f);
    if (region_x < 0.0f) {
      region_x = 0.0f;
    } else if (region_x > draw_w - region_sz) {
      region_x = draw_w - region_sz;
    }

    float region_y = io.MousePos.y - cursor_pos.y - (region_sz * 0.5f);

    int branch_num = 0;
    if (region_y < 0.0f) {
      region_y = 0.0f;
      branch_num = 1;
    } else if (region_y > draw_h - region_sz) {
      // region_y -= draw_h + region_sz;
      region_y += draw_h - region_sz;
      branch_num = 2;
    }
  */
  ImGui::Text("Branch: %d", branch_num);
  ImGui::Text("Mouse: (%.2f, %.2f)", io.MousePos.x, io.MousePos.y);
  ImGui::Text("Pos: (%.2f, %.2f)", cursor_pos.x, cursor_pos.y);

  float zoom = 4.0f;
  ImGui::Text("Min: (%.2f, %.2f)", region_x, region_y);
  ImGui::SameLine();
  ImGui::Text("Max: (%.2f, %.2f)", region_x + region_sz, region_y + region_sz);
  const ImVec2 uv0 = ImVec2((region_x) / draw_w, (region_y) / draw_h);
  const ImVec2 uv1 =
      ImVec2((region_x + region_sz) / draw_w, (region_y + region_sz) / draw_h);
  ImGui::Image(tex_id,
               ImVec2(region_sz * zoom, region_sz * zoom),
               uv0,
               uv1,
               ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
               ImVec4(1.0f, 1.0f, 1.0f, 0.5f));
  ImGui::EndTooltip();
}
}  // namespace

TextureManager::TextureManager() {
}

void TextureManager::show_ui() const {
  ImGui::Begin("Textures");

  for (const auto& tex_pair : textures_) {
    const auto& name = tex_pair.first;
    const auto& tex = tex_pair.second;

    if (!ImGui::CollapsingHeader(name.c_str())) {
      continue;
    }

    ImGui::Text("%s", name.c_str());

    const int width = tex.size().width;
    const int height = tex.size().height;
    if (width != 0 && height != 0) {
      const auto tex_id = (void*)(intptr_t)tex.get_id();

      const double draw_w = 0.2 * width;
      const double draw_h = 0.2 * height;
      ImGui::Image(tex_id, ImVec2(draw_w, draw_h));

      if (ImGui::IsItemHovered()) {
        show_tooltip(tex_id, draw_w, draw_h, 32.0f);
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