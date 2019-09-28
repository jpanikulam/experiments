#include "rendering/buffers/texture_manager.hh"

#include "third_party/imgui/imgui.h"

namespace jcc {

TextureManager::TextureManager() {
}

void TextureManager::show_ui() const {
  ImGui::Begin("Textures");
  ImGuiIO& io = ImGui::GetIO();

  const ImVec2 pos = ImGui::GetCursorScreenPos();

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
        // TODO: Implement a picker

        ImGui::BeginTooltip();
        float region_sz = 32.0f;
        float region_x = io.MousePos.x - pos.x - (region_sz * 0.5f);
        if (region_x < 0.0f) {
          region_x = 0.0f;
        } else if (region_x > draw_w - region_sz) {
          region_x = draw_w - region_sz;
        }

        float region_y = io.MousePos.y - pos.y - (region_sz * 0.5f);
        if (region_y < 0.0f) {
          region_y = 0.0f;
        } else if (region_y > draw_h - region_sz) {
          region_y -= draw_h + region_sz;
        }

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
    }
  }
  ImGui::End();
}

}  // namespace jcc