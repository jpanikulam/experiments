#include "planning/simulation/sim_viewer.hh"

namespace jcc {

void SimViewer::draw() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("New")) {
      }
      if (ImGui::MenuItem("Open", "Ctrl+O")) {
      }
      if (ImGui::BeginMenu("Open Recent")) {
        ImGui::MenuItem("simple_jet.sim");
        ImGui::MenuItem("easy_jet.sim");
        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Save", "Ctrl+S")) {
      }
      if (ImGui::MenuItem("Save As..")) {
      }

      ImGui::Separator();
      if (ImGui::BeginMenu("Options")) {
        ImGui::EndMenu();
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Undo", "CTRL+Z")) {
        std::cout << "Undo" << std::endl;
      }
      if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {
      }  // Disabled item
      ImGui::Separator();
      if (ImGui::MenuItem("Cut", "CTRL+X")) {
      }
      if (ImGui::MenuItem("Copy", "CTRL+C")) {
      }
      if (ImGui::MenuItem("Paste", "CTRL+V")) {
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Controls")) {
      float f = 0.0;
      ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  } else {
    std::cout << "Don't begin" << std::endl;
  }

  int window_flags = ImGuiWindowFlags_NoMove;
  if (ImGui::Begin("Hello, world!", nullptr, window_flags)) {
    ImGui::SetWindowPos(ImVec2(0, gl_size_.height - ImGui::GetWindowHeight() - 40), true);
  }
  ImGui::End();
}

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title) {
  const GlSize gl_size(640, 640);
  auto window = std::make_shared<SimViewer>(gl_size);
  get_global_state().windows[title] = window;
  WindowManager::register_window(gl_size, window, title);
  return window;
}

}  // namespace jcc