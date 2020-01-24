#include "rendering/game_viewer.hh"

// https://alvarber.gitlab.io/toon-shader-survey.html

int main() {
  auto gv = jcc::create_gameviewer();
  gv->go();
}