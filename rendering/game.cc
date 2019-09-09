#include "rendering/game_viewer.hh"

int main() {
  auto gv = jcc::create_gameviewer();
  gv->go();
}