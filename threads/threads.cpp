#include "threads/threads.h"

namespace autoaim::threads {

Handles::~Handles() { joinAll(*this); }

Handles start(app::AutoAimApp & app) {
  Handles h;
  h.rx = std::thread([&app] { app.rxLoop(); });
  h.tx = std::thread([&app] { app.txLoop(); });
  h.camera = std::thread([&app] { app.cameraLoop(); });
  h.process = std::thread([&app] { app.processLoop(); });
  return h;
}

void joinAll(Handles & h) {
  if (h.camera.joinable()) h.camera.join();
  if (h.process.joinable()) h.process.join();
  if (h.rx.joinable()) h.rx.join();
  if (h.tx.joinable()) h.tx.join();
}

}
