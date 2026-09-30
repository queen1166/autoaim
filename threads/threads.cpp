#include "threads/threads.h"

namespace autoaim::threads {

Handles::~Handles() { joinAll(*this); }

Handles start(app::AutoAimApp & app) {
  Handles h;
  // 顺序有讲究：先起接收线程，再起发送线程，最后才是相机和处理。
  // 这样"发送"一定早于"处理"开始 —— 协议要求持续发送，
  // 下位机一直用最后一次收到的有效指令，所以越早开始发越安全。
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

}  // namespace autoaim::threads
