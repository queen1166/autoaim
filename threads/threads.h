// 线程编排

#pragma once

#include <thread>

#include "app/autoaim_app.h"

namespace autoaim::threads {

// 四个线程的句柄包。main.cpp 里作为一个局部变量持有它，
// 作用域结束时自动 join 所有线程 —— 这就是"线程生命周期管理"的全部。
//
// 注意它是 move-only 的：线程没法"复制"（复制一条正在跑的线程意味着什么？），
// 所以这个结构体也只能移动。
struct Handles {
  std::thread camera;
  std::thread process;
  std::thread tx;
  std::thread rx;

  // 默认构造：四个句柄都是"空的"（!joinable()）。
  // 由 start() 依次填上。
  Handles() = default;

  // 禁止拷贝：std::thread 本身就是 move-only。
  Handles(const Handles &) = delete;
  Handles & operator=(const Handles &) = delete;

  // 用户声明了析构函数 → 隐式移动被抑制，必须显式写出来，
  // 否则 start() 里的 `return h` 编不过（NRVO 不保证发生）。
  Handles(Handles &&) = default;
  Handles & operator=(Handles &&) = delete;

  // 析构时自动 join。
  // 这不是可有可无的 —— std::thread 析构时如果还 joinable，会直接
  // std::terminate。任何提前 return 或异常抛出都会踩到。
  ~Handles();
};

// 启动四个线程。各循环内部自己检查 app 的 running 标志。
//
// app 以【引用】被四条线程捕获（lambda 里的 [&app]），所以调用方
// 必须保证 app 活得比这四个线程久 —— 这就是 main.cpp 里
// `handles` 声明在 `application` 之后的原因（局部变量逆序析构）。
//
// 启动顺序有讲究：rx → tx → camera → process。这样"发送"一定早于
// "处理"开始 —— 协议要求持续发送，下位机一直用最后一次收到的有效指令，
// 越早开始发越安全。
//
// 返回 Handles（移动构造）。失败会抛异常（std::thread 构造失败）。
Handles start(app::AutoAimApp & app);

// 等待四个线程结束。会阻塞调用线程。
//
// 调用前应该先 app.stop() 把 running_ 置 false，否则工作线程
// 不认为该收工，会一直等到自然结束 —— 那就卡住了。
// 每个 join 前都检查 joinable()，所以重复调用是安全的。
//
// 顺序：camera → process → rx → tx。先停生产数据的，
// 最后停发送的 —— 保证云台在整个关机过程中一直收到指令。
void joinAll(Handles & h);

}  // namespace autoaim::threads
