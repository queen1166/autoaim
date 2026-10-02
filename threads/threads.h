
#pragma once

#include <thread>

#include "app/autoaim_app.h"

namespace autoaim::threads {

struct Handles {
  std::thread camera;
  std::thread process;
  std::thread tx;
  std::thread rx;

  Handles() = default;

  Handles(const Handles &) = delete;
  Handles & operator=(const Handles &) = delete;

  Handles(Handles &&) = default;
  Handles & operator=(Handles &&) = delete;

  ~Handles();
};

Handles start(app::AutoAimApp & app);

void joinAll(Handles & h);

}
