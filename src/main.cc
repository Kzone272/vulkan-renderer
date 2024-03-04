#include <cstdlib>
#include <print>
#include <stdexcept>

#include "app.h"

int main() {
  App app;

  try {
    app.run();
  } catch (const std::exception &e) {
    std::println(stderr, "{}", e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
