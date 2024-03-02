#include <cstdlib>
#include <print>
#include <stdexcept>

#include "app.h"

int main() {
  HelloTriangleApp app;

  try {
    app.run();
  } catch (const std::exception &e) {
    std::println(stderr, "{}", e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
