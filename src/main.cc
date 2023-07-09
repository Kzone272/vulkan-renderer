#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "hello_triangle_app.h"

int main() {
  HelloTriangleApp app;

  try {
    app.run();
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
