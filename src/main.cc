#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "asserts.h"
#include "hello_triangle_app.h"

using std::cerr;
using std::endl;

int main() {
  HelloTriangleApp app;

  try {
    app.run();
  } catch (const std::exception &e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
