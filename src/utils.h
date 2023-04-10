#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

namespace utils {

using glm::mat4;
using glm::vec3;
using glm::vec4;

// Only exists to prove the glm works.
void PrintMatrix() {
  mat4 m{};
  m[0][0] = 3;
  m[1][0] = 2.5;
  m[0][1] = -1;
  m[1][1] = m[1][0] + m[0][1];
  m[2] = vec4(vec3(3), 5);
  std::cout << glm::to_string(m) << std::endl;
}

}  // namespace utils
