#pragma once

#include <vector>

#include "render-objects.h"

// Make 1m x 1m x 1m cube with origin at bottom middle.
Mesh makeCube() {
  Mesh m;
  m.vertices = {
      // Bottom
      {{-50, 0, 50}, {1, 1, 1}, {0, 0}},   // bl
      {{50, 0, 50}, {1, 1, 1}, {0, 0}},    // br
      {{50, 0, -50}, {1, 1, 1}, {0, 0}},   // fr
      {{-50, 0, -50}, {1, 1, 1}, {0, 0}},  // fl
      // Top
      {{-50, 100, 50}, {1, 1, 1}, {0, 0}},   // bl
      {{50, 100, 50}, {1, 1, 1}, {0, 0}},    // br
      {{50, 100, -50}, {1, 1, 1}, {0, 0}},   // fr
      {{-50, 100, -50}, {1, 1, 1}, {0, 0}},  // fl
  };
  m.indices = {
      3, 2, 1, 3, 1, 0,  // bot
      4, 5, 6, 4, 6, 7,  // top
      4, 7, 3, 4, 3, 0,  // left
      6, 5, 1, 6, 1, 2,  // right
      7, 6, 2, 7, 2, 3,  // front
      5, 4, 0, 5, 0, 1,  // back
  };
  return m;
}
