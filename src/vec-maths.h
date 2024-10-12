#pragma once

#include "glm-include.h"

float fromSrgb(float component) {
  if (component <= 0.04045) {
    return component / 12.92;
  } else {
    return pow((component + 0.055) / 1.055, 2.4);
  }
}

vec3 fromSrgb(vec3 srgb) {
  return vec3(fromSrgb(srgb.r), fromSrgb(srgb.g), fromSrgb(srgb.b));
}

vec3 fromHex(uint32_t hex) {
  uint8_t r = (hex >> 16) & 0xFF;
  uint8_t g = (hex >> 8) & 0xFF;
  uint8_t b = hex & 0xFF;
  return vec3(fromSrgb(r / 255.f), fromSrgb(g / 255.f), fromSrgb(b / 255.f));
}

// Perspective projection matrix using reverse-z and an infinite far plane.
// Details: https://iolite-engine.com/blog_posts/reverse_z_cheatsheet
// Why: https://developer.nvidia.com/content/depth-precision-visualized
mat4 perspectiveInfRevZ(float fovy, float aspect, float z_near) {
  // based on glm::perspective() impl.
  DASSERT(abs(aspect - std::numeric_limits<float>::epsilon()) > 0.f);

  float tan_half_fovy = tan(fovy / 2.f);

  mat4 proj(0);
  proj[0][0] = 1.f / (aspect * tan_half_fovy);
  proj[1][1] = -1.f / (tan_half_fovy);  // Negative because y is down in Vulkan
  proj[2][3] = 1.f;
  proj[3][2] = z_near;
  return proj;
}
