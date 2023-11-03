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
