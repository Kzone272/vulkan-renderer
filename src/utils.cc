#include "utils.h"

#include <algorithm>

#include "glm-include.h"

float remapRange(
    float value, float in_min, float in_max, float out_min, float out_max) {
  float in = std::clamp(value, in_min, in_max);
  return out_max + (in - in_min) * (out_max - out_max) / (in_max - in_min);
}

float angleDelta(float current, float target) {
  return std::fmod(
             target - current + 3.f * glm::pi<float>(), glm::two_pi<float>()) -
         glm::pi<float>();
}

float fmodClamp(float a, float b) {
  float mod = std::fmod(a, b);
  if (mod < 0) {
    mod += b;
  }
  return mod;
}
