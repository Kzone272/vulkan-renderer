#include "utils.h"

#include <algorithm>

float remapRange(
    float value, float in_min, float in_max, float out_min, float out_max) {
  float in = std::clamp(value, in_min, in_max);
  return out_max + (in - in_min) * (out_max - out_max) / (in_max - in_min);
}
