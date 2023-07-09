#pragma once

#include <chrono>

#include "asserts.h"
#include "glm-include.h"
#include "time-include.h"

enum class BlendType {
  Linear,
};

namespace anim {

float blend(float pct, BlendType blend) {
  if (blend == BlendType::Linear) {
    return pct;
  }

  printf("Unsupported blend type!\n");
  ASSERT(false);
  return 0;
}

}  // namespace anim

struct Animation {
  float* value;  // Maybe delete this.
  // TODO: This should probably be an array of to/from values and t values.
  // Duration to/from time should be an overall concept.
  // Maybe also a field enum like X,Y,Z instead of storing a value*?
  float from_val;
  Time from_time;
  float to_val;
  Time to_time;
  BlendType blend;

  static float sample(const Animation& a, Time now) {
    if (now > a.to_time) {
      return a.to_val;
    }

    float pct = FloatMs(now - a.from_time) / FloatMs(a.to_time - a.from_time);
    float t = anim::blend(pct, a.blend);

    return glm::mix(a.from_val, a.to_val, pct);
  }

  // Returns true if animation ended.
  static bool update(Animation& a, Time now) {
    *a.value = Animation::sample(a, now);
    if (*a.value == a.to_val) {
      return true;
    } else {
      return false;
    }
  }
};

Animation makeAnimation(
    float from, float to, float dur_ms, Time start,
    BlendType blend = BlendType::Linear) {
  return Animation{
      .value = nullptr,
      .from_val = from,
      .from_time = start,
      .to_val = to,
      .to_time =
          start + std::chrono::duration_cast<Clock::duration>(FloatMs{dur_ms}),
      .blend = blend,
  };
}
