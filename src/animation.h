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

struct Spline {
  enum class Type {
    LINEAR,
    BEZIER,  // cubic
  };
  Type type;
  std::vector<vec3> points;
  int segments = 0;
};

Spline makeSpline(Spline::Type type, const std::vector<vec3>& points) {
  if (type == Spline::Type::LINEAR) {
    return {
        .type = type,
        .points = points,
        .segments = static_cast<int>(points.size() - 1),
    };
  }

  printf("Unsupported spline type!\n");
  ASSERT(false);
  return {};
}

vec3 sampleSpline(const Spline& spline, float u) {
  if (spline.type == Spline::Type::LINEAR) {
    if (u <= 0) {
      return spline.points[0];
    }
    if (u >= spline.segments) {
      return spline.points.back();
    }
    const vec3& a = spline.points[floor(u)];
    const vec3& b = spline.points[ceil(u)];
    float t = glm::fract(u);
    return glm::mix(a, b, t);
  }

  printf("Unsupported spline type!\n");
  ASSERT(false);
  return {};
};

struct Animation {
  Spline spline;
  Time from_time;
  Time to_time;

  static vec3 sample(const Animation& a, Time now) {
    float pct = FloatMs(now - a.from_time) / FloatMs(a.to_time - a.from_time);
    float u = a.spline.segments * pct;

    return sampleSpline(a.spline, u);
  }
};

Animation makeAnimation(Spline& spline, float dur_ms, Time start) {
  return Animation{
      .spline = spline,
      .from_time = start,
      .to_time =
          start + std::chrono::duration_cast<Clock::duration>(FloatMs{dur_ms}),
  };
}
