#pragma once

#include <chrono>
#include <map>

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
    HERMITE,
  };
  Type type;
  std::vector<vec3> points;
  int segments = 0;
};

Spline makeSpline(Spline::Type type, const std::vector<vec3>& points) {
  int npoints = static_cast<int>(points.size());
  if (type == Spline::Type::LINEAR) {
    return {
        .type = type,
        .points = points,
        .segments = npoints - 1,
    };
  } else if (type == Spline::Type::BEZIER) {
    ASSERT((npoints - 1) % 3 == 0);
    return {
        .type = type,
        .points = points,
        .segments = (npoints - 1) / 3,
    };
  } else if (type == Spline::Type::HERMITE) {
    ASSERT((npoints % 2) == 0);
    std::vector<vec3> controls = points;
    return {
        .type = type,
        .points = std::move(controls),
        .segments = (npoints / 2) - 1,
    };
  }

  printf("Unsupported spline type!\n");
  ASSERT(false);
  return {};
}

std::map<Spline::Type, mat4> spline_mats{
    {
        Spline::Type::BEZIER,
        mat4(1, -3, 3, -1, 0, 3, -6, 3, 0, 0, 3, -3, 0, 0, 0, 1),
    },
    {
        Spline::Type::HERMITE,
        mat4(1, 0, -3, 2, 0, 1, -2, 1, 0, 0, 3, -2, 0, 0, -1, 1),
    },
};

vec3 blend(
    Spline::Type type, float t, const vec3& a, const vec3& b, const vec3& c,
    const vec3& d) {
  ASSERT(spline_mats.contains(type));
  const mat4& bersntein = spline_mats[type];
  vec4 ts(1, t, t * t, t * t * t);

  mat4 points;
  points[0] = vec4(a, 1);
  points[1] = vec4(b, 1);
  points[2] = vec4(c, 1);
  points[3] = vec4(d, 1);
  auto p = ts * bersntein * glm::transpose(points);
  return vec3(p);
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
  } else if (spline.type == Spline::Type::BEZIER) {
    if (u <= 0) {
      return spline.points[0];
    }
    if (u >= spline.segments) {
      return spline.points.back();
    }
    int i = floor(u) * 3;
    const vec3& a = spline.points[i];
    const vec3& b = spline.points[i + 1];
    const vec3& c = spline.points[i + 2];
    const vec3& d = spline.points[i + 3];
    float t = glm::fract(u);
    return blend(Spline::Type::BEZIER, t, a, b, c, d);
  } else if (spline.type == Spline::Type::HERMITE) {
    if (u <= 0) {
      return spline.points[0];
    }
    if (u >= spline.segments) {
      return spline.points[spline.points.size() - 2];
    }
    int i = floor(u) * 2;
    const vec3& a = spline.points[i];
    const vec3& b = spline.points[i + 1] / static_cast<float>(spline.segments);
    const vec3& c = spline.points[i + 2];
    const vec3& d = spline.points[i + 3] / static_cast<float>(spline.segments);
    float t = glm::fract(u);
    return blend(Spline::Type::HERMITE, t, a, b, c, d);
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
