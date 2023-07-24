#pragma once

#include <chrono>
#include <map>

#include "asserts.h"
#include "glm-include.h"
#include "time-include.h"
#include "utils.h"

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

enum class SplineType {
  Linear,
  Bezier,  // cubic
  Hermite,
};

template <class T>
struct Spline {
  SplineType type;
  std::vector<T> points;
  int segments = 0;
};

template <class T>
Spline<T> makeSpline(SplineType type, const std::vector<T>& points) {
  int npoints = static_cast<int>(points.size());
  if (type == SplineType::Linear) {
    return {
        .type = type,
        .points = points,
        .segments = npoints - 1,
    };
  } else if (type == SplineType::Bezier) {
    ASSERT((npoints - 1) % 3 == 0);
    return {
        .type = type,
        .points = points,
        .segments = (npoints - 1) / 3,
    };
  } else if (type == SplineType::Hermite) {
    ASSERT((npoints % 2) == 0);
    std::vector<T> controls = points;
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

std::map<SplineType, mat4> spline_mats{
    {
        SplineType::Bezier,
        mat4(1, -3, 3, -1, 0, 3, -6, 3, 0, 0, 3, -3, 0, 0, 0, 1),
    },
    {
        SplineType::Hermite,
        mat4(1, 0, -3, 2, 0, 1, -2, 1, 0, 0, 3, -2, 0, 0, -1, 1),
    },
};

template <class T>
T weight(vec4 weights, const T& a, const T& b, const T& c, const T& d) {
  glm::mat<4, T::length(), float, glm::qualifier::defaultp> points = {
      a, b, c, d};
  return weights * glm::transpose(points);
}

template <>
float weight(
    vec4 weights, const float& a, const float& b, const float& c,
    const float& d) {
  vec4 points{a, b, c, d};
  return glm::dot(weights, points);
}

template <class T>
T blend(
    SplineType type, float t, const T& a, const T& b, const T& c, const T& d) {
  ASSERT(spline_mats.contains(type));
  const mat4& bersntein = spline_mats[type];
  vec4 ts(1, t, t * t, t * t * t);

  vec4 weights = ts * bersntein;
  return weight(weights, a, b, c, d);
}

template <class T>
T sampleSpline(const Spline<T>& spline, float u) {
  if (spline.type == SplineType::Linear) {
    if (u <= 0) {
      return spline.points[0];
    }
    if (u >= spline.segments) {
      return spline.points.back();
    }
    const T& a = spline.points[floor(u)];
    const T& b = spline.points[ceil(u)];
    float t = glm::fract(u);
    return glm::mix(a, b, t);
  } else if (spline.type == SplineType::Bezier) {
    if (u <= 0) {
      return spline.points[0];
    }
    if (u >= spline.segments) {
      return spline.points.back();
    }
    int i = floor(u) * 3;
    const T& a = spline.points[i];
    const T& b = spline.points[i + 1];
    const T& c = spline.points[i + 2];
    const T& d = spline.points[i + 3];
    float t = glm::fract(u);
    return blend(SplineType::Bezier, t, a, b, c, d);
  } else if (spline.type == SplineType::Hermite) {
    if (u <= 0) {
      return spline.points[0];
    }
    if (u >= spline.segments) {
      return spline.points[spline.points.size() - 2];
    }
    int i = floor(u) * 2;
    const T& a = spline.points[i];
    // How am I scaling a ref? Am I referencing a temporary?
    // This scaling should most likely be done in makeAnimation().
    const T& b = spline.points[i + 1] / static_cast<float>(spline.segments);
    const T& c = spline.points[i + 2];
    const T& d = spline.points[i + 3] / static_cast<float>(spline.segments);
    float t = glm::fract(u);
    return blend(SplineType::Hermite, t, a, b, c, d);
  }

  printf("Unsupported spline type!\n");
  ASSERT(false);
  return {};
};

template <class T>
struct Animation {
  Spline<T> spline;
  float dur_ms;
  Time from_time;
  Time to_time;
  bool loop = false;
  // Used only for rotation animations.
  vec3 axis;
};

template <class T>
Animation<T> makeAnimation(
    Spline<T>& spline, float dur_ms, Time start, bool loop = false,
    vec3 axis = {0, 1, 0}) {
  return Animation{
      .spline = spline,
      .dur_ms = dur_ms,
      .from_time = start,
      .to_time = addMs(start, dur_ms),
      .loop = loop,
      .axis = axis,
  };
}

template <class T>
static T sampleAnimation(const Animation<T>& a, Time now) {
  float time_ms = FloatMs(now - a.from_time).count();
  if (a.loop) {
    time_ms = fmodClamp(time_ms, a.dur_ms);
  }
  float pct = time_ms / a.dur_ms;
  float u = a.spline.segments * pct;

  return sampleSpline(a.spline, u);
}
