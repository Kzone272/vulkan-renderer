#pragma once

#include <chrono>
#include <map>
#include <print>

#include "asserts.h"
#include "glm-include.h"
#include "maths.h"
#include "time-include.h"
#include "time-utils.h"

enum class BlendType {
  Linear,
};

enum class SplineType {
  Linear,
  Bezier,  // cubic
  Hermite,
};

template <class T>
struct Spline {
  Spline() {
  }
  Spline(SplineType type, const std::vector<T>& points);
  T sample(float u);

  SplineType type_;
  std::vector<T> points_;
  int segments_ = 0;
};

template <class T>
struct Animation {
  Animation() {
  }
  Animation(
      const Spline<T>& spline, float dur_ms, Time start, bool loop = false);
  T sample(Time now);

  Spline<T> spline_;
  float dur_ms_;
  Time from_time_;
  Time to_time_;
  bool loop_ = false;
};

namespace {

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

}  // namespace

template <class T>
Spline<T>::Spline(SplineType type, const std::vector<T>& points)
    : type_(type), points_(points) {
  int npoints = static_cast<int>(points.size());
  if (type == SplineType::Linear) {
    segments_ = npoints - 1;
  } else if (type == SplineType::Bezier) {
    ASSERT((npoints - 1) % 3 == 0);
    segments_ = (npoints - 1) / 3;
  } else if (type == SplineType::Hermite) {
    ASSERT((npoints % 2) == 0);
    segments_ = (npoints / 2) - 1;
  } else {
    std::println("Unsupported spline type!");
    ASSERT(false);
  }
}

template <class T>
T Spline<T>::sample(float u) {
  if (type_ == SplineType::Linear) {
    if (u <= 0) {
      return points_[0];
    }
    if (u >= segments_) {
      return points_.back();
    }
    const T& a = points_[floor(u)];
    const T& b = points_[ceil(u)];
    float t = glm::fract(u);
    return glm::mix(a, b, t);
  } else if (type_ == SplineType::Bezier) {
    if (u <= 0) {
      return points_[0];
    }
    if (u >= segments_) {
      return points_.back();
    }
    int i = floor(u) * 3;
    const T& a = points_[i];
    const T& b = points_[i + 1];
    const T& c = points_[i + 2];
    const T& d = points_[i + 3];
    float t = glm::fract(u);
    return blend(SplineType::Bezier, t, a, b, c, d);
  } else if (type_ == SplineType::Hermite) {
    if (u <= 0) {
      return points_[0];
    }
    if (u >= segments_) {
      return points_[points_.size() - 2];
    }
    int i = floor(u) * 2;
    const T& a = points_[i];
    const T& b = points_[i + 1];
    const T& c = points_[i + 2];
    const T& d = points_[i + 3];
    float t = glm::fract(u);
    return blend(SplineType::Hermite, t, a, b, c, d);
  } else {
    std::println("Unsupported spline type!");
    ASSERT(false);
    return {};
  }
};

template <class T>
Animation<T>::Animation(
    const Spline<T>& spline, float dur_ms, Time start, bool loop)
    : spline_(spline),
      dur_ms_(dur_ms),
      from_time_(start),
      to_time_(addMs(start, dur_ms)),
      loop_(loop) {
  // We need to scale the velocities of Hermite splines based on duration of
  // each segment.
  // Notes on scaling: https://www.cubic.org/docs/hermite.htm
  if (spline_.type_ == SplineType::Hermite) {
    for (int i = 1; i < spline_.points_.size(); i += 2) {
      spline_.points_[i] *= dur_ms_ / spline_.segments_ / 1000.f;
    }
  }
}

template <class T>
T Animation<T>::sample(Time now) {
  float time_ms = FloatMs(now - from_time_).count();
  if (loop_) {
    time_ms = fmodClamp(time_ms, dur_ms_);
  }
  float pct = time_ms / dur_ms_;
  float u = spline_.segments_ * pct;

  return spline_.sample(u);
}
