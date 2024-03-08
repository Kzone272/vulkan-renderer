#pragma once

#include <optional>

#include "glm-include.h"
#include "time-include.h"

// References:
// https://www.youtube.com/watch?v=KPoeNZZ6H4s
// https://en.wikipedia.org/wiki/Differential_equation#Second_order
// https://en.wikipedia.org/wiki/Verlet_integration

template <class T>
class SecondOrder {
 public:
  // params.x: frequency
  // params.y: damping
  // params.z: response
  SecondOrder(vec3 params, T x0) {
    updateParams(params);
    x_ = x0;
    y_ = x0;
    dy_ = T{0};
  }

  void updateParams(vec3 params) {
    float freq = params.x;
    float damp = params.y;
    float response = params.z;

    float f_pi = freq * glm::pi<float>();
    k1_ = damp / f_pi;
    k2_ = 1 / std::pow(2 * f_pi, 2);
    k3_ = response * damp / (2 * f_pi);
  }

  T update(float t, T x) {
    T dx = (x - x_) / t;
    x_ = x;

    float k2_stable = std::max(std::max(k2_, t * t / 2 + t * k1_ / 2), t * k1_);
    y_ = y_ + t * dy_;
    dy_ = dy_ + t * (x + k3_ * dx - y_ - k1_ * dy_) / k2_stable;
    return y_;
  }

 private:
  T x_;
  T y_;
  T dy_;
  float k1_;
  float k2_;
  float k3_;
};
