#include "aabb.h"

#include <limits>
#include <print>

AABB::AABB() = default;
AABB::AABB(AABB&& other) = default;
AABB& AABB::operator=(AABB&& other) = default;
AABB::AABB(const vec3& min, const vec3& max) : min_(min), max_(max) {
}

bool AABB::operator==(const AABB& other) const {
  return min_ == other.min_ && max_ == other.max_;
};

AABB AABB::transform(const mat4& matrix) const {
  vec3 newMin(std::numeric_limits<float>::max());
  vec3 newMax(std::numeric_limits<float>::lowest());

  for (uint8_t i = 0; i < 8; i++) {
    vec3 corner{
        i & 1 ? min_.x : max_.x,
        i & 2 ? min_.y : max_.y,
        i & 4 ? min_.z : max_.z,
    };
    corner = matrix * vec4(corner, 1);
    newMin = glm::min(newMin, corner);
    newMax = glm::max(newMax, corner);
  }
  return AABB(newMin, newMax);
}

AABB AABB::merge(const AABB& other) const {
  return AABB(glm::min(min_, other.min_), glm::max(max_, other.max_));
}
