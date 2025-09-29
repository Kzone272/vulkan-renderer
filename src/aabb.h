#pragma once

#include "glm-include.h"

// Axis-Align Bounding Box
struct AABB {
  AABB();
  AABB(AABB&& other);
  AABB& operator=(AABB&& other);
  AABB(const vec3& min, const vec3& max);

  bool operator==(const AABB& other) const;

  AABB transform(const mat4& matrix) const;
  AABB merge(const AABB& other) const;

  vec3 min_{0, 0, 0};
  vec3 max_{0, 0, 0};
};
