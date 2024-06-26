#pragma once

#include "glm-include.h"

class Transform {
 public:
  Transform() = default;
  Transform(const vec3& pos, const vec3& scale, const quat& rot)
      : pos_(pos), scale_(scale), rot_(rot) {
  }

  static Transform makeAdditive() {
    // Notably, scale should be 0, not 1.
    return Transform(vec3(0), vec3(0), glm::identity<quat>());
  }

  void setPos(const vec3& pos);
  void setScale(const vec3& scale);
  void setRot(const quat& rot);
  const vec3& getPos() const;
  const vec3& getScale() const;
  const quat& getRot() const;
  const mat4& matrix();

  static Transform blend(const Transform& t1, const Transform& t2, float a);
  static Transform addBlend(const Transform& t1, const Transform& t2, float a);

 private:
  void updateMatrix();

  bool dirty_ = true;
  vec3 pos_ = vec3(0);
  vec3 scale_ = vec3(1);
  quat rot_ = glm::identity<quat>();
  mat4 matrix_ = mat4(1);
};
