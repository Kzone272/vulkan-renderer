#include "transform.h"

void Transform::setPos(const vec3& pos) {
  pos_ = pos;
  dirty_ = true;
}
void Transform::setScale(const vec3& scale) {
  scale_ = scale;
  dirty_ = true;
}
void Transform::setRot(const quat& rot) {
  rot_ = rot;
  dirty_ = true;
}

const vec3& Transform::getPos() const {
  return pos_;
}
const vec3& Transform::getScale() const {
  return scale_;
}
const quat& Transform::getRot() const {
  return rot_;
}

const mat4& Transform::matrix() {
  if (dirty_) {
    updateMatrix();
    dirty_ = false;
  }
  return matrix_;
}

void Transform::updateMatrix() {
  // Equivalent to, but faster than:
  //   glm::translate(pos_) * glm::toMat4(rot_) * glm::scale(scale_);
  matrix_ = glm::toMat4(rot_);
  matrix_[0] *= scale_[0];
  matrix_[1] *= scale_[1];
  matrix_[2] *= scale_[2];
  matrix_[3][0] = pos_[0];
  matrix_[3][1] = pos_[1];
  matrix_[3][2] = pos_[2];
}

Transform Transform::blend(const Transform& t1, const Transform& t2, float a) {
  Transform t;
  t.pos_ = glm::mix(t1.pos_, t2.pos_, a);
  t.scale_ = glm::mix(t1.scale_, t2.scale_, a);
  t.rot_ = glm::mix(t1.rot_, t2.rot_, a);
  return t;
}

Transform Transform::addBlend(
    const Transform& t1, const Transform& t2, float a) {
  Transform t;
  t.pos_ = glm::mix(t1.pos_, t1.pos_ + t2.pos_, a);
  t.scale_ = glm::mix(t1.scale_, t1.scale_ + t2.scale_, a);
  t.rot_ = glm::mix(t1.rot_, t2.rot_ * t1.rot_, a);
  return t;
}
