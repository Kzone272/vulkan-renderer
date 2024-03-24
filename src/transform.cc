#include "transform.h"

void Transform::setPos(const vec3& pos) {
  pos_ = pos;
}
void Transform::setScale(const vec3& scale) {
  scale_ = scale;
}
void Transform::setRot(const quat& rot) {
  rot_ = rot;
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

mat4 Transform::matrix() {
  return glm::translate(pos_) * glm::toMat4(rot_) * glm::scale(scale_);
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
