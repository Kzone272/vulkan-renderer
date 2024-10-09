#include "pose.h"

#include "biped-rig.h"
#include "object.h"

Pose Pose::blend(const Pose& p1, const Pose& p2, float a) {
  DASSERT(p1.skl == p2.skl);
  DASSERT(p1.type == PoseType::Override);
  Pose p = p1;

  for (size_t i = 0; i < p1.bone_count; i++) {
    if (p2.bone_mask && !p2.bone_mask->contains(i)) {
      continue;
    }

    const Transform& t1 = p1.getTransform(i);
    const Transform& t2 = p2.getTransform(i);
    if (p2.type == PoseType::Additive) {
      p.setTransform(i, Transform::addBlend(t1, t2, a));
    } else {
      p.setTransform(i, Transform::blend(t1, t2, a));
    }
  }

  return p;
}

const vec3& Pose::getPos(size_t i) const {
  return bone_ts[i].getPos();
}

const Transform& Pose::getTransform(size_t i) const {
  DASSERT(!bone_mask || bone_mask->contains(i));
  return bone_ts[i];
}

const mat4& Pose::getMatrix(size_t i) {
  return bone_ts[i].matrix();
}

void Pose::setTransform(size_t i, const Transform& t) {
  bone_ts[i] = t;
}

void Pose::setPos(size_t i, const vec3& pos) {
  bone_ts[i].setPos(pos);
}

void Pose::setScale(size_t i, const vec3& scale) {
  bone_ts[i].setScale(scale);
}

void Pose::setRot(size_t i, const quat& rot) {
  bone_ts[i].setRot(rot);
}

void Pose::computeRootMatrices() {
  if (root_ms_.empty()) {
    root_ms_.resize(bone_count);
  }
  for (size_t i = 0; i < bone_count; i++) {
    mat4& m = root_ms_[i];
    m = getMatrix(i);
    size_t parent = skl->parent(i);
    if (parent != kNoParent) {
      m = root_ms_[parent] * m;
    }
  }
}

const mat4& Pose::getRootMatrix(size_t i) {
  return root_ms_[i];
}

vec3 Pose::getRootPos(size_t i) {
  return vec3(getRootMatrix(i) * vec4(0, 0, 0, 1));
}
