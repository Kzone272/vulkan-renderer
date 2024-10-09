#include "pose.h"

#include "biped-rig.h"
#include "object.h"

Pose Pose::freeze(const BipedRig& rig) {
  Pose p;
  for (size_t i = 0; i < kBoneCount; i++) {
    BoneId id = static_cast<BoneId>(i);
    p.setTransform(id, rig.getBone(id)->getTransform());
  }
  return p;
}

Pose Pose::blend(const Pose& p1, const Pose& p2, float a) {
  DASSERT(p1.type == PoseType::Override);
  Pose p = p1;

  for (size_t i = 0; i < kBoneCount; i++) {
    BoneId bone = static_cast<BoneId>(i);

    if (p2.bone_mask && !p2.bone_mask->contains(bone)) {
      continue;
    }

    const Transform& t1 = p1.getTransform(bone);
    const Transform& t2 = p2.getTransform(bone);
    if (p2.type == PoseType::Additive) {
      p.setTransform(bone, Transform::addBlend(t1, t2, a));
    } else {
      p.setTransform(bone, Transform::blend(t1, t2, a));
    }
  }

  return p;
}

size_t Pose::getBoneIndex(BoneId id) const {
  size_t i = static_cast<size_t>(id);
  DASSERT(i >= 0);
  DASSERT(i < kBoneCount);
  return i;
}

const vec3& Pose::getPos(BoneId bone) const {
  return bone_ts[getBoneIndex(bone)].getPos();
}

const Transform& Pose::getTransform(BoneId bone) const {
  DASSERT(!bone_mask || bone_mask->contains(bone));
  return bone_ts[getBoneIndex(bone)];
}

const mat4& Pose::getMatrix(BoneId bone) {
  return bone_ts[getBoneIndex(bone)].matrix();
}

void Pose::setTransform(BoneId bone, const Transform& t) {
  bone_ts[getBoneIndex(bone)] = t;
}

void Pose::setPos(BoneId bone, const vec3& pos) {
  bone_ts[getBoneIndex(bone)].setPos(pos);
}

void Pose::setScale(BoneId bone, const vec3& scale) {
  bone_ts[getBoneIndex(bone)].setScale(scale);
}

void Pose::setRot(BoneId bone, const quat& rot) {
  bone_ts[getBoneIndex(bone)].setRot(rot);
}
