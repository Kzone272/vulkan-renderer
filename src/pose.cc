#include "pose.h"

#include "biped-rig.h"
#include "object.h"

Pose Pose::freeze(const BipedRig& rig) {
  Pose p;
  for (uint32_t i = 0; i < kBoneCount; i++) {
    BoneId bone = static_cast<BoneId>(i);
    p.setTransform(bone, rig.getBone(bone)->getTransform());
  }
  return p;
}

Pose Pose::blend(const Pose& p1, const Pose& p2, float a) {
  DASSERT(p1.type == PoseType::Override);
  Pose p = p1;

  for (uint32_t i = 0; i < kBoneCount; i++) {
    BoneId bone = static_cast<BoneId>(i);

    if (!p2.bone_mask || p2.bone_mask->contains(bone)) {
      if (p2.type == PoseType::Additive) {
        p.setTransform(
            bone, Transform::addBlend(p1.bone_ts[i], p2.bone_ts[i], a));
      } else {
        p.setTransform(bone, Transform::blend(p1.bone_ts[i], p2.bone_ts[i], a));
      }
    }
  }

  for (auto& entry : p2.ik_dirs) {
    const IkChain* ik = entry.first;
    const vec3& dir2 = entry.second;

    auto ik1_it = p1.ik_dirs.find(ik);
    if (ik1_it != p1.ik_dirs.end()) {
      const vec3& dir1 = ik1_it->second;
      if (p2.type == PoseType::Additive) {
        p.ik_dirs.insert_or_assign(ik, glm::mix(dir1, dir1 + dir2, a));
      } else {
        p.ik_dirs.insert_or_assign(ik, glm::mix(dir1, dir2, a));
      }
    } else {
      p.ik_dirs.insert_or_assign(ik, dir2);
    }
  }

  return p;
}

const vec3& Pose::getPos(BoneId bone) const {
  DASSERT(bone < BoneId::COUNT);
  return bone_ts[static_cast<uint32_t>(bone)].getPos();
}

const Transform& Pose::getTransform(BoneId bone) const {
  DASSERT(bone < BoneId::COUNT);
  DASSERT(!bone_mask || bone_mask->contains(bone));
  return bone_ts[static_cast<uint32_t>(bone)];
}

const mat4& Pose::getMatrix(BoneId bone) {
  DASSERT(bone < BoneId::COUNT);
  return bone_ts[static_cast<uint32_t>(bone)].matrix();
}

void Pose::setTransform(BoneId bone, const Transform& t) {
  DASSERT(bone < BoneId::COUNT);
  bone_ts[static_cast<uint32_t>(bone)] = t;
}

void Pose::setPos(BoneId bone, const vec3& pos) {
  DASSERT(bone < BoneId::COUNT);
  bone_ts[static_cast<uint32_t>(bone)].setPos(pos);
}

void Pose::setScale(BoneId bone, const vec3& scale) {
  DASSERT(bone < BoneId::COUNT);
  bone_ts[static_cast<uint32_t>(bone)].setScale(scale);
}

void Pose::setRot(BoneId bone, const quat& rot) {
  DASSERT(bone < BoneId::COUNT);
  bone_ts[static_cast<uint32_t>(bone)].setRot(rot);
}
