#include "pose.h"

#include "biped-rig.h"
#include "object.h"

Pose Pose::freeze(const BipedRig& rig) {
  Pose p;
  for (uint32_t i = 0; i < kBoneCount + kDirCount; i++) {
    BoneId id = static_cast<BoneId>(i);
    if (i < kBoneCount) {
      p.setTransform(id, rig.getBone(id)->getTransform());
    } else {
      p.setDir(id, rig.getIkDir(id));
    }
  }
  return p;
}

Pose Pose::blend(const Pose& p1, const Pose& p2, float a) {
  DASSERT(p1.type == PoseType::Override);
  Pose p = p1;

  for (uint32_t i = 0; i < kBoneCount + kDirCount; i++) {
    BoneId bone = static_cast<BoneId>(i);

    if (p2.bone_mask && !p2.bone_mask->contains(bone)) {
      continue;
    }

    if (i < kBoneCount) {
      const Transform& t1 = p1.getTransform(bone);
      const Transform& t2 = p2.getTransform(bone);
      if (p2.type == PoseType::Additive) {
        p.setTransform(bone, Transform::addBlend(t1, t2, a));
      } else {
        p.setTransform(bone, Transform::blend(t1, t2, a));
      }
    } else {
      const vec3& dir1 = p1.getDir(bone);
      const vec3& dir2 = p2.getDir(bone);
      if (p2.type == PoseType::Additive) {
        p.setDir(bone, glm::normalize(glm::mix(dir1, dir1 + dir2, a)));
      } else {
        p.setDir(bone, glm::normalize(glm::mix(dir1, dir2, a)));
      }
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

size_t Pose::getDirIndex(BoneId id) const {
  size_t i = static_cast<size_t>(id);
  DASSERT(i >= kFirstDir);
  DASSERT(i < kFirstDir + kDirCount);
  return i - kFirstDir;
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

const vec3& Pose::getDir(BoneId id) const {
  return ik_dirs[getDirIndex(id)];
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

void Pose::setDir(BoneId id, const vec3& dir) {
  ik_dirs[getDirIndex(id)] = dir;
}
