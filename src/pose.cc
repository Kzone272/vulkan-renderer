#include "pose.h"

#include "biped-rig.h"
#include "object.h"

Pose Pose::freeze(const BipedRig& rig) {
  Pose p;
  std::vector<Object*> all_bones = {
      rig.cog_,   rig.neck_,  rig.head_,   rig.lsho_, rig.rsho_,
      rig.lhand_, rig.rhand_, rig.pelvis_, rig.lhip_, rig.rhip_,
      rig.lfoot_, rig.rfoot_, rig.ltoe_,   rig.rtoe_,
  };
  for (auto* bone : all_bones) {
    p.setBone(bone, bone->getTransform());
  }
  return p;
}

Pose Pose::blend(const Pose& p1, const Pose& p2, float a) {
  DASSERT(p1.type == PoseType::Override);
  Pose p = p1;

  for (auto& entry : p2.bone_ts) {
    const Object* bone = entry.first;
    const Transform& t2 = entry.second;

    auto* t1 = p1.maybeGetBone(bone);
    DASSERT(t1);
    if (p2.type == PoseType::Additive) {
      p.setBone(bone, Transform::addBlend(*t1, t2, a));
    } else {
      p.setBone(bone, Transform::blend(*t1, t2, a));
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

Transform& Pose::getBone(const Object* bone) {
  return bone_ts[bone];
}

const Transform* Pose::maybeGetBone(const Object* bone) const {
  auto it = bone_ts.find(bone);
  if (it != bone_ts.end()) {
    return &it->second;
  } else {
    return nullptr;
  }
}

void Pose::setBone(const Object* bone, const Transform& t) {
  getBone(bone) = t;
}
