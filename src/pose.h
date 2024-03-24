#pragma once

#include <map>

#include "glm-include.h"
#include "transform.h"

enum class PoseType {
  Override,
  Additive,
};

class Object;
struct IkChain;
struct BipedRig;

struct Pose {
  PoseType type = PoseType::Override;
  std::map<const Object*, Transform> bone_ts;
  std::map<const IkChain*, vec3> ik_dirs;

  void setBone(const Object* bone, const Transform& t);
  Transform& getBone(const Object* bone);
  const Transform* maybeGetBone(const Object* bone) const;

  static Pose freeze(const BipedRig& rig);
  static Pose blend(const Pose& p1, const Pose& p2, float a);
};
