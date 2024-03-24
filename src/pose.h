#pragma once

#include <map>
#include <optional>
#include <set>

#include "glm-include.h"
#include "transform.h"

enum class PoseType {
  Override,
  Additive,
};

class Object;
struct IkChain;
struct BipedRig;

enum class BoneId : uint32_t {
  Cog,
  Neck,
  Head,
  Lhand,
  Rhand,
  Pelvis,
  Lfoot,
  Rfoot,
  COUNT,
};

struct Pose {
  PoseType type = PoseType::Override;
  std::vector<Transform> bone_ts =
      std::vector<Transform>(static_cast<size_t>(BoneId::COUNT));
  std::optional<std::set<BoneId>> bone_mask;
  std::map<const IkChain*, vec3> ik_dirs;

  void setBone(BoneId bone, const Transform& t);
  const Transform& getBoneConst(BoneId bone) const;
  Transform& getBone(BoneId bone);

  static Pose freeze(const BipedRig& rig);
  static Pose blend(const Pose& p1, const Pose& p2, float a);
};
