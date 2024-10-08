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
  Lankle,
  Rankle,
  Lball,
  Rball,
  // Dirs
  Lelbow,
  Relbow,
  Lknee,
  Rknee,
  COUNT,
};
constexpr size_t kBoneCount = static_cast<size_t>(BoneId::COUNT);

struct Pose {
  Pose() = default;
  Pose(PoseType type)
      : type(type),
        bone_ts(
            kBoneCount, type == PoseType::Additive ? Transform::makeAdditive()
                                                   : Transform()) {
  }

  static Pose freeze(const BipedRig& rig);
  static Pose blend(const Pose& p1, const Pose& p2, float a);

  size_t getBoneIndex(BoneId id) const;

  void setPos(BoneId bone, const vec3& pos);
  void setScale(BoneId bone, const vec3& scale);
  void setRot(BoneId bone, const quat& rot);
  void setTransform(BoneId bone, const Transform& t);

  const vec3& getPos(BoneId bone) const;
  const Transform& getTransform(BoneId bone) const;
  const mat4& getMatrix(BoneId bone);

  PoseType type = PoseType::Override;
  std::vector<Transform> bone_ts = std::vector<Transform>(kBoneCount);
  std::optional<std::set<BoneId>> bone_mask;
};
