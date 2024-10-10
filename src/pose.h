#pragma once

#include <map>
#include <optional>
#include <set>

#include "glm-include.h"
#include "skeleton.h"
#include "transform.h"

enum class PoseType {
  Override,
  Additive,
};

class Object;
struct IkChain;
struct BipedRig;

struct Pose {
  Pose() = default;
  Pose(Skeleton* skl, PoseType type = PoseType::Override)
      : skl(skl),
        bone_count(skl->count()),
        type(type),
        bone_ts(
            bone_count, type == PoseType::Additive ? Transform::makeAdditive()
                                                   : Transform()) {
  }

  static Pose blend(const Pose& p1, const Pose& p2, float a);

  void setPos(size_t i, const vec3& pos);
  void setScale(size_t i, const vec3& scale);
  void setRot(size_t i, const quat& rot);
  void setTransform(size_t i, const Transform& t);

  const vec3& getPos(size_t i) const;
  const Transform& getTransform(size_t i) const;
  const mat4& getMatrix(size_t i);

  void computeRootMatrices();
  const mat4& getRootMatrix(size_t i) const;
  vec3 getRootPos(size_t i) const;
  const std::vector<mat4>& getRootMatrices() const {
    return root_ms_;
  }

  size_t bone_count = 0;
  Skeleton* skl = nullptr;
  PoseType type = PoseType::Override;
  std::vector<Transform> bone_ts = std::vector<Transform>(bone_count);
  std::vector<mat4> root_ms_;
  std::optional<std::set<size_t>> bone_mask;
};
