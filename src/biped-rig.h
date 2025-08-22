#pragma once

#include "glm-include.h"
#include "object.h"
#include "pose.h"
#include "render-objects.h"

struct SkellySizes;
struct BipedRig;

struct BipedSkeleton {
  enum Id : size_t {
    cog,
    pelvis,
    torso,
    lbicep,
    lforearm,
    lhand,
    rbicep,
    rforearm,
    rhand,
    head,
    lfemur,
    lshin,
    lfoot,
    ltoes,
    rfemur,
    rshin,
    rfoot,
    rtoes,
    COUNT,
    NoParent = kNoParent,
  };

  BipedSkeleton();
  // Not copyable
  BipedSkeleton(const BipedSkeleton& other) = delete;
  BipedSkeleton& operator=(const BipedSkeleton& other) = delete;

  void makeBones(const SkellySizes& sizes);
  const std::vector<mat4>& getMatrices(const mat4& parent);
  const std::vector<ModelId>& getModels() {
    return models_;
  }

  void setBone(Id bone, const vec3& pos, const mat4& model_t);
  void setPose(const std::vector<mat4>& pose) {
    curr_pose_ = pose;
  }
  const Pose& getZeroPose() {
    return zero_pose_;
  }

  Skeleton skl_ = {Id::COUNT};
  Pose zero_pose_ = {&skl_};

  std::vector<ModelId> models_ = {Id::COUNT, ModelId::Bone};
  std::vector<mat4> model_ts_ = {Id::COUNT, mat4(1)};
  std::vector<mat4> drawMats_ = {Id::COUNT, mat4(1)};

  std::vector<mat4> curr_pose_;
};

enum BipedRigId : size_t;

struct IkChain {
  IkChain() = default;
  IkChain(
      BipedRigId start, BipedRigId target, BipedRigId pole,
      const Pose& rig_zero_pose, BipedSkeleton::Id b1, BipedSkeleton::Id b2,
      float b1_l, float b2_l);
  void solve(const Pose& rig_pose, std::vector<mat4>& anim_pose);

  BipedRigId start;
  BipedRigId target;
  BipedRigId pole;
  // TODO: Support chain of bones.
  BipedSkeleton::Id b1;
  BipedSkeleton::Id b2;
  float b1_l;
  float b2_l;
  // Computed:
  // The normalized vector pointing from the start to the target when bones
  // are in zero-pose.
  vec3 point_zero;
  vec3 rot_axis;
};

enum BipedRigId : size_t {
  Cog,
  Neck,
  Head,
  Lsho,
  Rsho,
  Lhand,
  Rhand,
  Pelvis,
  Lhip,
  Rhip,
  Lankle,
  Rankle,
  Lball,
  Rball,
  Lelbow,
  Relbow,
  Lknee,
  Rknee,
  COUNT,
  NoParent = kNoParent,
};

struct BipedRig {
  using Id = BipedRigId;

  BipedRig();
  void makeRig(const Pose& anim_pose);
  Pose getZeroPose() {
    return zero_pose_;
  }
  void updateSkeleton(BipedSkeleton& skeleton, Pose& pose);
  void solveIk(const Pose& pose, std::vector<mat4>& anim_pose);
  Skeleton* skeleton() {
    return &skl_;
  };

  void setBone(Id bone, vec3 pos);
  BipedSkeleton::Id map(Id rig_id);

  const std::vector<mat4>& getMatrices(const mat4& parent, const Pose& pose);
  const std::vector<ModelId>& getModels() {
    return models_;
  }

  Skeleton skl_ = {Id::COUNT};
  Pose zero_pose_ = {&skl_};

  std::vector<ModelId> models_ = {Id::COUNT, ModelId::BallControl};
  std::vector<mat4> model_ts_ = {Id::COUNT, mat4(1)};
  std::vector<mat4> drawMats_ = {Id::COUNT, mat4(1)};

  IkChain larm_;
  IkChain rarm_;
  IkChain lleg_;
  IkChain rleg_;
  IkChain spine_;
  IkChain cerv_;  // Cervical spine (neck to head)
};
