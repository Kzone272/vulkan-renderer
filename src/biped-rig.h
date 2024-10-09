#pragma once

#include "glm-include.h"
#include "object.h"
#include "pose.h"
#include "render-objects.h"

struct SkellySizes;
struct BipedRig;

struct BipedSkeleton {
  BipedSkeleton() = default;
  // Not copyable
  BipedSkeleton(const BipedSkeleton& other) = delete;
  BipedSkeleton& operator=(const BipedSkeleton& other) = delete;

  void makeBones(const SkellySizes& sizes, Object* root);
  void setMaterial(MaterialId material);

  Object* root_;
  Object* cog_;
  Object* pelvis_;
  Object* torso_;
  Object* lbicep_;
  Object* lforearm_;
  Object* lhand_;
  Object* rbicep_;
  Object* rforearm_;
  Object* rhand_;
  Object* head_;
  Object* lfemur_;
  Object* lshin_;
  Object* lfoot_;
  Object* ltoes_;
  Object* rfemur_;
  Object* rshin_;
  Object* rfoot_;
  Object* rtoes_;

  float femur_l_;
  float shin_l_;
  float bicep_l_;
  float forearm_l_;
};

struct IkChain {
  IkChain() = default;
  IkChain(
      Object* start, Object* target, Object* pole, Object* b1, Object* b2,
      float b1_l, float b2_l);
  void solve();
  // Positions in start's parent's space
  vec3 startPos();
  vec3 targetPos();

  Object* start;
  Object* target;
  Object* pole;  // TODO: Currently treated as vector, not a position.
  Object* lca;   // LCA of start and target
  // TODO: Support chain of bones.
  Object* b1;
  Object* b2;
  float b1_l;
  float b2_l;
  vec3 dir_zero;
  // Computed:
  // The normalized vector pointing from the root to the target in the root's
  // parent's space, when bones are in zero-state.
  vec3 point_zero;
  vec3 rot_axis;
};

struct BipedRig {
  enum Id : size_t {
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

  BipedRig() = default;
  void makeRig(const BipedSkeleton& skeleton, Object* root);
  void setMaterial(MaterialId material);
  Pose getZeroPose() {
    return zero_pose_;
  }
  void updateSkeleton(BipedSkeleton& skeleton);
  void solveIk();
  void applyPose(const Pose& pose);
  Skeleton* skeleton() {
    return &skl_;
  };

  void addBone(Id bone, Id parent, vec3 pos);

  void getSceneObjects(
      const mat4& parent, std::vector<SceneObject>& objs,
      const std::set<ModelId>& hidden);

  Skeleton skl_ = {Id::COUNT};
  MaterialId mat_ = kMaterialIdNone;

  Object* root_;
  Object* cog_;
  Object* neck_;
  Object* head_;
  Object* lsho_;  // (sho)ulder
  Object* rsho_;
  Object* lhand_;
  Object* rhand_;
  Object* pelvis_;
  Object* lhip_;
  Object* rhip_;
  Object* lankle_;
  Object* lball_;  // ball of foot
  Object* rankle_;
  Object* rball_;
  Object* lelbow_;
  Object* relbow_;
  Object* lknee_;
  Object* rknee_;

  IkChain larm_;
  IkChain rarm_;
  IkChain lleg_;
  IkChain rleg_;
  IkChain spine_;
  IkChain cerv_;  // Cervical spine (neck to head)

  Pose zero_pose_;
  Pose curr_pose_;
};
