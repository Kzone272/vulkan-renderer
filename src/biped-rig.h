#pragma once

#include "glm-include.h"
#include "object.h"
#include "pose.h"

struct SkellySizes;
struct BipedRig;

struct BipedSkeleton {
  BipedSkeleton() = default;
  // Not copyable
  BipedSkeleton(const BipedSkeleton& other) = delete;
  BipedSkeleton& operator=(const BipedSkeleton& other) = delete;

  void makeBones(const SkellySizes& sizes, Object* root);

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
  Object* rfemur_;
  Object* rshin_;
  Object* rfoot_;

  float femur_l_;
  float shin_l_;
  float bicep_l_;
  float forearm_l_;
  vec3 toe_pos_;
  vec3 ankle_pos_;
};

struct IkChain {
  IkChain() = default;
  IkChain(
      Object* start, Object* target, Object* b1, Object* b2, float b1_l,
      float b2_l, vec3 rot_axis);
  void solve();
  // Positions in start's parent's space
  vec3 startPos();
  vec3 targetPos();

  Object* start;
  Object* target;
  Object* lca;  // LCA of start and target
  // TODO: Support chain of bones.
  Object* b1;
  Object* b2;
  float b1_l;
  float b2_l;
  vec3 rot_axis;
  // Computed:
  // The normalized vector pointing from the root to the target in the root's
  // parent's space, when bones are in zero-state.
  vec3 point_zero;
};

struct BipedRig {
  BipedRig() = default;
  void makeRig(const BipedSkeleton& skeleton, Object* root);
  Pose getZeroPose() {
    return zero_p_;
  }
  void updateSkeleton(BipedSkeleton& skeleton);
  void solveIk();
  void applyPose(const Pose& pose);
  Object* getBone(BoneId bone) const;

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
  Object* ltoe_;
  Object* lankle_;
  Object* rtoe_;
  Object* rankle_;

  IkChain larm_;
  IkChain rarm_;
  IkChain lleg_;
  IkChain rleg_;
  IkChain spine_;
  IkChain cerv_;  // Cervical spine (neck to head)

  Pose zero_p_;
};
