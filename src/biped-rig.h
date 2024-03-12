#pragma once

#include "glm-include.h"
#include "object.h"

struct SkellySizes;
struct BipedRig;

struct BipedSkeleton {
  BipedSkeleton() = default;
  // Not copyable
  BipedSkeleton(const BipedSkeleton& other) = delete;
  BipedSkeleton& operator=(const BipedSkeleton& other) = delete;

  void makeBones(const SkellySizes& sizes, Object* root);
  void setFromRig(const BipedRig& rig);

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
};

class IkChain {
  Object* root;
  Object* target;
  // TODO: Support chain of bones.
  Object* b1;
  Object* b2;
  vec3 dir;
};

struct FootMeta {
  Object* foot;
  Object* toe;
  bool planted = false;
  bool in_swing = false;
  bool is_left = false;
  vec3 world_target;
  vec3 start_pos;
  float toe_angle = 0;  // Angle relative to floor
  vec3 toe_pos;
  vec3 toe_dir = {0, 0, 1};
  vec3 toe_dir_start = {0, 0, 1};
};

struct BipedRig {
  BipedRig() = default;
  void makeRig(const BipedSkeleton& skeleton, Object* root);
  void plantFoot(FootMeta& foot_m);
  void initFoot(FootMeta& foot_m);

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
  Object* lfoot_;
  Object* rfoot_;
  Object* ltoe_;
  Object* rtoe_;

  IkChain larm_;
  IkChain rarm_;
  IkChain lleg_;
  IkChain rleg_;
  IkChain spine_;
  IkChain cerv_;  // Cervical spine (neck to head)

  FootMeta lfoot_m_{.is_left = true};
  FootMeta rfoot_m_;
};
