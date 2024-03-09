#pragma once

#include "object.h"
#include "glm-include.h"

struct CenterOfGravity {
  vec3 pos;
};

struct Foot {
  Object* obj;
  bool planted = false;
  bool in_swing = false;
  vec3 world_target;
  glm::quat world_rot;
  vec3 offset;
  float angle = 0;  // Angle relative to floor
  vec3 knee = {0, 0, 1};
};

struct Pelvis {
  float tilt;  // x
  float sway;  // z
  float spin;  // y
};

struct Torso {
  vec3 pos;
  float tilt;  // x
  float sway;  // z
  float spin;  // y
};

struct Hand {
  Object* obj;
  vec3 elbow = {0, 0, -1};  // dir
};

struct SkellySizes;
struct MoveOptions;

struct BipedRig {
  BipedRig() = default;
  // Not copyable
  BipedRig(const BipedRig& other) = delete;
  BipedRig& operator=(const BipedRig& other) = delete;

  void makeBones(const SkellySizes& sizes, const MoveOptions& move_);
  void plantFoot(Foot& foot, const MoveOptions& move);

  Object root_;
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

  // Controls "_c"
  CenterOfGravity cog_c_;
  Pelvis pelvis_c_;
  Foot lfoot_c_;
  Foot rfoot_c_;
  Torso torso_c_;
  Hand lhand_c_;
  Hand rhand_c_;
};
