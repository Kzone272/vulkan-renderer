#pragma once

#include "biped-rig.h"
#include "glm-include.h"
#include "input.h"
#include "object.h"
#include "second-order.h"

struct MoveOptions {
  float max_speed = 200;
  float adjust_time = 500;  // milliseconds
  float max_rot_speed = 270;
  float max_leg_pct = 0.95;
  float stance_w_pct = 0.5;
  float step_height = 5;
  float step_offset = 0;
  float bounce = 2;
  float hip_sway = 6;
  float hip_spin = 8;
  float shoulder_spin = 6;
  float arm_span_pct = 0.1;
  float hand_height_pct = 0.95;
  float hands_forward = 0;
};

struct SkellySizes {
  // Configurable
  float height = 185;
  float bone_w = 6;
  float leg = 100;  // floor to hip
  float femur_pct = 0.55;
  float pelvis_w = 30;
  float shoulders_w = 40;
  float arm = 70;
  float bicep_pct = 0.5;
  // Constants
  vec3 ankle = vec3(0, 10, -18);  // from toe to ankle
  vec3 toe = -ankle;              // from ankle to toe
  float pelvis_h = 15;            // height above hip
  float head_h = 25;              // length between shoulders and head
  float neck = 10;                // length between shoulders and head
  float foot_l = 25;              // Toe to heel
  float hand_l = 10;
  // Driven by params above.
  float pelvis_y;
  float shoulders_y;
  float ankle_d;  // Distance from hip to ankle
  float femur;
  float shin;
  float wrist_d;  // Distance from shoulder to wrist
  float bicep;
  float forearm;
  vec3 sho_pos;  // left shoulder
  vec3 hip_pos;  // left hip
};

struct MoveMods {
  float hand_blend = 0;
  float crouch_pct = 1;
  bool plant_feet = true;
  float lean = 10;
  vec3 lean_params = {10, 5, 0};
};

struct FootMeta {
  Object* foot;
  Object* toe;
  bool planted = false;
  bool is_left = false;
  bool just_lifted = false;
  vec3 world_target;
  vec3 start_pos;
  vec3 contact;
  vec3 liftoff;
  float step_offset = 0;
  float step_dur = 1;
  float toe_angle = 0;  // Angle relative to floor
  vec3 toe_pos;
  vec3 toe_dir = {0, 0, 1};
  vec3 toe_dir_start = {0, 0, 1};
};

template <class T>
struct Movement {
  void start(float cycle_dur);

  // Definition
  float offset = 0;
  float dur = 1;
  bool loop = false;
  // State
  Spline<T> spline;
  std::optional<Animation<T>> anim;
};

struct Cycle {
  Movement<vec3> lstep;
  Movement<vec3> rstep;
  Movement<vec3> lslide;
  Movement<vec3> rslide;
  Movement<float> lheel;
  Movement<float> rheel;
  Movement<float> bounce;
  Movement<float> sway;
  Movement<float> spin;
  Movement<float> heels_add;  // Mostly for debugging heel angle
  Movement<float> shoulders;
  Movement<vec3> larm;
  Movement<vec3> rarm;
};

class WalkCycle {
 public:
  WalkCycle() = default;
  WalkCycle(BipedRig& rig, const MoveOptions& move, const SkellySizes& sizes);

  void updateCycle(float cycle_dur, float target_speed);
  const Pose& getPose(float cycle_t);
  float getCycleDur() {
    return cycle_dur_;
  }
  float getTargetSpeed() {
    return target_speed_;
  }
  Cycle& getCycle() {
    return cycle_;
  }
  FootMeta& getLfoot() {
    return lfoot_m_;
  }
  FootMeta& getRfoot() {
    return rfoot_m_;
  }

 private:
  void updateStep(FootMeta& foot_m);
  void updateCog();
  void updatePelvis();
  void updateFeet();
  void updateToeAngle(FootMeta& foot);
  void updateToe(FootMeta& foot_m);
  void updateAnkle(const vec3& hip_pos, FootMeta& foot_m);
  void initFoot(FootMeta& foot_m, vec3 toe_pos);
  void plantFoot(FootMeta& foot_m);
  void updateShoulders();
  void updateHands();

  Cycle cycle_ = {
      // Offsets should be in [0, 1)
      // Durations should be in [0, 1]
      .lstep = {.offset = 0.05, .dur = 0.45},  // foot contact at 0.5
      .rstep = {.offset = 0.55, .dur = 0.45},  // foot contact at 0
      .lslide = {},  // Offset and dur set based on step movements.
      .rslide = {},
      .lheel = {.offset = 0.05, .dur = 0.45},
      .rheel = {.offset = 0.55, .dur = 0.45},
      .bounce = {.offset = 0, .dur = 0.5, .loop = true},      // pelvis up/down
      .sway = {.offset = 0, .dur = 1, .loop = true},          // z
      .spin = {.offset = 0, .dur = 1, .loop = true},          // y
      .heels_add = {.offset = 0.25, .dur = 1, .loop = true},  // x
      .shoulders = {.offset = 0, .dur = 1, .loop = true},     // y
      .larm = {.offset = 0.5, .dur = 1, .loop = true},
      .rarm = {.offset = 0, .dur = 1, .loop = true},
  };
  Pose pose_ = {};
  float cycle_dur_ = 0;
  float target_speed_ = 0;
  float cycle_t_ = 0;
  float prev_cycle_t_ = 0;

  // TODO: Delete these maybe
  Object* root_ = nullptr;
  const MoveOptions* move_ = nullptr;
  const SkellySizes* sizes_ = nullptr;

  FootMeta lfoot_m_ = {.is_left = true};
  FootMeta rfoot_m_ = {};
};

class Duration {
 public:
  Duration() = default;
  Duration(float dur_s) : dur_s_(dur_s) {
  }
  void update(float delta_s);
  float t() const {
    return t_;
  }

 private:
  float dur_s_ = 0;
  float t_ = 0;
};

class Skelly {
 public:
  Skelly();

  void makeBones();
  void handleInput(const InputState& input);
  void update(float delta_s);

  vec3 getPos();
  Object* getObj();
  float getPelvisHeight();
  void UpdateImgui();

 private:
  struct UiState {
    int move_preset = 0;
    int size_preset = 0;
  } ui_;

  void updateSpeed(float delta_s);

  void updateCycle(float delta_s);
  Pose getCyclePose(WalkCycle& cycle, float delta_s);
  void modifyPose(WalkCycle& cycle, Pose& pose, float delta_s);
  void updateLean(Pose& add_pose, float delta_s);
  void updateHandPose(Pose& pose);
  void plantFoot(Pose& pose, FootMeta& foot_m);
  void offsetFoot(Pose& add_pose, FootMeta& foot_m);

  void cycleUi(Cycle& cycle);
  void movementUi(const std::string& label, auto& move);

  MoveOptions options_;
  SkellySizes sizes_;
  MoveMods mods_;

  Object root_ = {};
  BipedSkeleton skeleton_ = {};
  Pose pose_ = {};
  Pose hand_pose_ = {};
  BipedRig rig_ = {};

  float cycle_t_ = 0;
  float prev_cycle_t_ = 0;
  float cycle_dur_ = 1000;

  std::vector<WalkCycle> move_cycles_;
  std::optional<Duration> move_transition_;

  WalkCycle idle_;
  const MoveOptions idle_move_ = {.step_offset = 15};

  Movement<vec3> lstep_offset_;
  Movement<vec3> rstep_offset_;

  vec2 input_dir_{0};
  std::optional<Animation<vec3>> vel_curve_;
  Duration vel_dur_;
  vec3 vel_{0};
  float target_speed_ = 0;
  bool target_speed_changed_ = false;

  std::unique_ptr<SecondOrder<vec3>> lean_so_;
};
