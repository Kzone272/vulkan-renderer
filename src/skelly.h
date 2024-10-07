#pragma once

#include "biped-rig.h"
#include "glm-include.h"
#include "input.h"
#include "object.h"
#include "render-objects.h"
#include "second-order.h"

struct MoveOptions {
  float max_speed = 200;
  float adjust_time = 500;  // milliseconds
  float max_rot_speed = 270;
  float blend_time = 0.5;  // seconds
};

struct WalkOptions {
  float speed = 200;
  float stance_w_pct = 0.5;
  float step_height = 10;
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
  float foot_l = 25;                      // Toe to heel
  float toes_l = 8;                       // Toe to ball
  vec3 ankle = vec3(0, 5, -18);           // from toe to ankle
  vec3 toe = -ankle;                      // from ankle to toe
  vec3 ball = toe + vec3(0, 0, -toes_l);  // from ankle to ball of foot
  vec3 heel = toe + vec3(0, 0, -foot_l);  // from ankle to heel
  float pelvis_h = 15;                    // height above hip
  float head_h = 25;                      // length between shoulders and head
  float neck = 10;                        // length between shoulders and head
  float hand_l = 10;
  float min_knee_angle = glm::radians(5.f);
  // Driven by params above.
  float pelvis_y;  // Pelvis height in zero-position
  float shoulders_y;
  float ankle_d;  // Distance from hip to ankle
  float femur;
  float shin;
  float max_leg;
  float normal_pelvis_y;  // Pelvis height when standing normally
  float wrist_d;          // Distance from shoulder to wrist
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
  bool just_landed = false;
  vec3 world_target;
  vec3 start_pos;
  vec3 contact;
  vec3 liftoff;
  float step_offset = 0;
  float step_dur = 1;
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
  Movement<float> ltoe;
  Movement<float> rtoe;
  Movement<float> lball;
  Movement<float> rball;
  Movement<float> ldrop;
  Movement<float> rdrop;
  Movement<float> bounce;
  Movement<float> sway;
  Movement<float> spin;
  Movement<float> shoulders;
  Movement<vec3> larm;
  Movement<vec3> rarm;
};

class WalkCycle {
 public:
  WalkCycle() = default;
  WalkCycle(
      BipedRig& rig, const WalkOptions& walk, const SkellySizes& sizes,
      const Cycle& cycle, float cycle_dur);

  void updateCycle(const WalkOptions& walk, const Cycle& cycle);
  const Pose& getPose(float cycle_t);
  float getCycleDur() {
    return cycle_dur_;
  }
  float getTargetSpeed() {
    return walk_.speed;
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
  void updateToe(FootMeta& foot_m);
  void updateAnkle(FootMeta& foot_m);
  void updateShoulders();
  void updateHands();

  Cycle cycle_ = {};
  Pose pose_ = {};
  float cycle_dur_ = 0;
  float cycle_t_ = 0;
  float prev_cycle_t_ = 0;

  // TODO: Delete these maybe
  Object* root_ = nullptr;
  WalkOptions walk_ = {};
  const SkellySizes* sizes_ = nullptr;

  FootMeta lfoot_m_ = {.is_left = true};
  FootMeta rfoot_m_ = {};
};

class WalkPoser {
 public:
  WalkPoser() = default;
  WalkPoser(WalkCycle walk, const MoveMods& mods, Object* root);
  Pose getPose(float cycle_t, float delta_s);
  float getCycleDur() {
    return walk_.getCycleDur();
  }
  void updateCycle(const WalkOptions& walk, const Cycle& cycle) {
    walk_.updateCycle(walk, cycle);
  }

 private:
  void plantFoot(FootMeta& foot_m);
  void offsetFoot(float cycle_t, FootMeta& foot_m);
  void setWorld(FootMeta& foot_m);

  WalkCycle walk_ = {};
  Movement<vec3> lstep_offset_;
  Movement<vec3> rstep_offset_;
  Pose add_pose_ = {PoseType::Additive};

  const MoveMods* mods_ = nullptr;
  Object* root_ = nullptr;
  bool world_set_ = false;
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
  void setMaterials(MaterialId bone_mat, MaterialId control_mat);
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
  void updateMoveCycle();
  void updateLean(float delta_s);
  void updateHandPose(Pose& pose);

  bool cycleUi(Cycle& cycle);
  bool movementUi(const std::string& label, auto& move);

  MoveOptions move_ = {};
  WalkOptions walk_ = {};
  WalkOptions run_ = {
      .speed = 600,
      .step_height = 20,
      .bounce = 10,
      .hip_spin = 20,
      .shoulder_spin = 10,
      .hand_height_pct = 0.4,
  };
  SkellySizes sizes_ = {};
  MoveMods mods_ = {};

  Object root_ = {};
  BipedSkeleton skeleton_ = {};
  Pose pose_ = {};
  Pose lean_pose_ = {PoseType::Additive};
  Pose hand_pose_ = {};
  BipedRig rig_ = {};

  float cycle_t_ = 0;
  float prev_cycle_t_ = 0;
  float cycle_dur_ = 1000;

  const Cycle default_walk_ = {
      // Offsets should be in [0, 1)
      // Durations should be in [0, 1]
      .lstep = {.offset = 0.05, .dur = 0.45},  // foot contact at 0.5
      .rstep = {.offset = 0.55, .dur = 0.45},  // foot contact at 0
      .lslide = {},  // Offset and dur set based on step movements.
      .rslide = {},
      .lheel = {.offset = 0.05, .dur = 0.45},
      .rheel = {.offset = 0.55, .dur = 0.45},
      .bounce = {.offset = 0, .dur = 0.5, .loop = true},   // pelvis up/down
      .sway = {.offset = 0, .dur = 1, .loop = true},       // z
      .spin = {.offset = 0, .dur = 1, .loop = true},       // y
      .shoulders = {.offset = 0, .dur = 1, .loop = true},  // y
      .larm = {.offset = 0.5, .dur = 1, .loop = true},
      .rarm = {.offset = 0, .dur = 1, .loop = true},
  };
  Cycle walk_cycle_ = default_walk_;
  Cycle run_cycle_ = {
      .lstep = {.offset = 0.85, .dur = 0.8},
      .rstep = {.offset = 0.35, .dur = 0.8},
      .lslide = {},
      .rslide = {},
      .lheel = {.offset = 0.85, .dur = 0.8},
      .rheel = {.offset = 0.35, .dur = 0.8},
      .bounce = {.offset = 0, .dur = 0.5, .loop = true},
      .sway = {.offset = 0, .dur = 1, .loop = true},
      .spin = {.offset = 0, .dur = 1, .loop = true},
      .shoulders = {.offset = 0, .dur = 1, .loop = true},
      .larm = {.offset = 0.5, .dur = 1, .loop = true},
      .rarm = {.offset = 0, .dur = 1, .loop = true},
  };

  std::vector<WalkPoser> move_cycles_;
  std::optional<Duration> move_transition_;

  WalkCycle idle_;
  const WalkOptions idle_walk_ = {.speed = 0, .step_offset = 15};

  vec2 input_dir_{0};
  std::optional<Animation<vec3>> vel_curve_;
  Duration vel_dur_;
  vec3 vel_{0};
  float target_speed_ = 0;
  bool target_speed_changed_ = false;
  bool move_cycle_changed_ = false;

  std::unique_ptr<SecondOrder<vec3>> lean_so_;
};
