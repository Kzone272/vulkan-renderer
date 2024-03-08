#pragma once

#include "glm-include.h"
#include "input.h"
#include "object.h"
#include "second-order.h"
#include "time-include.h"

struct MoveOptions {
  float max_speed = 200;
  float adjust_time = 500;  // milliseconds
  float max_rot_speed = 270;
  float crouch_pct = 0.95;
  float stance_w = 15;
  float foot_dist = 5;
  float step_height = 5;
  float lean = 20;
  float bounce = 2;
  float hip_sway = 6;
  float hip_spin = 8;
  float heel_lift_pct = 0.75;
  float heel_shift = 0;
  float shoulder_spin = 6;
  float arm_span_pct = 0.1;
  float hand_height_pct = 0.95;
  float hands_forward = 0;
  // The option has very bad step placement when turning. Consider deleting.
  bool animate_in_world = false;
  vec3 lean_params = {10, 5, 0};
};

struct SkellySizes {
  // Configurable
  float height = 185;
  float bone_w = 6;
  float leg = 100;  // floor to hip
  float femur_pct = 0.5;
  float pelvis_w = 30;
  float shoulders_w = 40;
  float arm = 70;
  float bicep_pct = 0.5;
  // Constants
  vec3 ankle = vec3(0, 10, -18);  // from foot to ankle
  float pelvis_h = 15;            // height above hip
  float head_h = 25;              // length between shoulders and head
  float neck = 10;                // length between shoulders and head
  float foot_l = 25;              // Toe to heel
  float hand_l = 10;
  // Driven by params above.
  float pelvis_y;
  float shoulders_y;
  float femur;
  float shin;
  float wrist_d;  // Distance from shoulder to wrist
  float bicep;
  float forearm;
};

class Skelly {
 public:
  Skelly();

  void makeBones();
  void handleInput(const InputState& input, Time now);
  void update(Time now, float delta_s);

  vec3 getPos();
  Object* getObj();
  float getPelvisHeight();
  void UpdateImgui();

 private:
  struct UiState {
    int move_preset = 0;
    int size_preset = 0;
  } ui_;

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

  struct IkControls {
    CenterOfGravity cog;
    Pelvis pelvis;
    Foot lfoot;
    Foot rfoot;
    Torso torso;
    Hand lhand;
    Hand rhand;
  } ik_ = {};

  template <class T>
  struct Movement {
    // Definition
    float offset;
    float dur;
    bool loop = false;
    // State
    bool should_start = false;
    Spline<T> spline;
    std::optional<Animation<T>> anim;
  };
  struct Cycle {
    Movement<vec3> lstep;
    Movement<vec3> rstep;
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
  Cycle walk_{
      // Offsets should be in [0, 1)
      // Durations should be in [0, 1]
      .lstep = {.offset = 0.05, .dur = 0.45},  // foot contact at 0.5
      .rstep = {.offset = 0.55, .dur = 0.45},  // foot contact at 0
      .lheel = {.offset = 0.85, .dur = 0.7},
      .rheel = {.offset = 0.35, .dur = 0.7},
      .bounce = {.offset = 0, .dur = 0.5, .loop = true},      // pelvis up/down
      .sway = {.offset = 0, .dur = 1, .loop = true},          // z
      .spin = {.offset = 0, .dur = 1, .loop = true},          // y
      .heels_add = {.offset = 0.25, .dur = 1, .loop = true},  // x
      .shoulders = {.offset = 0, .dur = 1, .loop = true},     // y
      .larm = {.offset = 0.5, .dur = 1, .loop = true},
      .rarm = {.offset = 0, .dur = 1, .loop = true},
  };

  void initFoot(Foot& foot, vec3 offset);
  void updateSpeed(Time now, float delta_s);

  bool inCycle(const auto& move, float t);
  void checkMove(auto& move, Time now, bool moves_changed, float prev_t);

  void updateCycle(Time now, float delta_s);
  void updateMovements();

  void startMovement(auto& move, Time now);
  Time getMoveStart(auto& move, Time now);
  vec3 sampleMovement(Movement<vec3>& move, Time now);
  float sampleMovement(Movement<float>& move, Time now);

  void updateCog(Time now, float delta_s);
  void updatePelvis(Time now);
  void updateFeet(Time now);
  void updateHeel(Time now, Foot& foot, Movement<float>& move);
  void updateFoot(Foot& foot, Time now, Movement<vec3>& move);
  void swingFoot(Foot& foot, Time now, Movement<vec3>& move);
  void plantFoot(Foot& foot);
  void updateLegs();
  void updateLeg(Object& femur, Object& shin, Object& foot, Foot& ik_foot);
  void updateTwoBoneIk(
      Object& bone1, float b1_l, Object& bone2, float b2_l, vec3 b1_pos,
      vec3 target, vec3 main_axis, vec3 rot_axis);
  void updateShoulders(Time now);
  void updateHands(Time now);
  void updateArms();

  // Returns pair of angles for bone1 and bone2.
  std::pair<float, float> solveIk(float bone1, float bone2, float target);

  void cycleUi(Cycle& cycle);
  void movementUi(const std::string& label, auto& move);

  MoveOptions options_;
  SkellySizes sizes_;

  Object root_;
  Object cog_;
  Object* pelvis_;
  Object* torso_;
  Object lbicep_;
  Object lforearm_;
  Object lhand_;
  Object rbicep_;
  Object rforearm_;
  Object rhand_;
  Object* head_;
  Object* lfemur_;
  Object* lshin_;
  Object* lfoot_;
  Object* rfemur_;
  Object* rshin_;
  Object* rfoot_;

  float cycle_t_ = 0;
  float cycle_dur_ = 1000;

  vec2 input_dir_{0};
  std::optional<Animation<vec3>> vel_curve_;
  vec3 vel_{0};
  float target_speed_ = 0;
  bool target_speed_changed_ = false;

  std::unique_ptr<SecondOrder<vec3>> lean_so_;
};
