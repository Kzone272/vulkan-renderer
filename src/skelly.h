#pragma once

#include "animation.h"
#include "glm-include.h"
#include "input.h"
#include "maths.h"
#include "object.h"
#include "strings.h"

struct MoveOptions {
  float max_speed = 200;
  float adjust_time = 500;
  float crouch_pct = 0.95;
  float stance_w = 15;
  float foot_dist = 5;
  float step_height = 5;
  float max_rot_speed = 270;
  float lean = 0.05;
  float bounce = 3;
  float hip_sway = 6;
  float hip_spin = 12;
  float heel_lift_pct = 0.75;
  float heel_shift = 0;
  // The option has very bad step placement when turning. Consider deleting.
  bool animate_in_world = false;
};

struct SkellySizes {
  // Configurable
  float height = 185;
  float bone_w = 6;
  float leg = 100;  // floor to hip
  float femur_pct = 0.56;
  float pelvis_w = 25;
  float shoulders_w = 50;
  // Constants
  vec3 ankle = vec3(0, 10, -18);  // from foot to ankle
  float pelvis_h = 15;            // height above hip
  float head_h = 25;              // length between shoulders and head
  float neck = 5;                 // length between shoulders and head
  float foot_l = 25;              // Toe to heel
  // Driven by params above.
  float pelvis_y;
  float shoulders_y;
  float femur;
  float shin;
};

class Skelly {
 public:
  Skelly() {
    root_.setPos(vec3(200, 0, 200));
    makeBones();
    updateCurves();
  }

  void makeBones() {
    sizes_.pelvis_y = sizes_.leg + sizes_.pelvis_h;
    sizes_.shoulders_y =
        sizes_.height - sizes_.pelvis_y - sizes_.neck - sizes_.head_h;
    float leg_l = sizes_.leg - sizes_.ankle.y;
    sizes_.femur = sizes_.femur_pct * leg_l;
    sizes_.shin = leg_l - sizes_.femur;

    root_.clearChildren();

    cog_ = Object(ModelId::Control, glm::scale(vec3(5)));
    root_.addChild(&cog_);
    cog_.setPos(vec3(0, sizes_.pelvis_y, 0));

    mat4 pelvis_t = glm::scale(vec3(sizes_.pelvis_w, -sizes_.pelvis_h, 20));
    pelvis_ = cog_.addChild(std::make_unique<Object>(ModelId::Bone, pelvis_t));
    pelvis_->setPos(vec3(0, 0, 0));

    mat4 torso_t = glm::scale(vec3(sizes_.shoulders_w, -20, 25));
    torso_ = cog_.addChild(std::make_unique<Object>(ModelId::Bone, torso_t));
    torso_->setPos(vec3(0, sizes_.shoulders_y, 0));

    mat4 head_t = glm::scale(vec3(20, sizes_.head_h, 25));
    head_ = torso_->addChild(std::make_unique<Object>(ModelId::Bone, head_t));
    head_->setPos(vec3(0, sizes_.neck, 5));

    mat4 femur_t =
        glm::scale(vec3(sizes_.bone_w, -sizes_.femur, sizes_.bone_w));
    lfemur_ =
        pelvis_->addChild(std::make_unique<Object>(ModelId::Bone, femur_t));
    vec3 femur_pos = vec3(-(sizes_.pelvis_w / 2 + 3), -sizes_.pelvis_h, 0);
    lfemur_->setPos(femur_pos);

    mat4 shin_t = glm::scale(vec3(sizes_.bone_w, -sizes_.shin, sizes_.bone_w));
    lshin_ = lfemur_->addChild(std::make_unique<Object>(ModelId::Bone, shin_t));
    vec3 shin_pos = vec3(0, -sizes_.femur, 0);
    lshin_->setPos(shin_pos);

    mat4 foot_t = glm::translate(-sizes_.ankle) *
                  glm::scale(vec3(10, 8, sizes_.foot_l)) *
                  glm::translate(vec3(0, 0, -0.5));
    lfoot_ = lshin_->addChild(std::make_unique<Object>(ModelId::Bone, foot_t));
    vec3 foot_pos = vec3(0, -sizes_.shin, 0);
    lfoot_->setPos(foot_pos);

    // Add opposite limbs with flipped positions.
    mat3 flip = mat3(glm::scale(vec3(-1, 1, 1)));

    rfemur_ =
        pelvis_->addChild(std::make_unique<Object>(ModelId::Bone, femur_t));
    rfemur_->setPos(flip * femur_pos);

    rshin_ = rfemur_->addChild(std::make_unique<Object>(ModelId::Bone, shin_t));
    rshin_->setPos(flip * shin_pos);

    rfoot_ = rshin_->addChild(std::make_unique<Object>(ModelId::Bone, foot_t));
    rfoot_->setPos(flip * foot_pos);

    makeIkControls();
  }

  void handleInput(const InputState& input, Time now) {
    if (input.kb.pressed.contains(' ')) {
      vec3 start = root_.getPos();

      float speedz = 300;
      float speedy = 600;
      if (input.kb.down.contains(Keys::Shift)) {
        speedz *= -1;
      }
      vec3 posb = start + vec3(0, speedy, speedz / 3);
      vec3 end = start + vec3(0, 0, speedz);
      vec3 posc = end + vec3(0, speedy, -speedz / 3);

      vec3 posd = end + vec3(0, speedy / 4, speedz / 10);
      vec3 end2 = end + vec3(0, 0, speedz / 3);
      vec3 pose = end2 + vec3(0, speedy / 4, -speedz / 10);

      auto spline = makeSpline<vec3>(
          SplineType::Bezier, {start, posb, posc, end, posd, pose, end2});
      // root_.addPosAnim(makeAnimation(spline, 800, now));
    }

    // Check if we should update velocity because max speed slider changed.
    // TODO: Use slider update directly.
    bool max_speed_changed =
        target_speed_ > 0.1f &&
        std::abs(target_speed_ - options_.max_speed) > 0.1f;

    target_speed_changed_ = false;
    if (glm::length(input.move.dir - input_dir_) > 0.1f || max_speed_changed) {
      input_dir_ = input.move.dir;
      vec3 target_vel =
          options_.max_speed * vec3(input_dir_.x, 0, input_dir_.y);

      float speed = glm::length(target_vel);
      if (std::abs(speed - target_speed_) > 0.1f) {
        target_speed_ = speed;
        target_speed_changed_ = true;
      }

      if (options_.adjust_time == 0) {
        vel_ = target_vel;
      } else {
        vec3 a = vel_;
        vec3 d = target_vel;
        vec3 b = glm::mix(a, d, 1.f / 3.f);
        vec3 c = d;
        auto spline = makeSpline<vec3>(SplineType::Bezier, {a, b, c, d});
        vel_curve_ = makeAnimation(spline, options_.adjust_time, now);
      }
    }
  }

  void update(Time now, float delta_s) {
    updateMove(now, delta_s);
    // Root animate used for awkward jump animation I should probably delete.
    root_.animate(now);
    updateCycle(now, delta_s);
    updateCog(now);
    updatePelvis(now);
    updateFeet(now);
    updateLegs();
  }

  vec3 getPos() {
    return root_.getPos();
  }

  Object* getObj() {
    return &root_;
  }

  MoveOptions* getMoveOptions() {
    return &options_;
  }
  SkellySizes* getSkellySizes() {
    return &sizes_;
  }

 private:
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
    vec3 pos;
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

  void makeIkControls() {
    mat3 flip = mat3(glm::scale(vec3(-1, 1, 1)));

    vec3 foot_offset = {-options_.stance_w / 2, 0, 12};
    initFoot(ik_.lfoot, foot_offset);
    initFoot(ik_.rfoot, flip * vec4(foot_offset, 1));
  }

  void initFoot(Foot& foot, vec3 offset) {
    mat4 control_t = glm::scale(vec3(5));
    foot.obj =
        root_.addChild(std::make_unique<Object>(ModelId::Control, control_t));
    foot.offset = offset;
    foot.obj->setPos(foot.offset);
    if (options_.animate_in_world) {
      foot.world_target = root_.toWorld() * vec4(foot.obj->getPos(), 1);
    }
    plantFoot(foot);
  }

  template <class T>
  struct Movement {
    float offset;
    float dur;
    bool loop = false;
    bool should_start = false;
    vec3 axis;  // used only for rotation
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
  };
  Cycle walk_{
      // Offsets should be in [0, 1)
      // Durations should be in [0, 1]
      .lstep = {.offset = 0.05, .dur = 0.45},  // foot contact at 0.5
      .rstep = {.offset = 0.55, .dur = 0.45},  // foot contact at 0
      .lheel = {.offset = 0.85, .dur = 0.7},
      .rheel = {.offset = 0.35, .dur = 0.7},
      .bounce = {.offset = 0, .dur = 0.5, .loop = true},  // pelvis up/down
      .sway = {.offset = 0, .dur = 1, .loop = true, .axis = {0, 0, -1}},
      .spin = {.offset = 0, .dur = 1, .loop = true, .axis = {0, 1, 0}},
      .heels_add = {.offset = 0.25, .dur = 1, .loop = true, .axis = {0, 1, 0}},
  };

  void updateMove(Time now, float delta_s) {
    vec3 curr_vel = vel_;

    if (vel_curve_) {
      vel_ = sampleAnimation(*vel_curve_, now);
      if (now > vel_curve_->to_time) {
        vel_curve_.reset();
      }
      // Average old and new velocity for this frame's update.
      curr_vel += vel_;
      curr_vel /= 2;
    }

    auto pos = getPos();
    pos += curr_vel * delta_s;
    root_.setPos(pos);

    if (glm::length(curr_vel) > 0.1) {
      float target_angle = glm::orientedAngle(
          vec3(0, 0, 1), glm::normalize(curr_vel), vec3(0, 1, 0));
      target_angle = fmodClamp(target_angle, glm::radians(360.f));

      float current = glm::angle(root_.getRot());
      float delta = angleDelta(current, target_angle);

      float angle = target_angle;
      float change = delta_s * glm::radians(options_.max_rot_speed);
      if (std::abs(delta) > change) {
        float dir = (delta > 0) ? 1 : -1;
        angle = current + dir * change;
      }
      angle = fmodClamp(angle, glm::radians(360.f));

      root_.setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
    }
  }

  template <class T>
  bool inCycle(const Movement<T>& move, float t) {
    if (t < move.offset) {
      t += 1;
      DASSERT(t >= move.offset);
    }
    float end = move.offset + move.dur;
    return t <= end;
  }

  template <class T>
  void checkStart(Movement<T>& move, float prev_t, float t) {
    move.should_start = !inCycle(move, prev_t) && inCycle(move, t);
  }

  void updateCycle(Time now, float delta_s) {
    float prev_t = cycle_t_;

    // TODO: This check probably needs word.
    // Maybe it should start once we take the first step, then stop once
    // velocity reaches zero?
    if ((false) && glm::length(vel_) < 0.1f) {
      cycle_t_ = 0;
      // TODO: This is smelly.
      walk_.lstep.should_start = false;
      walk_.rstep.should_start = false;
      walk_.lheel.should_start = false;
      walk_.rheel.should_start = false;
    } else {
      cycle_t_ = fmod(cycle_t_ + delta_s / (cycle_dur_ / 1000.f), 1.f);
      // Don't check start for looping animations.
      checkStart(walk_.lstep, prev_t, cycle_t_);
      checkStart(walk_.rstep, prev_t, cycle_t_);
      checkStart(walk_.lheel, prev_t, cycle_t_);
      checkStart(walk_.rheel, prev_t, cycle_t_);
    }

    if (!target_speed_changed_) {
      return;
    }

    // TODO: Clean up this math. step_dur should not be a factor.
    float speed = target_speed_;
    float step_l = std::min(sizes_.leg * 0.6f, speed / 3);
    float step_dur = 1.5f * (step_l / speed) * 1000.f;
    if (speed == 0) {
      step_dur = 500;
    }
    cycle_dur_ = step_dur / walk_.lstep.dur;
    updateCurves();
    // TODO: clean up movement starting.
    startMovement(walk_.heels_add, now);
  }

  void updateCurves() {
    float bounce = std::pow(cycle_dur_ / 1000, 2) * options_.bounce;
    walk_.bounce.spline = makeSpline<float>(
        SplineType::Hermite, {-bounce, 0, bounce, 0, -bounce, 0});

    float hip_sway = glm::radians(options_.hip_sway);
    walk_.sway.spline = makeSpline<float>(
        SplineType::Hermite, {-hip_sway, 0, hip_sway, 0, -hip_sway, 0});

    float hip_spin = glm::radians(options_.hip_spin);
    walk_.spin.spline = makeSpline<float>(
        SplineType::Hermite, {-hip_spin, 0, hip_spin, 0, -hip_spin, 0});

    std::vector<float> values = {0, 10, 50, 5, -15, 5, 0, 0};
    float speed_scale = std::min(target_speed_, 300.f) / 150.f;
    for (float& val : values) {
      val *= options_.heel_lift_pct * speed_scale;
    }
    walk_.lheel.spline =
        makeSpline<float>(SplineType::Hermite, std::move(values));
    walk_.rheel.spline = walk_.lheel.spline;

    float heel = options_.heel_shift;
    walk_.heels_add.spline =
        makeSpline<float>(SplineType::Hermite, {-heel, 0, heel, 0, -heel, 0});
  }

  template <class T>
  void startMovement(Movement<T>& move, Time now) {
    float dur = move.dur * cycle_dur_;
    Time start = getMoveStart(move, now);
    move.anim = makeAnimation(move.spline, dur, start, move.loop, move.axis);
  }

  template <class T>
  Time getMoveStart(const Movement<T>& move, Time now) {
    float t = cycle_t_;
    if (t < move.offset) {
      t += 1;
      DASSERT(t >= move.offset);
    }
    float pos_in_anim = t - move.offset;
    return addMs(now, -pos_in_anim * cycle_dur_);
  }

  void updateCog(Time now) {
    vec3 pos = vec3(0, options_.crouch_pct * sizes_.pelvis_y, 0);

    if (target_speed_changed_) {
      startMovement(walk_.bounce, now);
    }
    if (walk_.bounce.anim) {
      pos.y += sampleAnimation(*walk_.bounce.anim, now);
    }

    vec2 lean = options_.lean * vel_.xz();
    vec3 offset = root_.toLocal() * vec4(vec3(lean.x, 0, lean.y), 0);
    pos += offset;
    cog_.setPos(pos);
  }

  void updatePelvis(Time now) {
    if (target_speed_changed_) {
      pelvis_->clearRotAnims();

      startMovement(walk_.sway, now);
      startMovement(walk_.spin, now);
    }
    if (walk_.sway.anim) {
      ik_.pelvis.sway = sampleAnimation(*walk_.sway.anim, now);
    }
    if (walk_.spin.anim) {
      ik_.pelvis.spin = sampleAnimation(*walk_.spin.anim, now);
    }

    glm::quat rot = glm::angleAxis(ik_.pelvis.spin, vec3(0, 1, 0)) *
                    glm::angleAxis(ik_.pelvis.sway, vec3(0, 0, -1));
    pelvis_->setRot(rot);
  }

  void updateFeet(Time now) {
    updateHeel(now, ik_.lfoot, walk_.lheel);
    updateHeel(now, ik_.rfoot, walk_.rheel);
    updateFoot(ik_.rfoot, now, walk_.rstep);
    updateFoot(ik_.lfoot, now, walk_.lstep);
  }

  void updateHeel(Time now, Foot& foot, Movement<float>& heel) {
    if (heel.should_start) {
      startMovement(heel, now);
    }

    if (heel.anim) {
      foot.angle = sampleAnimation(*heel.anim, now);
      if (now > heel.anim->to_time) {
        heel.anim.reset();
        foot.angle = 0;
      }
    } else {
      foot.angle = 0;
    }

    if (walk_.heels_add.anim) {
      foot.angle += sampleAnimation(*walk_.heels_add.anim, now);
    }
  }

  void updateFoot(Foot& foot, Time now, Movement<vec3>& move) {
    {
      // This code isn't used anymore, but probably should be used to determine
      // when walking starts, or if we should move feet back when stopped.
      bool supported = !ik_.lfoot.in_swing && !ik_.rfoot.in_swing;
      vec3 pos = foot.obj->getPos();
      float target_dist = glm::length(pos - foot.offset);
      bool should_step = supported && target_dist > options_.foot_dist;
    }

    if (move.should_start) {
      swingFoot(foot, now, move);
    }

    if (foot.planted) {
      mat4 to_local = root_.toLocal();
      foot.obj->setPos(to_local * vec4(foot.world_target, 1));
    }

    if (move.anim) {
      if (now > move.anim->to_time) {
        vec3 plant_pos = sampleAnimation(*move.anim, move.anim->to_time);
        if (options_.animate_in_world) {
          plant_pos = root_.toLocal() * vec4(plant_pos, 1);
        }
        foot.obj->setPos(plant_pos);
        move.anim.reset();
        plantFoot(foot);
      } else {
        vec3 swing_pos = sampleAnimation(*move.anim, now);
        if (options_.animate_in_world) {
          swing_pos = root_.toLocal() * vec4(swing_pos, 1);
        }
        foot.obj->setPos(swing_pos);
      }
    }
  }

  void swingFoot(Foot& foot, Time now, Movement<vec3>& move) {
    foot.in_swing = true;
    foot.planted = false;

    vec3 start = foot.obj->getPos();
    mat4 to_world = root_.toWorld();
    if (options_.animate_in_world) {
      start = to_world * vec4(start, 1);
    }

    vec3 end;
    if (options_.animate_in_world) {
      vec3 offset = to_world * vec4(foot.offset, 0);
      vec3 target_v = vel_;
      if (vel_curve_) {
        target_v += sampleAnimation(*vel_curve_, addMs(now, move.dur));
        target_v /= 2;
      }
      vec3 step{0};
      if (glm::length(target_v) > 0.1) {
        float step_l = std::min(sizes_.leg * 1.4f, glm::length(target_v));
        step = step_l * glm::normalize(target_v);
      }
      vec3 going = (move.dur / 1000.f) * target_v;
      foot.world_target = getPos() + going + step + offset;
      end = foot.world_target;
    } else {
      float step_l = std::min(sizes_.leg * 0.6f, target_speed_ / 3);
      end = vec3(0, 0, step_l) + foot.offset;
      foot.world_target = to_world * vec4(end, 1);
    }

    vec3 mid_pos = (start + end) / 2.f + vec3(0, options_.step_height, 0);

    vec3 path = end - start;
    vec3 swing_vel = 1.25f * path / (move.dur * cycle_dur_ / 1000);
    vec3 no_vel = vec3(0, 0, 0);
    vec3 toe_drop = vec3(0, -options_.step_height / 2, 0);
    // Subtract root motion when animating in local space
    if (!options_.animate_in_world) {
      no_vel.z -= target_speed_;
      toe_drop.z -= target_speed_;
    }

    move.spline = makeSpline<vec3>(
        SplineType::Hermite,
        {start, no_vel, mid_pos, swing_vel, end, toe_drop});

    startMovement(move, now);
  }

  void plantFoot(Foot& foot) {
    foot.planted = true;
    foot.in_swing = false;
    if (!options_.animate_in_world) {
      foot.world_target = root_.toWorld() * vec4(foot.obj->getPos(), 1);
    }
    mat4 to_world = foot.obj->toWorld();
    foot.world_rot = glm::quat_cast(to_world);
  }

  void updateLegs() {
    updateLeg(*lfemur_, *lshin_, *lfoot_, ik_.lfoot);
    updateLeg(*rfemur_, *rshin_, *rfoot_, ik_.rfoot);
  }

  void updateLeg(Object& femur, Object& shin, Object& foot, Foot& ik_foot) {
    float angle = ik_foot.angle;
    glm::quat ankle_rot = glm::angleAxis(glm::radians(angle), vec3(1, 0, 0));
    vec3 ankle = sizes_.ankle;
    ankle = ankle_rot * ankle;
    if (angle < 0) {
      // Lift up foot if it would go through the ground.
      vec3 target = root_.toLocal() * vec4(ik_foot.world_target, 1);
      float above = ik_foot.obj->getPos().y - target.y;
      float lift = glm::rotate(vec2(sizes_.foot_l, 0), -glm::radians(angle)).y;
      ankle.y += std::max(lift - above, 0.f);
      // TODO: Move foot back so heel doesn't slide forward
    }
    // Ankle in root space
    vec3 root_ankle = ik_foot.obj->getTransform() * vec4(ankle, 1);
    // Positions in pelvis space.
    vec3 pelvis_ankle = pelvis_->toLocal(&root_) * vec4(root_ankle, 1);
    vec3 target = pelvis_ankle - femur.getPos();
    float target_l = glm::length(target);

    glm::quat point_foot =
        glm::rotation(vec3(0, -1, 0), glm::normalize(target));

    auto [hip, knee] = solveIk(sizes_.femur, sizes_.shin, target_l);
    femur.setRot(point_foot * glm::angleAxis(hip, vec3(1, 0, 0)));
    shin.setRot(glm::angleAxis(knee, vec3(1, 0, 0)));

    mat4 foot_to_root = shin.toLocal(&root_);
    glm::quat foot_rot = ankle_rot * glm::quat_cast(foot_to_root);
    foot.setRot(foot_rot);
  }

  // Returns pair of angles for bone1 and bone2.
  std::pair<float, float> solveIk(float bone1, float bone2, float target) {
    if (target >= bone1 + bone2) {
      return {0.f, 0.f};
    }

    float b1 = -cosineLaw(bone1, target, bone2);
    float b2 = glm::radians(180.f) - cosineLaw(bone1, bone2, target);
    return {b1, b2};
  }

  MoveOptions options_;
  SkellySizes sizes_;

  Object root_;
  Object cog_;
  Object* pelvis_;
  Object* torso_;
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
};
