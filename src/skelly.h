#pragma once

#include "animation.h"
#include "glm-include.h"
#include "input.h"
#include "utils.h"

struct MoveOptions {
  float max_speed = 200;
  float adjust_time = 500;
  float stance_w = 30;
  float foot_dist = 5;
  float step_height = 20;
  float max_rot_speed = 270;
  float lean = 0.05;
  float bounce = 5;
};

struct SkellySizes {
  float height = 185;
  float bonew = 6;
  float leg = 100;
  float femur = 50;
  float pelvisw = 30;
  float shouldersw = 50;
};

class Skelly {
 public:
  Skelly() {
    root_.setPos(vec3(200, 0, 200));
    makeBones();
  }

  void makeBones() {
    root_.clearChildren();

    mat4 pelvis_t = glm::translate(vec3(0, -10, 0)) *
                    glm::scale(vec3(sizes_.pelvisw, 20, 20));
    pelvis_ = root_.addChild(std::make_unique<Object>(ModelId::Bone, pelvis_t));
    pelvis_->setPos(vec3(0, sizes_.leg, 0));

    mat4 torso_t = glm::scale(vec3(sizes_.shouldersw, -20, 25));
    torso_ =
        pelvis_->addChild(std::make_unique<Object>(ModelId::Bone, torso_t));
    torso_->setPos(vec3(0, sizes_.height - sizes_.leg - 30, 0));

    mat4 head_t = glm::scale(vec3(20, -25, 25));
    head_ = torso_->addChild(std::make_unique<Object>(ModelId::Bone, head_t));
    head_->setPos(vec3(0, 30, 5));

    mat4 femur_t = glm::scale(vec3(sizes_.bonew, -sizes_.femur, sizes_.bonew));
    lfemur_ =
        pelvis_->addChild(std::make_unique<Object>(ModelId::Bone, femur_t));
    vec3 femur_pos = vec3(-(sizes_.pelvisw / 2 + 3), 0, 0);
    lfemur_->setPos(femur_pos);

    mat4 shin_t = glm::scale(
        vec3(sizes_.bonew, -(sizes_.leg - sizes_.femur), sizes_.bonew));
    lshin_ = lfemur_->addChild(std::make_unique<Object>(ModelId::Bone, shin_t));
    vec3 shin_pos = vec3(0, -sizes_.femur, 0);
    lshin_->setPos(shin_pos);

    // Add opposite limbs with flipped positions.
    mat3 flip = mat3(glm::scale(vec3(-1, 1, 1)));

    rfemur_ =
        pelvis_->addChild(std::make_unique<Object>(ModelId::Bone, femur_t));
    rfemur_->setPos(flip * femur_pos);

    rshin_ = rfemur_->addChild(std::make_unique<Object>(ModelId::Bone, shin_t));
    rshin_->setPos(flip * shin_pos);

    lfoot_.obj = root_.addChild(
        std::make_unique<Object>(ModelId::Bone, glm::scale(vec3(10, 8, 25))));
    vec3 foot_offset = {-options_.stance_w / 2, 0, 0};
    lfoot_.offset = foot_offset;
    lfoot_.obj->setPos(lfoot_.offset);
    plantFoot(lfoot_);

    rfoot_.obj = root_.addChild(
        std::make_unique<Object>(ModelId::Bone, glm::scale(vec3(10, 8, 25))));
    rfoot_.offset = flip * foot_offset;
    rfoot_.obj->setPos(rfoot_.offset);
    plantFoot(rfoot_);
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

      auto spline = makeSpline(
          Spline::Type::Bezier, {start, posb, posc, end, posd, pose, end2});
      root_.addPosAnim(makeAnimation(spline, 800, now));
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
        printf("speed %f targetsp %f\n", speed, target_speed_);
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
        auto spline = makeSpline(Spline::Type::Bezier, {a, b, c, d});
        vel_curve_ = makeAnimation(spline, options_.adjust_time, now);
      }
    }
  }

  void update(Time now, float delta_s) {
    updateMove(now, delta_s);
    // Root animate used for awkward jump animation I should probably delete.
    root_.animate(now);
    updateCycle(now, delta_s);
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
  struct Foot {
    Object* obj;
    bool planted = false;
    bool in_swing = false;
    vec3 world_pos;
    glm::quat world_rot;
    vec3 offset;
    vec3 vel;
  };
  struct Movement;

  void updateMove(Time now, float delta_s) {
    vec3 curr_vel = vel_;

    if (vel_curve_) {
      vel_ = Animation::sample(*vel_curve_, now);
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

  bool inCycle(const Movement& move, float t) {
    float end = fmod(move.offset + move.dur, 1.f);
    return t >= move.offset && t <= end;
  }

  void checkStarts(const std::vector<Movement*>& moves, float prev_t, float t) {
    for (auto* move : moves) {
      move->should_start = !inCycle(*move, prev_t) && inCycle(*move, t);
    }
  }

  void updateCycle(Time now, float delta_s) {
    float prev_t = cycle_t_;

    // TODO: This check probably needs word.
    // Maybe it should start once we take the first step, then stop once
    // velocity reaches zero?
    if (glm::length(vel_) < 0.1f) {
      cycle_t_ = 0;
      walk_.lstep.should_start = false;
      walk_.rstep.should_start = false;
    } else {
      cycle_t_ = fmod(cycle_t_ + delta_s / (cycle_dur_ / 1000.f), 1.f);
      // Don't check start for looping animations.
      std::vector<Movement*> moves = {&walk_.lstep, &walk_.rstep};
      checkStarts(moves, prev_t, cycle_t_);
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
    printf("updated cycledur %f\n", cycle_dur_);
    updateCurves();
  }

  void updateCurves() {
    float bounce = std::pow(cycle_dur_ / 1000, 2) * options_.bounce;

    walk_.bounce.spline = makeSpline(
        Spline::Type::Hermite, {
                                   {0, -bounce, 0},
                                   {0, 0, 0},
                                   {0, bounce, 0},
                                   {0, 0, 0},
                                   {0, -bounce, 0},
                                   {0, 0, 0},
                               });
  }

  Time getMoveStart(const Movement& move, Time now) {
    float t = cycle_t_;
    if (t < move.offset) {
      t += 1;
      DASSERT(t >= move.offset);
    }
    float pos_in_anim = t - move.offset;
    return addMs(now, -pos_in_anim * cycle_dur_);
  }

  void updatePelvis(Time now) {
    if (target_speed_changed_) {
      pelvis_->clearAddAnims();
      float bounce_dur = walk_.bounce.dur * cycle_dur_;
      Time bounce_start = getMoveStart(walk_.bounce, now);
      pelvis_->addPosAnim(
          makeAnimation(walk_.bounce.spline, bounce_dur, bounce_start, true));
    }

    vec2 lean = options_.lean * vel_.xz();
    vec3 pelvis_pos = glm::inverse(root_.getTransform()) *
                      vec4(root_.getPos() + vec3(lean.x, 0, lean.y), 1);
    pelvis_pos.y = pelvis_->getPos().y;
    // This is really fragile. It's a feedback loop that updates position based
    // on the position. It works because it u
    // TODO: Add a method for adding "offsets" to be added to position like
    // there exists for animations.
    pelvis_->setPos(pelvis_pos);

    pelvis_->animate(now);
  }

  void updateFeet(Time now) {
    updateFoot(rfoot_, now, walk_.rstep);
    updateFoot(lfoot_, now, walk_.lstep);
  }

  void updateFoot(Foot& foot, Time now, const Movement& move) {
    vec3 pos = foot.obj->getPos();
    {
      // This code isn't used anymore, but probably should be used to determine
      // when walking starts, or if we should move feet back when stopped.
      bool supported = !lfoot_.in_swing && !rfoot_.in_swing;
      float target_dist = glm::length(pos - foot.offset);
      bool should_step = supported && target_dist > options_.foot_dist;
    }

    if (move.should_start) {
      swingFoot(foot, now, move);
    }

    if (foot.planted) {
      mat4 to_local = glm::inverse(root_.getTransform());
      foot.obj->setPos(to_local * vec4(foot.world_pos, 1));
      foot.obj->setRot(glm::quat_cast(to_local) * lfoot_.world_rot);
    }

    bool done = foot.obj->animate(now);
    foot.vel = foot.obj->getPos() - pos;

    if (foot.in_swing && done) {
      plantFoot(foot);
    }
  }

  void swingFoot(Foot& foot, Time now, const Movement& move) {
    foot.in_swing = true;
    foot.planted = false;

    float step_dur = move.dur * cycle_dur_;
    float step_l = std::min(sizes_.leg * 0.6f, target_speed_ / 3);

    // Scale speeds based on the duration of the animation.
    float scale = (step_dur / 1000.f);
    float scaled_speed = scale * target_speed_;

    vec3 pos = foot.obj->getPos();
    vec3 target_pos = vec3(0, 0, step_l) + foot.offset;
    vec3 no_vel = vec3(0, 0, -scaled_speed);
    vec3 mid_pos = (pos + target_pos) / 2.f + vec3(0, options_.step_height, 0);
    vec3 swing_vel = 1.5f * scaled_speed * glm::normalize(target_pos - pos);
    auto spline = makeSpline(
        Spline::Type::Hermite,
        {pos, -foot.vel * scale, mid_pos, swing_vel, target_pos, no_vel});

    Time start = getMoveStart(move, now);
    foot.obj->setPosAnim(makeAnimation(spline, step_dur, start));
    foot.obj->setRot(glm::quat());
  }

  void plantFoot(Foot& foot) {
    foot.planted = true;
    foot.in_swing = false;
    mat4 to_world = root_.getTransform();
    foot.world_pos = to_world * vec4(foot.obj->getPos(), 1);
    foot.world_rot = glm::quat_cast(to_world) * foot.obj->getRot();
  }

  void updateLegs() {
    updateLeg(*lfemur_, *lshin_, lfoot_);
    updateLeg(*rfemur_, *rshin_, rfoot_);
  }

  void updateLeg(Object& femur, Object& shin, Foot& foot) {
    float femur_l = sizes_.femur;
    float shin_l = sizes_.leg - femur_l;

    // Positions in root space.
    vec3 hip_pos = pelvis_->getTransform() * vec4(femur.getPos(), 1);
    vec3 foot_pos = foot.obj->getPos() + vec3(0, 10, 0);
    vec3 target = foot_pos - hip_pos;
    float target_l = glm::length(target);

    glm::quat point_foot =
        glm::rotation(vec3(0, -1, 0), glm::normalize(target));

    auto [hip, knee] = solveIk(femur_l, shin_l, target_l);
    femur.setRot(point_foot * glm::angleAxis(hip, vec3(1, 0, 0)));
    shin.setRot(glm::angleAxis(knee, vec3(1, 0, 0)));
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

  Object root_{ModelId::None};
  Object* pelvis_;
  Object* torso_;
  Object* head_;
  Object* lfemur_;
  Object* lshin_;
  Object* rfemur_;
  Object* rshin_;

  Foot lfoot_;
  Foot rfoot_;

  struct Movement {
    float offset;
    float dur;
    bool should_start = false;
    Spline spline;
  };
  struct Cycle {
    Movement lstep;
    Movement rstep;
    Movement bounce;
  };
  Cycle walk_{
      .lstep = {.offset = 0, .dur = 0.45},
      .rstep = {.offset = 0.5, .dur = 0.45},
      .bounce = {.offset = -0.05, .dur = 0.5},
  };
  float cycle_t_ = 0.f;
  float cycle_dur_ = 0.f;

  vec2 input_dir_{0};
  std::optional<Animation> vel_curve_;
  vec3 vel_{0};
  float target_speed_ = 0;
  bool target_speed_changed_ = false;
};
