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
  float plant_pct = 0;
  float max_rot_speed = 270;
  float lean = 0.1f;
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
      root_.animPos(makeAnimation(spline, 800, now));
    }

    if (glm::length(input.move.dir - input_dir_) > 0.1f) {
      input_dir_ = input.move.dir;
      vec3 target_vel =
          options_.max_speed * vec3(input_dir_.x, 0, input_dir_.y);

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

  void update(Time now, float time_delta_s) {
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
    pos += curr_vel * time_delta_s;
    root_.setPos(pos);

    if (glm::length(curr_vel) > 0.1) {
      float target_angle = glm::orientedAngle(
          vec3(0, 0, 1), glm::normalize(curr_vel), vec3(0, 1, 0));
      target_angle = fmodClamp(target_angle, glm::radians(360.f));

      float current = glm::angle(root_.getRot());
      float delta = angleDelta(current, target_angle);

      float angle = target_angle;
      float change = time_delta_s * glm::radians(options_.max_rot_speed);
      if (std::abs(delta) > change) {
        float dir = (delta > 0) ? 1 : -1;
        angle = current + dir * change;
      }
      angle = fmodClamp(angle, glm::radians(360.f));

      root_.setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
    }

    vec2 lean = options_.lean * curr_vel.xz();
    vec3 pelvis_pos = glm::inverse(root_.getTransform()) *
                      vec4(root_.getPos() + vec3(lean.x, 0, lean.y), 1);
    pelvis_pos.y = pelvis_->getPos().y;
    pelvis_->setPos(pelvis_pos);

    root_.animate(now);
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
    Time move_again;
  };

  void updateFeet(Time now) {
    updateFoot(rfoot_, now);
    updateFoot(lfoot_, now);
  }

  void updateFoot(Foot& foot, Time now) {
    vec3 pos = foot.obj->getPos();

    float speed = glm::length(vel_);
    if (vel_curve_) {
      // Use target speed.
      speed = glm::length(Animation::sample(*vel_curve_, now + 10s));
    }
    float step_l = std::min(sizes_.leg * 0.6f, speed / 3);

    float foot_dist = std::max(1.f, step_l);
    if (lfoot_.planted && rfoot_.planted) {
      foot_dist = options_.foot_dist;
    }

    bool can_move =
        now > foot.move_again && !lfoot_.in_swing && !rfoot_.in_swing;
    float target_dist = glm::length(foot.obj->getPos() - foot.offset);
    // printf("can move :%d dist: %f\n", can_move, target_dist);
    // printf("foot_dist:%f step_l: %f\n", foot_dist, step_l);
    if (can_move && target_dist > foot_dist) {
      foot.in_swing = true;
      foot.planted = false;

      float step_dur = 1.5f * (step_l / speed) * 1000.f;
      if (speed == 0) {
        step_dur = 500;
      }

      // Scale speeds based on the duration of the animation.
      float scale = (step_dur / 1000.f);
      float scaled_speed = speed * scale;
      vec3 target_pos = vec3(0, 0, step_l) + foot.offset;
      vec3 no_vel = vec3(0, 0, -scaled_speed);
      vec3 mid_pos =
          (pos + target_pos) / 2.f + vec3(0, options_.step_height, 0);
      vec3 swing_vel = 1.5f * scaled_speed * glm::normalize(target_pos - pos);
      auto spline = makeSpline(
          Spline::Type::Hermite,
          {pos, -foot.vel * scale, mid_pos, swing_vel, target_pos, no_vel});

      foot.obj->animPos(makeAnimation(spline, step_dur, now));
      foot.obj->setRot(glm::quat());
      foot.move_again =
          now + std::chrono::duration_cast<Clock::duration>(
                    FloatMs((1.f + options_.plant_pct) * 2.f * step_dur));
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
      return {0, 0};
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

  vec2 input_dir_{0};
  std::optional<Animation> vel_curve_;
  vec3 vel_{0};
};
