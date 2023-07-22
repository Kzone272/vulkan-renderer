#pragma once

#include "animation.h"
#include "glm-include.h"
#include "input.h"
#include "utils.h"

struct MoveOptions {
  float max_speed = 150;
  float adjust_time = 500;
  float foot_dist = 15;
  float step_height = 5;
  float plant_pct = 0.1;
  float max_rot_speed = 180;
};

class Skelly {
 public:
  Skelly() {
    root_.setPos(vec3(200, 0, 200));

    pelvis_ = root_.addChild(
        std::make_unique<Object>(ModelId::Bone, glm::scale(vec3(35, 20, 30))));
    pelvis_->setPos(vec3(0, 80, 0));

    lfoot_.obj = root_.addChild(
        std::make_unique<Object>(ModelId::Bone, glm::scale(vec3(10, 8, 25))));
    lfoot_.offset = vec3(-20, 0, 0);
    lfoot_.obj->setPos(lfoot_.offset);
    plantFoot(lfoot_);

    rfoot_.obj = root_.addChild(
        std::make_unique<Object>(ModelId::Bone, glm::scale(vec3(10, 8, 25))));
    rfoot_.offset = vec3(20, 0, 0);
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

    root_.animate(now);
    updateFeet(now);
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
    float step_l = std::min(speed / 3.f, 60.f);

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

  MoveOptions options_;

  Object root_{ModelId::None};
  Object* pelvis_;

  Foot lfoot_;
  Foot rfoot_;

  vec2 input_dir_{0};
  std::optional<Animation> vel_curve_;
  vec3 vel_{0};
};
