#pragma once

#include "animation.h"
#include "glm-include.h"
#include "input.h"

struct MoveOptions {
  float max_speed = 150;
  float adjust_time = 200;
};

class Skelly {
 public:
  Skelly() {
    root_.setPos(vec3(200, 0, 200));

    pelvis_ = root_.addChild(std::make_unique<Object>(ModelId::MOVER));
    pelvis_->setScale(vec3(0.35, 0.2, 0.3));
    pelvis_->setPos(vec3(0, 65, 0));

    lfoot_ = root_.addChild(std::make_unique<Object>(ModelId::MOVER));
    lfoot_->setScale(vec3(0.1, 0.08, 0.25));
    lfoot_->setPos(vec3(-20, 0, 0));

    rfoot_ = root_.addChild(std::make_unique<Object>(ModelId::MOVER));
    rfoot_->setScale(vec3(0.1, 0.08, 0.25));
    rfoot_->setPos(vec3(20, 0, 0));
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
          Spline::Type::BEZIER, {start, posb, posc, end, posd, pose, end2});
      root_.animPos(makeAnimation(spline, 800, now));
    }

    auto dir = getWasdDir(input);
    if (dir != input_dir_) {
      input_dir_ = dir;
      vec3 target_vel =
          options_.max_speed * vec3(input_dir_.x, 0, input_dir_.y);

      if (options_.adjust_time == 0) {
        vel_ = target_vel;
      } else {
        vec3 a = vel_;
        vec3 d = target_vel;
        vec3 b = glm::mix(a, d, 1.f / 3.f);
        vec3 c = d;
        auto spline = makeSpline(Spline::Type::BEZIER, {a, b, c, d});
        vel_curve_ = makeAnimation(spline, options_.adjust_time, now);
      }
    }
  }

  void update(Time now, float time_delta_ms) {
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
    pos += curr_vel * (time_delta_ms / 1000.f);
    root_.setPos(pos);

    if (glm::length(curr_vel) > 0.1) {
      float angle = glm::orientedAngle(
          vec3(0, 0, 1), glm::normalize(curr_vel), vec3(0, 1, 0));
      root_.setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
    }

    root_.animate(now);
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
  MoveOptions options_;

  Object root_{ModelId::NONE};
  Object* pelvis_;
  Object* lfoot_;
  Object* rfoot_;

  vec2 input_dir_{0};
  std::optional<Animation> vel_curve_;
  vec3 vel_{0};
};
