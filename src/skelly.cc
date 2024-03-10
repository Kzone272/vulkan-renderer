#include "skelly.h"

#include <imgui/imgui.h>

#include <print>

#include "animation.h"
#include "maths.h"
#include "time-utils.h"

namespace {

// Some presets for different walk cycles
void moveDefault(MoveOptions& move) {
  move = MoveOptions{};
}
void moveTightrope(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 120;
  move.crouch_pct = 0.9;
  move.stance_w_pct = 0;
  move.hand_height_pct = 0.25;
  move.arm_span_pct = 0.9;
}
void movePreppy(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 240;
  move.crouch_pct = 0.9;
  move.step_height = 15;
  move.bounce = 5;
  move.arm_span_pct = 0;
  move.hand_height_pct = 0.5;
}
void moveSnow(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 100;
  move.crouch_pct = 0.9;
  move.step_height = 50;
  move.arm_span_pct = 0.25;
  move.hand_height_pct = 0.65;
  move.hands_forward = 15;
}
void moveRunway(MoveOptions& move) {
  move = MoveOptions{};
  move.stance_w_pct = 0.2;
  move.hip_sway = 12;
  move.hip_spin = 10;
  move.shoulder_spin = 12;
  move.arm_span_pct = 0.5;
  move.hand_height_pct = 0.9;
  move.hands_forward = -15;
}
void moveCrouch(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 120;
  move.crouch_pct = 0.65;
  move.stance_w_pct = 1.2;
  move.step_height = 15;
  move.arm_span_pct = 0.2;
  move.hand_height_pct = 0.5;
  move.hands_forward = 15;
}
void moveFlanders(MoveOptions& move) {
  move = MoveOptions{};
  move.max_speed = 165;
  move.bounce = 4;
  move.hip_sway = 23;
  move.hip_spin = 4.5;
  move.hand_height_pct = 0.53;
  move.hands_forward = 0;
}

// Some presets for different skeleton sizes
void sizeDefault(SkellySizes& sizes) {
  sizes = SkellySizes{};
}
void sizeTall(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 250;
  sizes.leg = 140;
  sizes.arm = 110;
}
void sizeBig(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 210;
  sizes.leg = 100;
  sizes.bone_w = 10;
  sizes.pelvis_w = 60;
  sizes.shoulders_w = 100;
  sizes.arm = 100;
}
void sizeDwarf(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 114;
  sizes.leg = 40;
  sizes.pelvis_w = 25;
  sizes.shoulders_w = 40;
  sizes.arm = 50;
}
void sizeChimp(SkellySizes& sizes) {
  sizes = SkellySizes{};
  sizes.height = 150;
  sizes.leg = 50;
  sizes.arm = 100;
  sizes.bicep_pct = 0.4;
}

}  // namespace

Skelly::Skelly() {
  rig_.root_.setPos(vec3(200, 0, 200));
  makeBones();
  updateMovements();
  lean_so_ = std::make_unique<SecondOrder<vec3>>(options_.lean_params, vec3{0});
}

void Skelly::makeBones() {
  sizes_.pelvis_y = sizes_.leg + sizes_.pelvis_h;
  sizes_.shoulders_y =
      sizes_.height - sizes_.pelvis_y - sizes_.neck - sizes_.head_h;
  float leg_l = sizes_.leg - sizes_.ankle.y;
  sizes_.femur = sizes_.femur_pct * leg_l;
  sizes_.shin = leg_l - sizes_.femur;
  sizes_.wrist_d = sizes_.arm - sizes_.hand_l;
  sizes_.bicep = sizes_.bicep_pct * sizes_.wrist_d;
  sizes_.forearm = sizes_.wrist_d - sizes_.bicep;

  rig_.makeBones(sizes_);
}

void Skelly::handleInput(const InputState& input, Time now) {
  if (input.kb.pressed.contains(' ')) {
    vec3 start = rig_.root_.getPos();

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

    auto spline = Spline<vec3>(
        SplineType::Bezier, {start, posb, posc, end, posd, pose, end2});
    // root_.addPosAnim(Animation<vec3>(spline, 800, now));
  }

  // Check if we should update velocity because max speed slider changed.
  // TODO: Use slider update directly.
  bool max_speed_changed = target_speed_ > 0.1f &&
                           std::abs(target_speed_ - options_.max_speed) > 0.1f;

  target_speed_changed_ = false;
  if (glm::length(input.move.dir - input_dir_) > 0.1f || max_speed_changed) {
    input_dir_ = input.move.dir;
    vec3 target_vel = options_.max_speed * vec3(input_dir_.x, 0, input_dir_.y);

    float speed = glm::length(target_vel);
    if (std::abs(speed - target_speed_) > 0.1f) {
      target_speed_ = speed;
      target_speed_changed_ = true;
    }

    if (options_.adjust_time == 0) {
      vel_ = target_vel;
    } else {
      float acc_force = options_.max_speed / options_.adjust_time;
      float adjust_time =
          std::max(200.f, glm::length(target_vel - vel_) / acc_force);

      vec3 acc(0);
      vec3 curr_vel = vel_;
      if (vel_curve_) {
        curr_vel = vel_curve_->sample(now);
        acc = vel_curve_->sample(now, SampleType::Velocity) /
              vel_curve_->dur_ms_ * 1000.f;
      }
      auto spline = Spline<vec3>(
          SplineType::Hermite, {curr_vel, acc, target_vel, vec3(0)});
      vel_curve_ = Animation<vec3>(spline, adjust_time, now);
    }
  }
}

void Skelly::update(Time now, float delta_s) {
  updateSpeed(now, delta_s);
  // Root animate used for awkward jump animation I should probably delete.
  rig_.root_.animate(now);
  updateCycle(now, delta_s);
  updateCog(now, delta_s);
  updatePelvis(now);
  updateFeet(now);
  updateLegs();
  updateShoulders(now);
  updateHands(now);
  updateArms();
}

vec3 Skelly::getPos() {
  return rig_.root_.getPos();
}

Object* Skelly::getObj() {
  return &rig_.root_;
}

float Skelly::getPelvisHeight() {
  return sizes_.pelvis_y;
}

void Skelly::UpdateImgui() {
  ImGui::Begin("Skeleton");

  ImGui::SliderFloat("Max Speed", &options_.max_speed, 0, 500);
  ImGui::Separator();

  ImGui::BeginTabBar("Skeleton Tabs");
  if (ImGui::BeginTabItem("Movement")) {
    if (ImGui::Combo(
            "Movement Presets", &ui_.move_preset,
            "Normal\0Tightrope\0Preppy\0Snow\0Runway\0Crouch\0Flanders")) {
      if (ui_.move_preset == 0) {
        moveDefault(options_);
      } else if (ui_.move_preset == 1) {
        moveTightrope(options_);
      } else if (ui_.move_preset == 2) {
        movePreppy(options_);
      } else if (ui_.move_preset == 3) {
        moveSnow(options_);
      } else if (ui_.move_preset == 4) {
        moveRunway(options_);
      } else if (ui_.move_preset == 5) {
        moveCrouch(options_);
      } else if (ui_.move_preset == 6) {
        moveFlanders(options_);
      }
      makeBones();
    }

    ImGui::SliderFloat("Vel Adjust Time", &options_.adjust_time, 0, 1000);
    ImGui::SliderFloat("Max Rot Speed", &options_.max_rot_speed, 1, 360);
    ImGui::SliderFloat("Crouch %", &options_.crouch_pct, 0.01, 1.2);
    if (ImGui::SliderFloat("Stance W %", &options_.stance_w_pct, 0, 2)) {
      makeBones();
    }
    ImGui::SliderFloat("Step Height", &options_.step_height, 0, 50);
    ImGui::SliderFloat("Lean", &options_.lean, 0, 200);
    ImGui::SliderFloat("Bounce", &options_.bounce, 0, 10);
    ImGui::SliderFloat("Hip Sway", &options_.hip_sway, 0, 30);
    ImGui::SliderFloat("Hip Spin", &options_.hip_spin, 0, 45);
    ImGui::SliderFloat("Heel Lift %", &options_.heel_lift_pct, 0, 2);
    ImGui::SliderFloat("Heels Shift", &options_.heel_shift, 0, 90);
    ImGui::SliderFloat("Shoulder Spin", &options_.shoulder_spin, 0, 45);
    ImGui::SliderFloat("Arm Span %", &options_.arm_span_pct, -0.2, 1);
    ImGui::SliderFloat("Hand Height %", &options_.hand_height_pct, -1, 1);
    ImGui::SliderFloat("Hands Forward", &options_.hands_forward, -50, 50);

    if (ImGui::SliderFloat3(
            "Lean SO", (float*)&options_.lean_params, 0.001, 20, "%.5f")) {
      lean_so_->updateParams(options_.lean_params);
    }

    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Sizes")) {
    if (ImGui::Combo(
            "Size Presets", &ui_.size_preset,
            "Default\0Tall\0Big\0Dwarf\0Chimp")) {
      if (ui_.size_preset == 0) {
        sizeDefault(sizes_);
      } else if (ui_.size_preset == 1) {
        sizeTall(sizes_);
      } else if (ui_.size_preset == 2) {
        sizeBig(sizes_);
      } else if (ui_.size_preset == 3) {
        sizeDwarf(sizes_);
      } else if (ui_.size_preset == 4) {
        sizeChimp(sizes_);
      }
      std::println("change size");
      makeBones();
    }
    ImGui::Separator();

    bool changed = false;
    changed |= ImGui::SliderFloat("Height", &sizes_.height, 1, 250);
    changed |= ImGui::SliderFloat("Leg", &sizes_.leg, 1, sizes_.height);
    changed |= ImGui::SliderFloat("Femur", &sizes_.femur_pct, 0.05, 1);
    changed |= ImGui::SliderFloat("Bone W", &sizes_.bone_w, 0.5, 10);
    changed |= ImGui::SliderFloat("Pelvis W", &sizes_.pelvis_w, 1, 60);
    changed |= ImGui::SliderFloat("Shoudlers W", &sizes_.shoulders_w, 1, 100);
    changed |= ImGui::SliderFloat("Arm", &sizes_.arm, 1, 150);
    changed |= ImGui::SliderFloat("Bicep %", &sizes_.bicep_pct, 0.05, 1);
    if (changed) {
      makeBones();
    }

    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Cycle")) {
    cycleUi(walk_);
    ImGui::EndTabItem();
  }

  ImGui::EndTabBar();
  ImGui::End();
}

void Skelly::updateSpeed(Time now, float delta_s) {
  vec3 curr_vel = vel_;

  if (vel_curve_) {
    vel_ = vel_curve_->sample(now);
    if (now > vel_curve_->to_time_) {
      vel_curve_.reset();
    }
    // Average old and new velocity for this frame's update.
    curr_vel += vel_;
    curr_vel /= 2;
  }

  auto pos = getPos();
  pos += curr_vel * delta_s;
  rig_.root_.setPos(pos);

  if (glm::length(curr_vel) > 0.1) {
    float target_angle = glm::orientedAngle(
        vec3(0, 0, 1), glm::normalize(curr_vel), vec3(0, 1, 0));
    target_angle = fmodClamp(target_angle, glm::radians(360.f));

    float current = glm::angle(rig_.root_.getRot());
    float delta = angleDelta(current, target_angle);

    float angle = target_angle;
    float change = delta_s * glm::radians(options_.max_rot_speed);
    if (std::abs(delta) > change) {
      float dir = (delta > 0) ? 1 : -1;
      angle = current + dir * change;
    }
    angle = fmodClamp(angle, glm::radians(360.f));

    rig_.root_.setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
  }
}

bool Skelly::inCycle(const auto& move, float t) {
  if (t < move.offset) {
    t += 1;
    DASSERT(t >= move.offset);
  }
  float end = move.offset + move.dur;
  return t <= end;
}

void Skelly::checkMove(auto& move, Time now, bool moves_changed, float prev_t) {
  if (!move.loop) {
    move.should_start = !inCycle(move, prev_t) && inCycle(move, cycle_t_);
  } else if (moves_changed) {
    startMovement(move, now);
  }
}

void Skelly::updateCycle(Time now, float delta_s) {
  // TODO: Don't update cycle when standing?
  // Maybe it should start once we take the first step, then stop once
  // velocity reaches zero?
  // if ((false) && glm::length(vel_) < 0.1f) {
  //   cycle_t_ = 0;
  // }

  float prev_t = cycle_t_;
  cycle_t_ = fmod(cycle_t_ + delta_s / (cycle_dur_ / 1000.f), 1.f);

  if (target_speed_changed_) {
    if (target_speed_ != 0) {
      float step_l = sizes_.leg * 0.76f;
      float step_dur = (step_l / target_speed_) * 1000.f;
      cycle_dur_ = std::min(1200.f, 2 * step_dur);
    }
    updateMovements();
  }

  // TODO: Easy way to put all moves in a list?
  checkMove(walk_.lstep, now, target_speed_changed_, prev_t);
  checkMove(walk_.rstep, now, target_speed_changed_, prev_t);
  checkMove(walk_.lheel, now, target_speed_changed_, prev_t);
  checkMove(walk_.rheel, now, target_speed_changed_, prev_t);
  checkMove(walk_.bounce, now, target_speed_changed_, prev_t);
  checkMove(walk_.sway, now, target_speed_changed_, prev_t);
  checkMove(walk_.spin, now, target_speed_changed_, prev_t);
  checkMove(walk_.heels_add, now, target_speed_changed_, prev_t);
  checkMove(walk_.larm, now, target_speed_changed_, prev_t);
  checkMove(walk_.rarm, now, target_speed_changed_, prev_t);
  checkMove(walk_.shoulders, now, target_speed_changed_, prev_t);
}

void Skelly::updateMovements() {
  float bounce = std::pow(cycle_dur_ / 1000, 2) * options_.bounce;
  walk_.bounce.spline =
      Spline<float>(SplineType::Hermite, {-bounce, 0, bounce, 0, -bounce, 0});

  float hip_sway = glm::radians(options_.hip_sway);
  walk_.sway.spline = Spline<float>(
      SplineType::Hermite, {-hip_sway, 0, hip_sway, 0, -hip_sway, 0});

  float hip_spin = glm::radians(options_.hip_spin);
  walk_.spin.spline = Spline<float>(
      SplineType::Hermite, {-hip_spin, 0, hip_spin, 0, -hip_spin, 0});

  std::vector<float> values = {0, 10, 50, 5, -15, 5, 0, 0};
  float speed_scale = std::min(target_speed_, 300.f) / 150.f;
  for (float& val : values) {
    val *= options_.heel_lift_pct * speed_scale;
  }
  walk_.lheel.spline = Spline<float>(SplineType::Hermite, std::move(values));
  walk_.rheel.spline = walk_.lheel.spline;

  float heel = options_.heel_shift;
  walk_.heels_add.spline =
      Spline<float>(SplineType::Hermite, {-heel, 0, heel, 0, -heel, 0});

  float shoulder_rot = glm::radians(options_.shoulder_spin);
  walk_.shoulders.spline = Spline<float>(
      SplineType::Hermite,
      {shoulder_rot, 0, -shoulder_rot, 0, shoulder_rot, 0});

  float hand_dist = options_.hand_height_pct * sizes_.wrist_d;
  vec3 shoulder = rig_.lbicep_->getPos();
  float hand_width = options_.arm_span_pct * sizes_.wrist_d;
  vec3 back =
      vec3(-hand_width, -hand_dist, -0.2 * hand_dist + options_.hands_forward) +
      shoulder;
  vec3 forward = vec3(
                     -hand_width, -hand_dist + 5,
                     0.4 * hand_dist + options_.hands_forward) +
                 shoulder;
  walk_.larm.spline = Spline<vec3>(
      SplineType::Hermite, {back, vec3(0), forward, vec3(0), back, vec3(0)});

  walk_.rarm.spline = walk_.larm.spline;
  // Flips right arm points on x axis.
  mat3 flip = mat3(glm::scale(vec3(-1, 1, 1)));
  for (vec3& point : walk_.rarm.spline.points_) {
    point = flip * point;
  }
}

void Skelly::startMovement(auto& move, Time now) {
  float dur = move.dur * cycle_dur_;
  Time start = getMoveStart(move, now);
  move.anim = Animation(move.spline, dur, start, move.loop);
}

Time Skelly::getMoveStart(auto& move, Time now) {
  float t = cycle_t_;
  if (t < move.offset) {
    t += 1;
    DASSERT(t >= move.offset);
  }
  float pos_in_anim = t - move.offset;
  return addMs(now, -pos_in_anim * cycle_dur_);
}

vec3 Skelly::sampleMovement(Movement<vec3>& move, Time now) {
  if (move.anim) {
    return move.anim->sample(now);
  }
  return vec3(0);
}

float Skelly::sampleMovement(Movement<float>& move, Time now) {
  if (move.anim) {
    return move.anim->sample(now);
  }
  return 0;
}

void Skelly::updateCog(Time now, float delta_s) {
  vec3 pos = vec3(0, options_.crouch_pct * sizes_.pelvis_y, 0);
  pos.y += sampleMovement(walk_.bounce, now);

  vec3 lean_target(0);
  if (vel_curve_) {
    vec3 acc =
        vel_curve_->sample(now, SampleType::Velocity) / vel_curve_->dur_ms_;
    lean_target = options_.lean * acc;
  }
  vec3 lean;
  if (delta_s == 0) {
    lean = lean_target;
  } else {
    lean = lean_so_->update(delta_s, lean_target);
  }

  vec3 offset = rig_.root_.toLocal() * vec4(lean, 0);
  pos += offset;

  rig_.cog_->setPos(pos);
}

void Skelly::updatePelvis(Time now) {
  rig_.pelvis_c_.sway = sampleMovement(walk_.sway, now);
  rig_.pelvis_c_.spin = sampleMovement(walk_.spin, now);

  glm::quat rot = glm::angleAxis(rig_.pelvis_c_.spin, vec3(0, 1, 0)) *
                  glm::angleAxis(rig_.pelvis_c_.sway, vec3(0, 0, -1));
  rig_.pelvis_->setRot(rot);
}

void Skelly::updateFeet(Time now) {
  updateHeel(now, rig_.lfoot_c_, walk_.lheel);
  updateHeel(now, rig_.rfoot_c_, walk_.rheel);
  updateFoot(rig_.rfoot_c_, now, walk_.rstep);
  updateFoot(rig_.lfoot_c_, now, walk_.lstep);
}

void Skelly::updateHeel(Time now, Foot& foot, Movement<float>& move) {
  if (move.should_start) {
    startMovement(move, now);
  }

  foot.angle = sampleMovement(move, now);
  if (move.anim && now > move.anim->to_time_) {
    move.anim.reset();
    foot.angle = 0;
  }

  foot.angle += sampleMovement(walk_.heels_add, now);
}

void Skelly::updateFoot(Foot& foot, Time now, Movement<vec3>& move) {
  {
    // This code isn't used anymore, but probably should be used to determine
    // when walking starts, or if we should move feet back when stopped.
    bool supported = !rig_.lfoot_c_.in_swing && !rig_.rfoot_c_.in_swing;
    vec3 pos = foot.obj->getPos();
    float target_dist = glm::length(pos - foot.start_pos);
    bool should_step = supported && target_dist > options_.foot_dist;
  }

  if (move.should_start) {
    swingFoot(foot, now, move);
  }

  if (foot.planted) {
    mat4 to_local = rig_.root_.toLocal();
    foot.obj->setPos(to_local * vec4(foot.world_target, 1));
  }

  if (move.anim) {
    if (now > move.anim->to_time_) {
      vec3 plant_pos = move.anim->sample(move.anim->to_time_);
      foot.obj->setPos(plant_pos);
      move.anim.reset();
      rig_.plantFoot(foot);
    } else {
      vec3 swing_pos = sampleMovement(move, now);
      foot.obj->setPos(swing_pos);
    }
  }
}

void Skelly::swingFoot(Foot& foot, Time now, Movement<vec3>& move) {
  foot.in_swing = true;
  foot.planted = false;

  vec3 start = foot.obj->getPos();
  mat4 to_world = rig_.root_.toWorld();

  float forward = std::min(sizes_.leg * 0.3f, target_speed_ / 3);
  vec3 step_offset = foot.start_pos;
  step_offset.x =
      (foot.is_left ? -1 : 1) * options_.stance_w_pct * sizes_.pelvis_w / 2;
  vec3 end = vec3(0, 0, forward) + step_offset;
  foot.world_target = to_world * vec4(end, 1);

  vec3 mid_pos = (start + end) / 2.f + vec3(0, options_.step_height, 0);

  vec3 path = end - start;
  vec3 swing_vel = 1.25f * path / (move.dur * cycle_dur_ / 1000);
  vec3 no_vel = vec3(0, 0, -target_speed_);
  vec3 toe_drop = vec3(0, -options_.step_height / 2, -target_speed_);

  move.spline = Spline<vec3>(
      SplineType::Hermite, {start, no_vel, mid_pos, swing_vel, end, toe_drop});

  startMovement(move, now);
}

void Skelly::updateLegs() {
  updateLeg(*rig_.lfemur_, *rig_.lshin_, *rig_.lfoot_, rig_.lfoot_c_);
  updateLeg(*rig_.rfemur_, *rig_.rshin_, *rig_.rfoot_, rig_.rfoot_c_);
}

// TODO: Use updateTwoBoneIk() for most of this.
void Skelly::updateLeg(
    Object& femur, Object& shin, Object& foot, Foot& ik_foot) {
  float toe_angle = 0;
  // If hip is too far from ankle, compute the angle to lift the heel just
  // enough to compensate.
  vec3 hip = femur.toAncestor(&rig_.root_) * vec4(0, 0, 0, 1);
  vec3 ankle_pos = ik_foot.obj->toAncestor(&rig_.root_) * vec4(sizes_.ankle, 1);
  float hip_to_ankle = glm::length(hip - ankle_pos);
  float max_leg = 0.97 * (sizes_.femur + sizes_.shin);
  if (hip_to_ankle > max_leg) {
    float hip_to_toe = glm::length(ik_foot.obj->getPos() - hip);
    float toe_to_ankle = glm::length(sizes_.ankle);
    float flat_angle = cosineLaw(toe_to_ankle, hip_to_toe, hip_to_ankle);
    if (hip_to_toe > max_leg + toe_to_ankle) {
      // Target for toe is too far away. Point toes at target.
      toe_angle = flat_angle;
    } else {
      float lift_angle = cosineLaw(toe_to_ankle, hip_to_toe, max_leg);
      toe_angle = flat_angle - lift_angle;
    }
  }

  // float angle = glm::radians(ik_foot.angle); // Ignore animated heel angle.
  glm::quat ankle_rot = glm::angleAxis(toe_angle, vec3(1, 0, 0));
  vec3 ankle = sizes_.ankle;
  ankle = ankle_rot * ankle;

  // TODO: this currently isn't possible with heel angle determined by IK. But
  // this will become relevant again if I add an animation to land on the heel.
  if (toe_angle < 0) {
    // Lift up foot if it would go through the ground.
    vec3 target = rig_.root_.toLocal() * vec4(ik_foot.world_target, 1);
    float above = ik_foot.obj->getPos().y - target.y;
    float lift =
        glm::rotate(vec2(sizes_.foot_l, 0), -glm::radians(toe_angle)).y;
    ankle.y += std::max(lift - above, 0.f);
    // TODO: Move foot back so heel doesn't slide forward
  }

  // Ankle in pelvis space.
  vec3 pelvis_ankle = rig_.pelvis_->toLocal(&rig_.root_) *
                      ik_foot.obj->getTransform() * vec4(ankle, 1);

  updateTwoBoneIk(
      femur, sizes_.femur, shin, sizes_.shin, femur.getPos(), pelvis_ankle,
      vec3(0, -1, 0), vec3(1, 0, 0));

  mat4 foot_to_root = shin.toLocal(&rig_.root_);
  glm::quat foot_rot = ankle_rot * glm::quat_cast(foot_to_root);
  foot.setRot(foot_rot);
}

// b1_pos, target, main_axis, and rot_axis all need to be in b1's parent space.
void Skelly::updateTwoBoneIk(
    Object& bone1, float b1_l, Object& bone2, float b2_l, vec3 b1_pos,
    vec3 target, vec3 main_axis, vec3 rot_axis) {
  // target and b1_pos in the same space.
  vec3 toward = target - b1_pos;
  float toward_l = glm::length(toward);

  glm::quat point = glm::rotation(main_axis, glm::normalize(toward));

  auto [j1, j2] = solveIk(b1_l, b2_l, toward_l);
  bone1.setRot(point * glm::angleAxis(j1, rot_axis));
  bone2.setRot(glm::angleAxis(j2, rot_axis));
}

void Skelly::updateShoulders(Time now) {
  float angle = sampleMovement(walk_.shoulders, now);
  rig_.torso_->setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
}

void Skelly::updateHands(Time now) {
  // This animation is in torso space, but the hands are in root space.
  if (walk_.larm.anim) {
    vec3 root_hand = sampleMovement(walk_.larm, now);
    rig_.lhand_c_.obj->setPos(
        rig_.torso_->posToAncestor(&rig_.root_, root_hand));
  }
  if (walk_.rarm.anim) {
    vec3 root_hand = sampleMovement(walk_.rarm, now);
    rig_.rhand_c_.obj->setPos(
        rig_.torso_->posToAncestor(&rig_.root_, root_hand));
  }
}

void Skelly::updateArms() {
  updateTwoBoneIk(
      *rig_.lbicep_, sizes_.bicep, *rig_.lforearm_, sizes_.forearm,
      rig_.lbicep_->getPos(),
      rig_.torso_->posToLocal(&rig_.root_, rig_.lhand_c_.obj->getPos()),
      vec3(-1, 0, 0), vec3(0, 1, 0));
  updateTwoBoneIk(
      *rig_.rbicep_, sizes_.bicep, *rig_.rforearm_, sizes_.forearm,
      rig_.rbicep_->getPos(),
      rig_.torso_->posToLocal(&rig_.root_, rig_.rhand_c_.obj->getPos()),
      vec3(1, 0, 0), vec3(0, -1, 0));
}

// Returns pair of angles for bone1 and bone2.
std::pair<float, float> Skelly::solveIk(
    float bone1, float bone2, float target) {
  if (target >= bone1 + bone2) {
    return {0.f, 0.f};
  }

  float b1 = -cosineLaw(bone1, target, bone2);
  float b2 = glm::radians(180.f) - cosineLaw(bone1, bone2, target);
  return {b1, b2};
}

void Skelly::cycleUi(Cycle& cycle) {
  movementUi("lstep", cycle.lstep);
  movementUi("rstep", cycle.rstep);
}

void Skelly::movementUi(const std::string& label, auto& move) {
  ImGui::PushID(label.c_str());

  ImGui::Text("%s: Offset | Duration", label.c_str());
  ImGui::SameLine();
  ImGui::Checkbox("Loop", &move.loop);

  ImGui::PushID("offsetdur");
  float vals[2] = {move.offset, move.dur};
  if (ImGui::SliderFloat2("", vals, 0, 1)) {
    move.offset = vals[0];
    move.dur = vals[1];
  }
  ImGui::PopID();  // offsetdur

  // Draw duration bar.
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  ImVec2 bar_size = ImVec2(ImGui::CalcItemWidth(), ImGui::GetFrameHeight());
  ImVec2 p0 = ImGui::GetCursorScreenPos();
  ImVec2 p1 = ImVec2(p0.x + bar_size.x, p0.y + bar_size.y);
  draw_list->AddRectFilled(p0, p1, IM_COL32_BLACK);  // bg
  ImVec2 p2 = ImVec2(p0.x + move.offset * bar_size.x, p0.y);
  float t2 = move.offset + move.dur;
  float end = remapRange(t2, 0, 1, p0.x, p1.x);  // this is clamped
  ImVec2 p3 = ImVec2(end, p1.y);
  draw_list->AddRectFilled(p2, p3, IM_COL32_WHITE);  // first chunk
  if (t2 > 1) {
    float t2_wrapped = t2 - 1;
    float wrapped_end = remapRange(t2_wrapped, 0, 1, p0.x, p1.x);
    ImVec2 p4 = ImVec2(wrapped_end, p1.y);
    draw_list->AddRectFilled(p0, p4, IM_COL32_WHITE);  // wrapped chunk
  }
  ImGui::PushID("bar");
  ImGui::InvisibleButton("bar", bar_size);  // advances cursor past the rect
  ImGui::PopID();                           // bar

  ImGui::PopID();  // label
}