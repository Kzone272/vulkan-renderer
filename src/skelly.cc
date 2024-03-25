#include "skelly.h"

#include <imgui/imgui.h>

#include <print>

#include "animation.h"
#include "maths.h"
#include "skelly-presets.h"
#include "time-utils.h"

Skelly::Skelly() {
  root_.setPos(vec3(200, 0, 200));
  makeBones();
  updateMovements();
  lean_so_ = std::make_unique<SecondOrder<vec3>>(options_.lean_params, vec3{0});
}

void Skelly::makeBones() {
  sizes_.pelvis_y = sizes_.leg + sizes_.pelvis_h;
  sizes_.shoulders_y =
      sizes_.height - sizes_.pelvis_y - sizes_.neck - sizes_.head_h;
  sizes_.ankle_d = sizes_.leg - sizes_.ankle.y;
  sizes_.femur = sizes_.femur_pct * sizes_.ankle_d;
  sizes_.shin = sizes_.ankle_d - sizes_.femur;
  sizes_.wrist_d = sizes_.arm - sizes_.hand_l;
  sizes_.bicep = sizes_.bicep_pct * sizes_.wrist_d;
  sizes_.forearm = sizes_.wrist_d - sizes_.bicep;

  root_.clearChildren();
  skeleton_.makeBones(sizes_, &root_);
  rig_.makeRig(skeleton_, &root_);
  initFoot(lfoot_m_, rig_.lfoot_, rig_.ltoe_);
  initFoot(rfoot_m_, rig_.rfoot_, rig_.rtoe_);
  pose_ = rig_.getZeroPose();

  mod_pose_.type = PoseType::Override;
  mod_pose_.bone_mask = std::set<BoneId>{BoneId::Lhand, BoneId::Rhand};
  vec3 hands_pos(0, 100, 30);
  mod_pose_.getBone(BoneId::Lhand).setPos(hands_pos);
  mod_pose_.getBone(BoneId::Rhand).setPos(hands_pos);
}

void Skelly::handleInput(const InputState& input, Time now) {
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
  root_.animate(now);
  updateCycle(now, delta_s);
  updateCog(now, delta_s);
  updatePelvis(now);
  updateFeet(now);
  updateShoulders(now);
  updateHands(now);
  rig_.applyPose(Pose::blend(pose_, mod_pose_, options_.mod_blend));
  rig_.updateSkeleton(skeleton_);
}

vec3 Skelly::getPos() {
  return root_.getPos();
}

Object* Skelly::getObj() {
  return &root_;
}

float Skelly::getPelvisHeight() {
  return sizes_.pelvis_y;
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

bool Skelly::inCycle(const auto& move, float t) {
  if (t < move.offset) {
    t += 1;
    DASSERT(t >= move.offset);
  }
  float end = move.offset + move.dur;
  return t <= end;
}

float getMoveT(const auto& move, float cycle_t) {
  float end = move.offset + move.dur;
  float move_t = remapRange(cycle_t, move.offset, end, 0, 1);
  if (move.loop) {
    move_t = fmodClamp(move_t, 1);
  }
  return move_t;
}

bool Skelly::moveStarted(auto& move) {
  return !inCycle(move, prev_cycle_t_) && inCycle(move, cycle_t_);
}

bool Skelly::moveStopped(auto& move) {
  return inCycle(move, prev_cycle_t_) && !inCycle(move, cycle_t_);
}

void Skelly::updateCycle(Time now, float delta_s) {
  // TODO: Don't update cycle when standing?
  // Maybe it should start once we take the first step, then stop once
  // velocity reaches zero?
  // if ((false) && glm::length(vel_) < 0.1f) {
  //   cycle_t_ = 0;
  // }

  prev_cycle_t_ = cycle_t_;
  cycle_t_ = fmod(cycle_t_ + delta_s / (cycle_dur_ / 1000.f), 1.f);

  if (target_speed_changed_) {
    if (target_speed_ != 0) {
      float step_l = sizes_.leg * 0.76f;
      float step_dur = (step_l / target_speed_) * 1000.f;
      cycle_dur_ = std::min(1200.f, 2 * step_dur);
    }
    updateMovements();
  }
}

void Skelly::updateMovements() {
  float bounce = std::pow(cycle_dur_ / 1000, 2) * options_.bounce;
  walk_.bounce.spline =
      Spline<float>(SplineType::Hermite, {-bounce, 0, bounce, 0, -bounce, 0});
  startMovement(walk_.bounce);

  float hip_sway = glm::radians(options_.hip_sway);
  walk_.sway.spline = Spline<float>(
      SplineType::Hermite, {-hip_sway, 0, hip_sway, 0, -hip_sway, 0});
  startMovement(walk_.sway);

  float hip_spin = glm::radians(options_.hip_spin);
  walk_.spin.spline = Spline<float>(
      SplineType::Hermite, {-hip_spin, 0, hip_spin, 0, -hip_spin, 0});
  startMovement(walk_.spin);

  float shoulder_rot = glm::radians(options_.shoulder_spin);
  walk_.shoulders.spline = Spline<float>(
      SplineType::Hermite,
      {shoulder_rot, 0, -shoulder_rot, 0, shoulder_rot, 0});
  startMovement(walk_.shoulders);

  float hand_dist = options_.hand_height_pct * sizes_.wrist_d;
  vec3 shoulder = skeleton_.lbicep_->getPos();
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
  startMovement(walk_.larm);

  walk_.rarm.spline = walk_.larm.spline;
  // Flips right arm points on x axis.
  mat3 flip = mat3(glm::scale(vec3(-1, 1, 1)));
  for (vec3& point : walk_.rarm.spline.points_) {
    point = flip * point;
  }
  startMovement(walk_.rarm);
}

void Skelly::startMovement(auto& move) {
  float dur = move.dur * cycle_dur_;
  move.anim = Animation(move.spline, dur, move.loop);
}

vec3 Skelly::sampleMovement(Movement<vec3>& move) {
  if (move.anim) {
    float anim_t = getMoveT(move, cycle_t_);
    return move.anim->sample(anim_t);
  }
  return vec3(0);
}

float Skelly::sampleMovement(Movement<float>& move) {
  if (move.anim) {
    float anim_t = getMoveT(move, cycle_t_);
    return move.anim->sample(anim_t);
  }
  return 0;
}

void Skelly::updateCog(Time now, float delta_s) {
  vec3 pos = vec3(0, options_.crouch_pct * sizes_.pelvis_y, 0);
  pos.y += sampleMovement(walk_.bounce);

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

  vec3 offset = root_.toLocal() * vec4(lean, 0);
  pos += offset;

  // Rotate COG based on lean amount.
  glm::quat rot = glm::rotation(
      glm::normalize(vec3(0, 1, 0)),
      glm::normalize(vec3(offset.x, sizes_.pelvis_y, offset.z)));

  Transform& cog_t = pose_.getBone(BoneId::Cog);
  cog_t.setPos(pos);
  cog_t.setRot(rot);
}

void Skelly::updatePelvis(Time now) {
  float sway = sampleMovement(walk_.sway);
  float spin = sampleMovement(walk_.spin);

  glm::quat rot = glm::angleAxis(spin, vec3(0, 1, 0)) *
                  glm::angleAxis(sway, vec3(0, 0, -1));

  pose_.getBone(BoneId::Pelvis).setRot(rot);
}

void Skelly::updateFeet(Time now) {
  updateToeAngle(now, lfoot_m_, walk_.lheel);
  updateToeAngle(now, rfoot_m_, walk_.rheel);
  updateToe(rfoot_m_, now, walk_.rstep);
  updateToe(lfoot_m_, now, walk_.lstep);

  mat4 to_root = pose_.getBone(BoneId::Cog).matrix() *
                 pose_.getBone(BoneId::Pelvis).matrix();
  vec3 lhip_pos = to_root * vec4(skeleton_.lfemur_->getPos(), 1);
  vec3 rhip_pos = to_root * vec4(skeleton_.rfemur_->getPos(), 1);

  updateAnkle(lhip_pos, lfoot_m_);
  updateAnkle(rhip_pos, rfoot_m_);
}

void Skelly::updateToeAngle(Time now, FootMeta& foot_m, Movement<float>& move) {
  if (moveStarted(move)) {
    move.spline = Spline<float>(
        SplineType::Hermite, {foot_m.toe_angle, foot_m.toe_angle * 3, 0, 0});
    startMovement(move);
  }

  if (move.anim && inCycle(move, cycle_t_)) {
    foot_m.toe_angle = sampleMovement(move);
  } else {
    foot_m.toe_angle = 0;
  }
}

void Skelly::updateToe(FootMeta& foot_m, Time now, Movement<vec3>& move) {
  if (moveStarted(move)) {
    swingFoot(foot_m, now, move);
  } else if (moveStopped(move) && move.anim) {
    vec3 plant_pos = move.anim->sample(1);
    foot_m.toe_pos = plant_pos;
    plantFoot(foot_m);
  }

  if (move.anim && inCycle(move, cycle_t_)) {
    foot_m.toe_pos = sampleMovement(move);
  } else {
    foot_m.toe_pos = root_.posToLocal(foot_m.world_target);
  }
}

void Skelly::initFoot(FootMeta& foot_m, Object* foot, Object* toe) {
  foot_m.foot = foot;
  foot_m.toe = toe;
  foot_m.toe_pos = foot_m.toe->posToAncestor(&root_);
  foot_m.start_pos = foot_m.toe_pos;
  plantFoot(foot_m);
}

void Skelly::plantFoot(FootMeta& foot_m) {
  foot_m.planted = true;
  foot_m.world_target = root_.posToWorld(foot_m.toe_pos);
}

void Skelly::swingFoot(FootMeta& foot_m, Time now, Movement<vec3>& move) {
  foot_m.planted = false;

  vec3 start = foot_m.toe_pos;

  float forward = std::min(sizes_.leg * 0.2f, target_speed_ / 3);
  vec3 step_offset =
      foot_m.start_pos + vec3(0, 0, options_.step_offset);  // TODO
  step_offset.x =
      (foot_m.is_left ? -1 : 1) * options_.stance_w_pct * sizes_.pelvis_w / 2;
  vec3 end = vec3(0, 0, forward) + step_offset;

  vec3 mid_pos = (start + end) / 2.f + vec3(0, options_.step_height, 0);

  vec3 path = end - start;
  vec3 swing_vel = 1.25f * path / (move.dur * cycle_dur_ / 1000);
  vec3 no_vel = vec3(0, 0, -target_speed_);
  vec3 toe_drop = vec3(0, -options_.step_height / 2, -target_speed_);

  move.spline = Spline<vec3>(
      SplineType::Hermite, {start, no_vel, mid_pos, swing_vel, end, toe_drop});
  startMovement(move);
}

void Skelly::updateAnkle(const vec3& hip_pos, FootMeta& foot_m) {
  // If hip is too far from ankle, compute the angle to lift the ankle just
  // enough to compensate.
  auto point_foot = glm::rotation(foot_m.toe_dir_start, foot_m.toe_dir);
  vec3 flat_ankle_pos = foot_m.toe_pos + point_foot * sizes_.ankle;
  float hip_to_ankle = glm::length(hip_pos - flat_ankle_pos);
  float max_leg = 0.98 * (sizes_.ankle_d);
  if (foot_m.planted && hip_to_ankle > max_leg) {
    float hip_to_toe = glm::length(foot_m.toe_pos - hip_pos);
    float toe_to_ankle = glm::length(sizes_.ankle);
    float flat_angle = cosineLaw(toe_to_ankle, hip_to_toe, hip_to_ankle);
    if (hip_to_toe > max_leg + toe_to_ankle) {
      // Target for toe is too far away. Point toes at target.
      foot_m.toe_angle = flat_angle;
    } else {
      float lift_angle = cosineLaw(toe_to_ankle, hip_to_toe, max_leg);
      foot_m.toe_angle = flat_angle - lift_angle;
    }
  }

  glm::quat ankle_rot =
      point_foot * glm::angleAxis(foot_m.toe_angle, vec3(1, 0, 0));

  Transform& foot_t =
      pose_.getBone(foot_m.is_left ? BoneId::Lfoot : BoneId::Rfoot);
  foot_t.setPos(foot_m.toe_pos + ankle_rot * sizes_.ankle);
  foot_t.setRot(ankle_rot);

  // TODO: this currently isn't possible with heel angle determined by IK. But
  // this will become relevant again if I add an animation to land on the
  // heel.
  // if (foot_m.toe_angle < 0) {
  //   // Lift up foot if it would go through the ground.
  //   float above = foot_m.toe_pos.y;
  //   float heel_y =
  //       glm::rotate(vec2(-sizes_.foot_l, 0),
  //       glm::radians(foot_m.toe_angle)).y;
  //   float lift = std::max(heel_y - above, 0.f);
  //   foot_m.foot->setPos(foot_m.foot->getPos() + vec3(0, lift, 0));
  //   // TODO: Move foot back so heel doesn't slide forward
  // }
}

void Skelly::updateShoulders(Time now) {
  float angle = sampleMovement(walk_.shoulders);
  pose_.getBone(BoneId::Neck).setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
}

void Skelly::updateHands(Time now) {
  // This animation is in torso space, but the hands are in root space.
  mat4 to_root = pose_.getBone(BoneId::Cog).matrix() *
                 pose_.getBone(BoneId::Neck).matrix();

  if (walk_.larm.anim) {
    vec3 root_hand = sampleMovement(walk_.larm);
    pose_.getBone(BoneId::Lhand).setPos(to_root * vec4(root_hand, 1));
  }
  if (walk_.rarm.anim) {
    vec3 root_hand = sampleMovement(walk_.rarm);
    pose_.getBone(BoneId::Rhand).setPos(to_root * vec4(root_hand, 1));
  }
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
    ImGui::SliderFloat("Step Offset", &options_.step_offset, -50, 50);
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

  if (ImGui::BeginTabItem("Mods")) {
    ImGui::SliderFloat("Mod Blend %", &options_.mod_blend, 0, 1);
    ImGui::EndTabItem();
  }

  ImGui::EndTabBar();
  ImGui::End();
}

void Skelly::cycleUi(Cycle& cycle) {
  movementUi("lstep", cycle.lstep);
  movementUi("rstep", cycle.rstep);
  movementUi("lheel", cycle.lheel);
  movementUi("rheel", cycle.rheel);
  movementUi("bounce", cycle.bounce);
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
  float end = remapRangeClamped(t2, 0, 1, p0.x, p1.x);
  ImVec2 p3 = ImVec2(end, p1.y);
  draw_list->AddRectFilled(p2, p3, IM_COL32_WHITE);  // first chunk
  if (t2 > 1) {
    float t2_wrapped = t2 - 1;
    float wrapped_end = remapRangeClamped(t2_wrapped, 0, 1, p0.x, p1.x);
    ImVec2 p4 = ImVec2(wrapped_end, p1.y);
    draw_list->AddRectFilled(p0, p4, IM_COL32_WHITE);  // wrapped chunk
  }
  ImGui::PushID("bar");
  ImGui::InvisibleButton("bar", bar_size);  // advances cursor past the rect
  ImGui::PopID();                           // bar

  ImGui::PopID();  // label
}
