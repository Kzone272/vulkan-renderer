#include "skelly.h"

#include <imgui/imgui.h>

#include <memory>
#include <print>

#include "animation.h"
#include "imgui-helpers.h"
#include "maths.h"
#include "skelly-presets.h"
#include "time-utils.h"
#include "world-tree.h"

namespace {

bool inCycle(const auto& move, float t) {
  if (t < move.offset) {
    t += 1;
    DASSERT(t >= move.offset);
  }
  float end = move.offset + move.dur;
  return t <= end;
}

float getMoveT(const auto& move, float cycle_t) {
  if (cycle_t < move.offset) {
    cycle_t += 1;
    DASSERT(cycle_t >= move.offset);
  }
  float end = move.offset + move.dur;
  float move_t = remapRange(cycle_t, move.offset, end, 0, 1);
  if (move.loop) {
    move_t = fmodClamp(move_t, 1);
  }
  return move_t;
}

bool moveStarted(auto& move, float cycle_t, float prev_cycle_t) {
  return !inCycle(move, prev_cycle_t) && inCycle(move, cycle_t);
}

bool moveStopped(auto& move, float cycle_t, float prev_cycle_t) {
  return inCycle(move, prev_cycle_t) && !inCycle(move, cycle_t);
}

vec3 sampleMovement(Movement<vec3>& move, float cycle_t) {
  if (move.anim) {
    float anim_t = getMoveT(move, cycle_t);
    return move.anim->sample(anim_t);
  }
  return vec3(0);
}

float sampleMovement(Movement<float>& move, float cycle_t) {
  if (move.anim) {
    float anim_t = getMoveT(move, cycle_t);
    return move.anim->sample(anim_t);
  }
  return 0;
}

}  // namespace

template <class T>
void Movement<T>::start(float cycle_dur) {
  anim = Animation<T>(spline, dur * cycle_dur, loop);
}

void Duration::update(float delta_s) {
  t_ += delta_s / dur_s_;
  if (t_ > 1) {
    t_ = 1;
  }
}

Skelly::Skelly(WorldTree* world) : world_(world), root_(world_) {
  world_->addChild(&root_);
  makeBones();
  lean_so_ = std::make_unique<SecondOrder<vec3>>(mods_.lean_params, vec3(0));
}

void Skelly::makeBones() {
  sizes_.pelvis_y = sizes_.leg + sizes_.pelvis_h;
  sizes_.shoulders_y =
      sizes_.height - sizes_.pelvis_y - sizes_.neck - sizes_.head_h;
  sizes_.ankle_d = sizes_.leg - sizes_.ankle.y;
  sizes_.femur = sizes_.femur_pct * sizes_.ankle_d;
  sizes_.shin = sizes_.ankle_d - sizes_.femur;
  sizes_.max_leg = sizes_.femur + cos(sizes_.min_knee_angle) * sizes_.shin;
  sizes_.normal_pelvis_y = sizes_.max_leg + sizes_.pelvis_h;
  sizes_.wrist_d = sizes_.arm - sizes_.hand_l;
  sizes_.bicep = sizes_.bicep_pct * sizes_.wrist_d;
  sizes_.forearm = sizes_.wrist_d - sizes_.bicep;
  sizes_.sho_pos = vec3(-(sizes_.shoulders_w / 2 + 3), -5, 0);
  sizes_.hip_pos = vec3(-(sizes_.pelvis_w / 2 + 3), -sizes_.pelvis_h, 0);

  root_.clearChildren();
  skeleton_.makeBones(sizes_);
  rig_.makeRig(skeleton_.getZeroPose());
  rig_skl_ = rig_.skeleton();
  idle_ = {rig_, idle_walk_, sizes_, walk_cycle_, 1000};
  move_cycles_ = {WalkPoser(idle_, rig_skl_, mods_, &root_)};
  move_transition_.reset();

  hand_pose_ = {rig_skl_};
  hand_pose_.bone_mask = {
      BipedRig::Id::Lhand, BipedRig::Id::Rhand, BipedRig::Id::Lelbow,
      BipedRig::Id::Relbow};
  lean_pose_ = {rig_skl_, PoseType::Additive};
  lean_pose_.bone_mask = {BipedRig::Id::Cog};
}

void Skelly::setMaterials(MaterialId bone_mat, MaterialId control_mat) {
  skeleton_.setMaterial(bone_mat);
  rig_.setMaterial(control_mat);
}

void Skelly::getSceneObjects(
    const mat4& parent, std::vector<SceneObject>& objs,
    const std::set<ModelId>& hidden) {
  mat4 root = root_.matrix();
  rig_.getSceneObjects(root, objs, hidden);
  skeleton_.getSceneObjects(root, objs, hidden);
}

void Skelly::handleInput(const InputState& input) {
  // Check if we should update velocity because max speed slider changed.
  // TODO: Use slider update directly.
  bool max_speed_changed =
      target_speed_ > 0.1f && std::abs(target_speed_ - move_.max_speed) > 0.1f;

  target_speed_changed_ = false;
  if (glm::length(input.move.dir - input_dir_) > 0.1f || max_speed_changed) {
    input_dir_ = input.move.dir;
    vec3 target_vel = move_.max_speed * vec3(input_dir_.x, 0, input_dir_.y);

    float speed = glm::length(target_vel);
    if (std::abs(speed - target_speed_) > 0.1f) {
      target_speed_ = speed;
      target_speed_changed_ = true;
    }

    if (move_.adjust_time == 0) {
      vel_ = target_vel;
    } else {
      float acc_force = move_.max_speed / move_.adjust_time;
      float adjust_time =
          move_.max_speed == 0 ? 0 : glm::length(target_vel - vel_) / acc_force;
      adjust_time = std::clamp(adjust_time, 200.f, move_.adjust_time);

      vec3 acc(0);
      vec3 curr_vel = vel_;
      if (vel_curve_) {
        curr_vel = vel_curve_->sample(vel_dur_.t());
        acc = vel_curve_->sample(vel_dur_.t(), SampleType::Velocity) /
              vel_curve_->dur_ms_ * 1000.f;
      }
      auto spline = Spline<vec3>(
          SplineType::Hermite, {curr_vel, acc, target_vel, vec3(0)});
      vel_curve_ = Animation<vec3>(spline, adjust_time);
      vel_dur_ = {adjust_time / 1000};
    }
  }
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

void Skelly::update(float delta_s) {
  updateSpeed(delta_s);
  updateCycle(delta_s);

  if (move_transition_) {
    DASSERT(move_cycles_.size() == 2);
    move_transition_->update(delta_s);
    float t = move_transition_->t();

    auto p1 = pose_ = move_cycles_[0].getPose(cycle_t_, delta_s);
    auto p2 = pose_ = move_cycles_[1].getPose(cycle_t_, delta_s);
    pose_ = Pose::blend(p1, p2, t);

    cycle_dur_ = glm::lerp(
        move_cycles_[0].getCycleDur(), move_cycles_[1].getCycleDur(), t);

    if (t >= 1) {
      move_transition_.reset();
      move_cycles_.erase(move_cycles_.begin());
    }
  } else {
    DASSERT(move_cycles_.size() == 1);
    pose_ = move_cycles_[0].getPose(cycle_t_, delta_s);
  }

  updateLean(delta_s);
  pose_ = Pose::blend(pose_, lean_pose_, 1);

  updateHandPose(pose_);
  pose_ = Pose::blend(pose_, hand_pose_, mods_.hand_blend);

  rig_.applyPose(pose_);
  rig_.updateSkeleton(skeleton_);
}

void Skelly::updateCycle(float delta_s) {
  // TODO: Don't update cycle when standing?
  // Maybe it should start once we take the first step, then stop once
  // velocity reaches zero?
  // if ((false) && glm::length(vel_) < 0.1f) {
  //   cycle_t_ = 0;
  // }

  prev_cycle_t_ = cycle_t_;
  cycle_t_ = fmod(cycle_t_ + delta_s / (cycle_dur_ / 1000.f), 1.f);

  if (target_speed_changed_ || move_cycle_changed_) {
    updateMoveCycle();
  }
}

void Skelly::updateMoveCycle() {
  if (move_transition_) {
    DASSERT(move_cycles_.size() == 2);
    // Delete the least relevant pose in the current transition.
    if (move_transition_->t() > 0.5) {
      move_cycles_.erase(move_cycles_.begin());
    } else {
      move_cycles_.pop_back();
    }
  }

  if (target_speed_ != 0) {
    float step_l = sizes_.leg * 0.76f;
    float step_dur = (step_l / target_speed_) * 1000.f;
    float cycle_dur = std::clamp(2 * step_dur, 500.f, 1200.f);

    if (target_speed_ > 350) {
      move_cycles_.push_back(WalkPoser(
          WalkCycle(rig_, run_, sizes_, run_cycle_, cycle_dur), rig_skl_, mods_,
          &root_));
    } else {
      move_cycles_.push_back(WalkPoser(
          WalkCycle(rig_, walk_, sizes_, walk_cycle_, cycle_dur), rig_skl_,
          mods_, &root_));
    }
  } else {
    move_cycles_.push_back(WalkPoser(idle_, rig_skl_, mods_, &root_));
  }
  move_transition_ = Duration(move_.blend_time);
}

void Skelly::updateSpeed(float delta_s) {
  vec3 curr_vel = vel_;

  if (vel_curve_) {
    vel_dur_.update(delta_s);
    vel_ = vel_curve_->sample(vel_dur_.t());
    if (vel_dur_.t() > 1) {
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
    float change = delta_s * glm::radians(move_.max_rot_speed);
    if (std::abs(delta) > change) {
      float dir = (delta > 0) ? 1 : -1;
      angle = current + dir * change;
    }
    angle = fmodClamp(angle, glm::radians(360.f));

    root_.setRot(glm::angleAxis(angle, vec3(0, 1, 0)));
  }
}

void Skelly::updateLean(float delta_s) {
  vec3 lean_target(0);
  if (vel_curve_) {
    vec3 acc = vel_curve_->sample(vel_dur_.t(), SampleType::Velocity) /
               vel_curve_->dur_ms_;
    lean_target = mods_.lean * acc;
  }
  vec3 lean;
  if (delta_s == 0) {
    lean = lean_target;
  } else {
    lean = lean_so_->update(delta_s, lean_target);
  }

  vec3 offset = root_.toLocal() * vec4(lean, 0);
  offset.y += (mods_.crouch_pct - 1) * sizes_.pelvis_y;

  // Rotate COG based on lean amount.
  glm::quat rot = glm::rotation(
      glm::normalize(vec3(0, 1, 0)),
      glm::normalize(vec3(offset.x, sizes_.pelvis_y, offset.z)));

  lean_pose_.setPos(BipedRig::Id::Cog, offset);
  lean_pose_.setRot(BipedRig::Id::Cog, rot);
}

void Skelly::updateHandPose(Pose& pose) {
  if (mods_.hand_blend == 0) {
    return;
  }

  mat4 hip_to_hand = glm::inverse(pose.getMatrix(BipedRig::Id::Neck)) *
                     pose.getMatrix(BipedRig::Id::Pelvis);
  vec3 akimbo = sizes_.hip_pos + vec3(-5, 5, 0);
  vec3 lhip = hip_to_hand * vec4(akimbo, 1);
  vec3 rhip = hip_to_hand * vec4(vec3(-1, 1, 1) * akimbo, 1);
  hand_pose_.setPos(BipedRig::Id::Lhand, lhip);
  hand_pose_.setPos(BipedRig::Id::Rhand, rhip);
  hand_pose_.setPos(BipedRig::Id::Lelbow, lhip + vec3(-100, 0, -100));
  hand_pose_.setPos(BipedRig::Id::Relbow, rhip + vec3(100, 0, -100));
}

WalkPoser::WalkPoser(
    WalkCycle walk, Skeleton* skl, const MoveMods& mods, Object* root)
    : walk_(walk), mods_(&mods), root_(root) {
  add_pose_ = {skl, PoseType::Additive};
  add_pose_.bone_mask = {BipedRig::Id::Lankle, BipedRig::Id::Rankle};
}

Pose WalkPoser::getPose(float cycle_t, float delta_s) {
  Pose pose = walk_.getPose(cycle_t);
  if (!world_set_) {
    setWorld(walk_.getLfoot());
    setWorld(walk_.getRfoot());
    world_set_ = true;
  }

  if (walk_.getTargetSpeed() != 0 && mods_->plant_feet) {
    plantFoot(walk_.getLfoot());
    plantFoot(walk_.getRfoot());
    offsetFoot(cycle_t, walk_.getLfoot());
    offsetFoot(cycle_t, walk_.getRfoot());
    pose = Pose::blend(pose, add_pose_, 1);
  }

  return pose;
}

void WalkPoser::setWorld(FootMeta& foot_m) {
  foot_m.world_target = root_->posToWorld(foot_m.toe_pos);
}

void WalkPoser::plantFoot(FootMeta& foot_m) {
  if (foot_m.just_landed) {
    setWorld(foot_m);
    return;
  }
  if (foot_m.planted) {
    vec3 root_pos = root_->posToLocal(foot_m.world_target);
    auto ankle_bone =
        foot_m.is_left ? BipedRig::Id::Lankle : BipedRig::Id::Rankle;
    add_pose_.setPos(ankle_bone, root_pos - foot_m.toe_pos);
  }
}

void WalkPoser::offsetFoot(float cycle_t, FootMeta& foot_m) {
  auto& move = foot_m.is_left ? lstep_offset_ : rstep_offset_;
  if (foot_m.just_lifted) {
    vec3 root_pos = root_->posToLocal(foot_m.world_target);
    vec3 delta = root_pos - foot_m.liftoff;
    move.offset = foot_m.step_offset;
    move.dur = foot_m.step_dur;
    move.spline = Spline<vec3>(SplineType::Linear, {delta, vec3(0)});
    move.start(walk_.getCycleDur());
  }

  if (!foot_m.planted) {
    auto bone = foot_m.is_left ? BipedRig::Id::Lankle : BipedRig::Id::Rankle;
    add_pose_.setPos(bone, sampleMovement(move, cycle_t));
  }
}

WalkCycle::WalkCycle(
    BipedRig& rig, const WalkOptions& walk, const SkellySizes& sizes,
    const Cycle& cycle, float cycle_dur) {
  sizes_ = &sizes;
  lfoot_m_.planted = true;
  rfoot_m_.planted = true;
  pose_ = rig.getZeroPose();
  cycle_dur_ = cycle_dur;
  updateCycle(walk, cycle);
}

void WalkCycle::updateCycle(const WalkOptions& walk, const Cycle& cycle) {
  walk_ = walk;
  cycle_ = cycle;

  float bounce = std::pow(cycle_dur_ / 1000, 2) * walk_.bounce;
  cycle_.bounce.spline =
      Spline<float>(SplineType::Hermite, {-bounce, 0, bounce, 0, -bounce, 0});
  cycle_.bounce.start(cycle_dur_);

  float hip_sway = glm::radians(walk_.hip_sway);
  cycle_.sway.spline = Spline<float>(
      SplineType::Hermite, {-hip_sway, 0, hip_sway, 0, -hip_sway, 0});
  cycle_.sway.start(cycle_dur_);

  float hip_spin = glm::radians(walk_.hip_spin);
  cycle_.spin.spline = Spline<float>(
      SplineType::Hermite, {-hip_spin, 0, hip_spin, 0, -hip_spin, 0});
  cycle_.spin.start(cycle_dur_);

  float shoulder_rot = glm::radians(walk_.shoulder_spin);
  cycle_.shoulders.spline = Spline<float>(
      SplineType::Hermite,
      {shoulder_rot, 0, -shoulder_rot, 0, shoulder_rot, 0});
  cycle_.shoulders.start(cycle_dur_);

  float hand_dist = walk_.hand_height_pct * sizes_->wrist_d;
  float hand_width = walk_.arm_span_pct * sizes_->wrist_d;
  vec3 back =
      vec3(-hand_width, -hand_dist, -0.2 * hand_dist + walk_.hands_forward) +
      sizes_->sho_pos;
  vec3 forward =
      vec3(-hand_width, -hand_dist + 5, 0.4 * hand_dist + walk_.hands_forward) +
      sizes_->sho_pos;
  cycle_.larm.spline = Spline<vec3>(
      SplineType::Hermite, {back, vec3(0), forward, vec3(0), back, vec3(0)});
  cycle_.larm.start(cycle_dur_);

  cycle_.rarm.spline = cycle_.larm.spline;
  // Flips right arm points on x axis.
  mat3 flip = mat3(glm::scale(vec3(-1, 1, 1)));
  for (vec3& point : cycle_.rarm.spline.points_) {
    point = flip * point;
  }
  cycle_.rarm.start(cycle_dur_);

  updateStep(lfoot_m_);
  updateStep(rfoot_m_);
}

const Pose& WalkCycle::getPose(float cycle_t) {
  prev_cycle_t_ = cycle_t_;
  cycle_t_ = cycle_t;

  if (cycle_dur_ == 0) {
    return pose_;
  }

  updateCog();
  updatePelvis();
  updateFeet();
  updateShoulders();
  updateHands();
  return pose_;
}

void WalkCycle::updateCog() {
  vec3 pos = vec3(0, sizes_->normal_pelvis_y, 0);
  pos.y += sampleMovement(cycle_.bounce, cycle_t_);

  pose_.setPos(BipedRig::Id::Cog, pos);
}

void WalkCycle::updatePelvis() {
  float sway = sampleMovement(cycle_.sway, cycle_t_);
  float spin = sampleMovement(cycle_.spin, cycle_t_);

  glm::quat rot = glm::angleAxis(spin, vec3(0, 1, 0)) *
                  glm::angleAxis(sway, vec3(0, 0, -1));

  pose_.setRot(BipedRig::Id::Pelvis, rot);
}

void WalkCycle::updateFeet() {
  updateToe(rfoot_m_);
  updateToe(lfoot_m_);
  updateAnkle(lfoot_m_);
  updateAnkle(rfoot_m_);
}

void WalkCycle::updateToe(FootMeta& foot_m) {
  auto& step = foot_m.is_left ? cycle_.lstep : cycle_.rstep;
  auto& slide = foot_m.is_left ? cycle_.lslide : cycle_.rslide;

  foot_m.just_lifted = moveStarted(step, cycle_t_, prev_cycle_t_);
  foot_m.just_landed = moveStopped(step, cycle_t_, prev_cycle_t_);
  if (foot_m.just_lifted) {
    foot_m.planted = false;
  } else if (foot_m.just_landed && step.anim) {
    foot_m.toe_pos = step.anim->sample(1);
    foot_m.planted = true;
  }

  if (step.anim && inCycle(step, cycle_t_)) {
    foot_m.toe_pos = sampleMovement(step, cycle_t_);
  } else {
    foot_m.toe_pos = sampleMovement(slide, cycle_t_);
  }
}

void WalkCycle::updateStep(FootMeta& foot_m) {
  auto& step = foot_m.is_left ? cycle_.lstep : cycle_.rstep;
  auto& slide = foot_m.is_left ? cycle_.lslide : cycle_.rslide;
  auto& ball = foot_m.is_left ? cycle_.lball : cycle_.rball;
  auto& toe = foot_m.is_left ? cycle_.ltoe : cycle_.rtoe;
  auto& heel = foot_m.is_left ? cycle_.lheel : cycle_.rheel;
  auto& drop = foot_m.is_left ? cycle_.ldrop : cycle_.rdrop;

  slide.offset = std::fmod(step.offset + step.dur, 1);
  slide.dur = 1 - step.dur;
  float dur_s = cycle_dur_ / 1000 * slide.dur;
  float stride = dur_s * walk_.speed;

  vec3 step_offset = vec3(0, 0, walk_.step_offset);
  step_offset.x =
      (foot_m.is_left ? -1 : 1) * walk_.stance_w_pct * sizes_->pelvis_w / 2;
  vec3 contact = vec3(0, 0, stride / 2) + step_offset;

  vec3 backward = {0, 0, -1};  // Move backwards in root space.
  vec3 liftoff = contact + (stride * backward);

  vec3 mid_pos = (contact + liftoff) / 2.f + vec3(0, walk_.step_height, 0);

  vec3 path = contact - liftoff;
  float step_dur_s = cycle_dur_ / 1000 * step.dur;
  vec3 swing_vel = 1.25f * path / step_dur_s;
  vec3 no_vel = vec3(0, 0, -walk_.speed);
  vec3 toe_drop = no_vel + vec3(0, -1.5 * walk_.step_height, 0);

  step.spline = Spline<vec3>(
      SplineType::Hermite,
      {liftoff, no_vel, mid_pos, swing_vel, contact, toe_drop});
  step.start(cycle_dur_);

  slide.spline = Spline<vec3>(SplineType::Linear, {contact, liftoff});
  slide.start(cycle_dur_);

  foot_m.contact = contact;
  foot_m.liftoff = liftoff;
  foot_m.step_offset = step.offset;
  foot_m.step_dur = step.dur;

  float min_knee_angle = glm::radians(5.f);
  vec2 hip_pos = vec2(0, sizes_->normal_pelvis_y + sizes_->hip_pos.y);
  float leg_vertical = hip_pos.y - sizes_->ankle.y;
  float flat_ankle_z = -sqrt(pow(sizes_->max_leg, 2) - pow(leg_vertical, 2));
  float flat_toe_z = flat_ankle_z + sizes_->toe.z + walk_.step_offset;

  float slide_end = slide.offset + slide.dur;
  float flat_t =
      remapRange(flat_toe_z, liftoff.z, contact.z, slide.offset, slide_end);
  float heel_lift_dur = slide_end - flat_t;
  flat_t = fmodClamp(flat_t, 1);

  vec2 end_toe = vec2(liftoff.z, 0);
  vec2 end_ankle = end_toe + sizes_->ankle.zy;
  float toe_to_ankle = glm::length(sizes_->ankle);
  float hip_to_ankle = glm::length(hip_pos - end_ankle);

  float lift_angle = 0;
  if (hip_to_ankle > sizes_->max_leg + toe_to_ankle) {
    // Target for toe is too far away. Use some max value.
    std::println("Steps are too long!");
    lift_angle = glm::radians(80.f);
  } else {
    float hip_to_toe = glm::length(end_toe - hip_pos);
    float flat_angle = cosineLaw(toe_to_ankle, hip_to_toe, hip_to_ankle);
    float max_leg_angle = cosineLaw(toe_to_ankle, hip_to_toe, sizes_->max_leg);
    // Adding 10 degrees looks good, and hopefully prevents some knee locking.
    lift_angle = flat_angle - max_leg_angle + glm::radians(10.f);
    lift_angle = std::clamp(lift_angle, 0.f, glm::radians(80.f));
  }

  ball.spline = Spline<float>(SplineType::Linear, {0, 0.66f * lift_angle, 0});
  ball.offset = flat_t;
  ball.dur = heel_lift_dur;
  ball.start(cycle_dur_);

  toe.spline = Spline<float>(SplineType::Linear, {0, lift_angle});
  float toe_dur = 0.5 * heel_lift_dur;
  toe.offset = fmodClamp(flat_t + toe_dur, 1);
  toe.dur = toe_dur;
  toe.start(cycle_dur_);

  float contact_angle = -std::min(glm::radians(30.f), 0.5f * lift_angle);
  heel.spline = Spline<float>(
      SplineType::Hermite, {lift_angle, lift_angle / 2, contact_angle, 0});
  heel.offset = step.offset;
  heel.dur = step.dur;
  heel.start(cycle_dur_);

  drop.spline = Spline<float>(SplineType::Linear, {contact_angle, 0});
  drop.offset = slide.offset;
  drop.dur = 0.1;
  drop.start(cycle_dur_);
}

void WalkCycle::updateAnkle(FootMeta& foot_m) {
  auto& ball_anim = foot_m.is_left ? cycle_.lball : cycle_.rball;
  auto& toe_anim = foot_m.is_left ? cycle_.ltoe : cycle_.rtoe;
  auto& heel_anim = foot_m.is_left ? cycle_.lheel : cycle_.rheel;
  auto& drop_anim = foot_m.is_left ? cycle_.ldrop : cycle_.rdrop;

  float ball_angle = 0;
  if (inCycle(ball_anim, cycle_t_)) {
    ball_angle = sampleMovement(ball_anim, cycle_t_);
  }

  float foot_angle = 0;
  if (inCycle(toe_anim, cycle_t_)) {
    foot_angle = sampleMovement(toe_anim, cycle_t_);
  }
  if (inCycle(heel_anim, cycle_t_)) {
    foot_angle = sampleMovement(heel_anim, cycle_t_);
  }
  if (inCycle(drop_anim, cycle_t_)) {
    foot_angle = sampleMovement(drop_anim, cycle_t_);
  }

  // Negative angles rotate from the heel, postive angles from the toe.
  float toe_angle = std::max(0.f, foot_angle);
  float heel_angle = std::min(0.f, foot_angle);
  glm::quat toe_rot = glm::angleAxis(toe_angle, vec3(1, 0, 0));
  glm::quat heel_rot = glm::angleAxis(heel_angle, vec3(1, 0, 0));

  vec2 ball_to_ankle = -1.f * sizes_->ball.zy;
  vec2 heel_to_ball = sizes_->ball.zy - sizes_->heel.zy;
  vec2 toe_to_heel = sizes_->heel.zy - sizes_->toe.zy;

  // Calculate the position of the ankle from the toe, given the hierarchy:
  //   ankle -> ball -> heel -> toe
  // Note: rotate() uses CCW angles. My animations use CW angles.
  vec2 ankle_2d = glm::rotate(ball_to_ankle, -ball_angle);
  ankle_2d = glm::rotate(ankle_2d + heel_to_ball, -heel_angle);
  ankle_2d = glm::rotate(ankle_2d + toe_to_heel, -toe_angle);

  glm::quat point_foot = glm::rotation(foot_m.toe_dir_start, foot_m.toe_dir);
  glm::quat ball_rot = glm::angleAxis(-ball_angle, vec3(1, 0, 0));
  glm::quat ankle_rot = glm::angleAxis(ball_angle + foot_angle, vec3(1, 0, 0));

  vec3 ankle = point_foot * vec3(0, ankle_2d.yx) + foot_m.toe_pos;

  BipedRig::Id ankle_bone =
      foot_m.is_left ? BipedRig::Id::Lankle : BipedRig::Id::Rankle;
  BipedRig::Id ball_bone =
      foot_m.is_left ? BipedRig::Id::Lball : BipedRig::Id::Rball;
  BipedRig::Id knee_bone =
      foot_m.is_left ? BipedRig::Id::Lknee : BipedRig::Id::Rknee;
  pose_.setPos(ankle_bone, ankle);
  pose_.setRot(ankle_bone, ankle_rot);
  pose_.setRot(ball_bone, ball_rot);
  pose_.setPos(knee_bone, ankle + vec3(0, 0, 200));
}

void WalkCycle::updateShoulders() {
  float angle = sampleMovement(cycle_.shoulders, cycle_t_);
  pose_.setRot(BipedRig::Id::Neck, glm::angleAxis(angle, vec3(0, 1, 0)));
}

void WalkCycle::updateHands() {
  if (cycle_.larm.anim) {
    vec3 pos = sampleMovement(cycle_.larm, cycle_t_);
    pose_.setPos(BipedRig::Id::Lhand, pos);
    pose_.setPos(BipedRig::Id::Lelbow, pos + vec3(0, 0, -100));
  }
  if (cycle_.rarm.anim) {
    vec3 pos = sampleMovement(cycle_.rarm, cycle_t_);
    pose_.setPos(BipedRig::Id::Rhand, pos);
    pose_.setPos(BipedRig::Id::Relbow, pos + vec3(0, 0, -100));
  }
}

bool WalkUi(WalkOptions& walk) {
  bool changed = false;
  changed |= ImGui::SliderFloat("Stance W %", &walk.stance_w_pct, 0, 2);
  changed |= ImGui::SliderFloat("Step Height", &walk.step_height, 0, 50);
  changed |= ImGui::SliderFloat("Step Offset", &walk.step_offset, -50, 50);
  changed |= ImGui::SliderFloat("Bounce", &walk.bounce, 0, 10);
  changed |= ImGui::SliderFloat("Hip Sway", &walk.hip_sway, 0, 30);
  changed |= ImGui::SliderFloat("Hip Spin", &walk.hip_spin, 0, 45);
  changed |= ImGui::SliderFloat("Shoulder Spin", &walk.shoulder_spin, 0, 45);
  changed |= ImGui::SliderFloat("Arm Span %", &walk.arm_span_pct, -0.2, 1);
  changed |= ImGui::SliderFloat("Hand Height %", &walk.hand_height_pct, -1, 1);
  changed |= ImGui::SliderFloat("Hands Forward", &walk.hands_forward, -50, 50);

  return changed;
}

void Skelly::UpdateImgui() {
  ImGui::Begin("Skeleton");

  if (ImGui::SliderFloat("Max Speed", &move_.max_speed, 0, 1000)) {
    walk_.speed = move_.max_speed;
    run_.speed = move_.max_speed;
  }
  ImGui::Separator();

  ImGui::BeginTabBar("Skeleton Tabs");
  if (ImGui::BeginTabItem("Move")) {
    ImGui::SliderFloat("Vel Adjust Time", &move_.adjust_time, 0, 1000);
    ImGui::SliderFloat("Max Rot Speed", &move_.max_rot_speed, 1, 360);
    ImGui::SliderFloat("Anim Blend Time", &move_.blend_time, 0, 1);

    ImGui::Separator();

    ImGui::Text("Mods");
    ImGui::SliderFloat("Hand Blend %", &mods_.hand_blend, 0, 1);
    SliderFloatDefault("Crouch %", &mods_.crouch_pct, 0, 1.2, 1);
    ImGui::Checkbox("Plant Feet", &mods_.plant_feet);
    ImGui::SliderFloat("Lean", &mods_.lean, 0, 200);
    if (ImGui::SliderFloat3(
            "Lean SO", (float*)&mods_.lean_params, 0.001, 20, "%.5f")) {
      lean_so_->updateParams(mods_.lean_params);
    }

    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Sizes")) {
    bool changed = false;
    if (ImGui::Combo(
            "Size Presets", &ui_.size_preset,
            "Default\0Tall\0Big\0Dwarf\0Chimp\0")) {
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
      changed = true;
    }
    ImGui::Separator();

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

  move_cycle_changed_ = false;
  if (ImGui::BeginTabItem("WalkOpt")) {
    if (ImGui::Combo(
            "Movement Presets", &ui_.move_preset,
            "Normal\0Tightrope\0Preppy\0Snow\0Runway\0Crouch\0Flanders\0")) {
      if (ui_.move_preset == 0) {
        moveDefault(walk_, mods_);
      } else if (ui_.move_preset == 1) {
        moveTightrope(walk_, mods_);
      } else if (ui_.move_preset == 2) {
        movePreppy(walk_, mods_), mods_;
      } else if (ui_.move_preset == 3) {
        moveSnow(walk_, mods_);
      } else if (ui_.move_preset == 4) {
        moveRunway(walk_, mods_);
      } else if (ui_.move_preset == 5) {
        moveCrouch(walk_, mods_);
      } else if (ui_.move_preset == 6) {
        moveFlanders(walk_, mods_);
      }
      move_.max_speed = walk_.speed;
      move_cycle_changed_ = true;
    }

    move_cycle_changed_ |= WalkUi(walk_);

    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("WalkCyc")) {
    move_cycle_changed_ |= cycleUi(walk_cycle_);
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("Run")) {
    move_cycle_changed_ |= WalkUi(run_);
    ImGui::EndTabItem();
  }

  if (ImGui::BeginTabItem("RunCyc")) {
    move_cycle_changed_ |= cycleUi(run_cycle_);
    ImGui::EndTabItem();
  }

  ImGui::EndTabBar();
  ImGui::End();
}

bool Skelly::cycleUi(Cycle& cycle) {
  bool changed = false;
  changed |= movementUi("lstep", cycle.lstep);
  changed |= movementUi("rstep", cycle.rstep);
  changed |= movementUi("lheel", cycle.lheel);
  changed |= movementUi("rheel", cycle.rheel);
  changed |= movementUi("bounce", cycle.bounce);
  if (ImGui::Button("Reset")) {
    walk_cycle_ = default_walk_;
    changed = true;
  }
  return changed;
}

bool Skelly::movementUi(const std::string& label, auto& move) {
  bool changed = false;
  ImGui::PushID(label.c_str());

  ImGui::Text("%s: Offset | Duration", label.c_str());
  ImGui::SameLine();
  ImGui::Checkbox("Loop", &move.loop);

  ImGui::PushID("offsetdur");
  float vals[2] = {move.offset, move.dur};
  if (ImGui::SliderFloat2("", vals, 0, 1)) {
    move.offset = vals[0];
    move.dur = vals[1];
    changed = true;
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
  return changed;
}
