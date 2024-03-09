#include "biped-rig.h"

#include "skelly.h"

namespace {

void initFoot(BipedRig& rig, Foot& foot, vec3 pos, bool is_left) {
  mat4 control_t = glm::scale(vec3(5));
  foot.obj = rig.root_.addChild(Object(ModelId::Control, control_t));
  foot.start_pos = pos;
  foot.obj->setPos(pos);
  foot.is_left = is_left;
  rig.plantFoot(foot);
}

}  // namespace

void BipedRig::makeBones(const SkellySizes& sizes) {
  root_.clearChildren();

  cog_ = root_.addChild(Object(ModelId::Control, glm::scale(vec3(5))));
  cog_->setPos(vec3(0, sizes.pelvis_y, 0));

  mat4 pelvis_t = glm::scale(vec3(sizes.pelvis_w, -sizes.pelvis_h, 15));
  pelvis_ = cog_->addChild(Object(ModelId::Bone, pelvis_t));
  pelvis_->setPos(vec3(0, 0, 0));

  mat4 torso_t = glm::scale(vec3(sizes.shoulders_w, -15, 15));
  torso_ = cog_->addChild(Object(ModelId::Bone, torso_t));
  torso_->setPos(vec3(0, sizes.shoulders_y, 0));

  vec3 bicep_pos = {-(sizes.shoulders_w / 2 + 3), -5, 0};
  mat4 arm_rot = glm::toMat4(glm::angleAxis(glm::radians(90.f), vec3(0, 0, 1)));
  mat4 bicep_t =
      glm::scale(vec3(sizes.bicep, sizes.bone_w, sizes.bone_w)) * arm_rot;
  lbicep_ = torso_->addChild(Object(ModelId::Bone, bicep_t));
  lbicep_->setPos(bicep_pos);

  vec3 forearm_pos = {-sizes.bicep, 0, 0};
  mat4 forearm_t =
      glm::scale(vec3(sizes.forearm, sizes.bone_w, sizes.bone_w)) * arm_rot;
  lforearm_ = lbicep_->addChild(Object(ModelId::Bone, forearm_t));
  lforearm_->setPos(forearm_pos);

  vec3 hand_pos = {-sizes.forearm, 0, 0};
  mat4 hand_t = glm::scale(vec3(sizes.hand_l, 7, 9)) * arm_rot;
  lhand_ = lforearm_->addChild(Object(ModelId::Bone, hand_t));
  lhand_->setPos(hand_pos);

  mat4 head_t =
      glm::translate(vec3(0, 0, 2)) * glm::scale(vec3(20, sizes.head_h, 20));
  head_ = torso_->addChild(Object(ModelId::Bone, head_t));
  head_->setPos(vec3(0, sizes.neck, 5));

  mat4 femur_t = glm::scale(vec3(sizes.bone_w, -sizes.femur, sizes.bone_w));
  lfemur_ = pelvis_->addChild(Object(ModelId::Bone, femur_t));
  vec3 femur_pos = {-(sizes.pelvis_w / 2 + 3), -sizes.pelvis_h, 0};
  lfemur_->setPos(femur_pos);

  mat4 shin_t = glm::scale(vec3(sizes.bone_w, -sizes.shin, sizes.bone_w));
  lshin_ = lfemur_->addChild(Object(ModelId::Bone, shin_t));
  vec3 shin_pos = vec3(0, -sizes.femur, 0);
  lshin_->setPos(shin_pos);

  mat4 foot_t = glm::translate(-sizes.ankle + vec3(-1, 0, 0)) *
                glm::scale(vec3(13, 4, sizes.foot_l)) *
                glm::translate(vec3(0, 0, -0.5));
  lfoot_ = lshin_->addChild(Object(ModelId::Bone, foot_t));
  vec3 foot_pos = vec3(0, -sizes.shin, 0);
  lfoot_->setPos(foot_pos);

  // Add opposite limbs with flipped positions.
  mat4 flip = glm::scale(vec3(-1, 1, 1));
  mat3 flip3 = mat3(flip);

  rfemur_ = pelvis_->addChild(Object(ModelId::Bone, femur_t));
  rfemur_->setPos(flip3 * femur_pos);

  rshin_ = rfemur_->addChild(Object(ModelId::Bone, shin_t));
  rshin_->setPos(flip3 * shin_pos);

  rfoot_ = rshin_->addChild(Object(ModelId::Bone, flip * foot_t));
  rfoot_->setPos(flip3 * foot_pos);

  rbicep_ = torso_->addChild(Object(ModelId::Bone, flip * bicep_t));
  rbicep_->setPos(flip3 * bicep_pos);

  rforearm_ = rbicep_->addChild(Object(ModelId::Bone, flip * forearm_t));
  rforearm_->setPos(flip3 * forearm_pos);

  rhand_ = rforearm_->addChild(Object(ModelId::Bone, flip * hand_t));
  rhand_->setPos(flip3 * hand_pos);

  // Add non-bone control objects
  vec3 left_foot_pos = {-sizes.pelvis_w / 2, 0, 12};
  initFoot(*this, lfoot_c_, left_foot_pos, /*=is_left*/ true);
  initFoot(*this, rfoot_c_, flip3 * left_foot_pos, /*=is_left*/ false);

  vec3 wrist_pos =
      lbicep_->posToAncestor(&root_, vec3(-sizes.wrist_d, 0, 0));
  mat4 control_t = glm::scale(vec3(5));
  lhand_c_.obj = root_.addChild(Object(ModelId::Control, control_t));
  lhand_c_.obj->setPos(wrist_pos);

  rhand_c_.obj = root_.addChild(Object(ModelId::Control, control_t));
  rhand_c_.obj->setPos(flip3 * wrist_pos);

  mat4 root_control_t = glm::scale(vec3(10, 1, 30));
  root_.addChild(Object(ModelId::Control, root_control_t));
}

void BipedRig::plantFoot(Foot& foot) {
  foot.planted = true;
  foot.in_swing = false;
  foot.world_target = root_.toWorld() * vec4(foot.obj->getPos(), 1);
  mat4 to_world = foot.obj->toWorld();
  foot.world_rot = glm::quat_cast(to_world);
}