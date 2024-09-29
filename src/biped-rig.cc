#include "biped-rig.h"

#include "skelly.h"

void BipedSkeleton::makeBones(const SkellySizes& sizes, Object* root) {
  root_ = root;

  femur_l_ = sizes.femur;
  shin_l_ = sizes.shin;
  bicep_l_ = sizes.bicep;
  forearm_l_ = sizes.forearm;
  toe_pos_ = sizes.toe;
  ball_pos_ = sizes.ball;
  heel_pos_ = sizes.heel;

  cog_ = root_->addChild(Object(ModelId::None, glm::scale(vec3(5))));
  cog_->setPos(vec3(0, sizes.pelvis_y, 0));

  mat4 pelvis_t = glm::scale(vec3(sizes.pelvis_w, -sizes.pelvis_h, 15));
  pelvis_ = cog_->addChild(Object(ModelId::Bone, pelvis_t));
  pelvis_->setPos(vec3(0, 0, 0));

  mat4 torso_t = glm::scale(vec3(sizes.shoulders_w, -15, 15));
  torso_ = cog_->addChild(Object(ModelId::Bone, torso_t));
  torso_->setPos(vec3(0, sizes.shoulders_y, 0));

  mat4 arm_rot = glm::toMat4(glm::angleAxis(glm::radians(90.f), vec3(0, 0, 1)));
  mat4 bicep_t =
      glm::scale(vec3(sizes.bicep, sizes.bone_w, sizes.bone_w)) * arm_rot;
  lbicep_ = torso_->addChild(Object(ModelId::Bone, bicep_t));
  lbicep_->setPos(sizes.sho_pos);

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
  lfemur_->setPos(sizes.hip_pos);

  mat4 shin_t = glm::scale(vec3(sizes.bone_w, -sizes.shin, sizes.bone_w));
  lshin_ = lfemur_->addChild(Object(ModelId::Bone, shin_t));
  vec3 shin_pos = vec3(0, -sizes.femur, 0);
  lshin_->setPos(shin_pos);

  mat4 foot_t = glm::translate(sizes.heel + vec3(-2, 0, 0)) *
                glm::scale(vec3(12, 4, sizes.foot_l - sizes.toes_l)) *
                glm::translate(vec3(0, 0, 0.5));
  lfoot_ = lshin_->addChild(Object(ModelId::Bone, foot_t));
  vec3 foot_pos = vec3(0, -sizes.shin, 0);
  lfoot_->setPos(foot_pos);

  mat4 toes_t = glm::translate(vec3(-2, 0, 0)) *
                glm::scale(vec3(12, 4, sizes.toes_l)) *
                glm::translate(vec3(0, 0, 0.5));
  ltoes_ = lfoot_->addChild(Object(ModelId::Bone, toes_t));
  vec3 toes_pos = sizes.ball;
  ltoes_->setPos(toes_pos);

  // Add opposite limbs with flipped positions.
  mat4 flip = glm::scale(vec3(-1, 1, 1));
  mat3 flip3 = mat3(flip);

  rfemur_ = pelvis_->addChild(Object(ModelId::Bone, femur_t));
  rfemur_->setPos(flip3 * sizes.hip_pos);

  rshin_ = rfemur_->addChild(Object(ModelId::Bone, shin_t));
  rshin_->setPos(flip3 * shin_pos);

  rfoot_ = rshin_->addChild(Object(ModelId::Bone, flip * foot_t));
  rfoot_->setPos(flip3 * foot_pos);

  rtoes_ = rfoot_->addChild(Object(ModelId::Bone, flip * toes_t));
  rtoes_->setPos(flip3 * toes_pos);

  rbicep_ = torso_->addChild(Object(ModelId::Bone, flip * bicep_t));
  rbicep_->setPos(flip3 * sizes.sho_pos);

  rforearm_ = rbicep_->addChild(Object(ModelId::Bone, flip * forearm_t));
  rforearm_->setPos(flip3 * forearm_pos);

  rhand_ = rforearm_->addChild(Object(ModelId::Bone, flip * hand_t));
  rhand_->setPos(flip3 * hand_pos);
}

void BipedSkeleton::setMaterial(MaterialId material) {
  root_->setMaterial(material);
  cog_->setMaterial(material);
  pelvis_->setMaterial(material);
  torso_->setMaterial(material);
  lbicep_->setMaterial(material);
  lforearm_->setMaterial(material);
  lhand_->setMaterial(material);
  rbicep_->setMaterial(material);
  rforearm_->setMaterial(material);
  rhand_->setMaterial(material);
  head_->setMaterial(material);
  lfemur_->setMaterial(material);
  lshin_->setMaterial(material);
  lfoot_->setMaterial(material);
  ltoes_->setMaterial(material);
  rfemur_->setMaterial(material);
  rshin_->setMaterial(material);
  rfoot_->setMaterial(material);
  rtoes_->setMaterial(material);
}

namespace {

// Returns pair of angles for bone1 and bone2.
std::pair<float, float> solveIk(float bone1, float bone2, float target) {
  if (target >= bone1 + bone2) {
    return {0.f, 0.f};
  }

  float b1 = -cosineLaw(bone1, target, bone2);
  float b2 = glm::radians(180.f) - cosineLaw(bone1, bone2, target);
  return {b1, b2};
}

// b1_pos, target, main_axis, and rot_axis all need to be in b1's parent space.
void solveTwoBoneIk(
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

}  // namespace

IkChain::IkChain(
    Object* start, Object* target, Object* b1, Object* b2, float b1_l,
    float b2_l, vec3 rot_axis)
    : start(start),
      target(target),
      lca(Object::lca(start, target)),
      b1(b1),
      b2(b2),
      b1_l(b1_l),
      b2_l(b2_l),
      rot_axis(rot_axis) {
  DASSERT(lca != nullptr);
  point_zero = glm::normalize(targetPos() - startPos());
}

vec3 IkChain::startPos() {
  return start->getPos();
}
vec3 IkChain::targetPos() {
  return start->getParent()->posToLocal(lca, target->posToAncestor(lca));
}

void IkChain::solve() {
  solveTwoBoneIk(
      *b1, b1_l, *b2, b2_l, startPos(), targetPos(), point_zero, rot_axis);
}

void BipedRig::updateSkeleton(BipedSkeleton& skl) {
  skl.cog_->setTransform(cog_->getTransform());
  skl.torso_->setTransform(neck_->getTransform());
  skl.head_->setTransform(head_->getTransform());
  skl.pelvis_->setTransform(pelvis_->getTransform());

  solveIk();

  skl.lfoot_->setRot(glm::identity<glm::quat>());
  auto l_flat = glm::quat_cast(skl.lfoot_->toLocal(root_));
  skl.lfoot_->setRot(ltoe_->getRot() * lball_->getRot() * l_flat);

  skl.ltoes_->setRot(glm::inverse(lball_->getRot()));

  skl.rfoot_->setRot(glm::identity<glm::quat>());
  auto r_flat = glm::quat_cast(skl.rfoot_->toLocal(root_));
  skl.rfoot_->setRot(rtoe_->getRot() * rball_->getRot() * r_flat);

  skl.rtoes_->setRot(glm::inverse(rball_->getRot()));
}

void BipedRig::makeRig(const BipedSkeleton& skeleton, Object* root) {
  root_ = root;

  mat4 control_t = glm::scale(vec3(5));
  cog_ = root_->addChild(Object(ModelId::BallControl, control_t));
  cog_->setPos(skeleton.cog_->getPos());

  neck_ = cog_->addChild(Object(ModelId::BallControl, control_t));
  neck_->setPos(skeleton.torso_->getPos());
  head_ = neck_->addChild(Object(ModelId::BallControl, control_t));
  head_->setPos(skeleton.head_->getPos());

  lsho_ = neck_->addChild(Object(ModelId::BallControl, control_t));
  lsho_->setPos(skeleton.lbicep_->getPos());
  rsho_ = neck_->addChild(Object(ModelId::BallControl, control_t));
  rsho_->setPos(skeleton.rbicep_->getPos());

  lhand_ = neck_->addChild(Object(ModelId::BallControl, control_t));
  lhand_->setPos(skeleton.lhand_->posToAncestor(skeleton.torso_));
  rhand_ = neck_->addChild(Object(ModelId::BallControl, control_t));
  rhand_->setPos(skeleton.rhand_->posToAncestor(skeleton.torso_));

  mat4 pelvis_t = glm::scale(vec3(15, 1, 15));
  pelvis_ = cog_->addChild(Object(ModelId::BoxControl, pelvis_t));
  cog_->setPos(skeleton.cog_->getPos());

  lhip_ = pelvis_->addChild(Object(ModelId::BallControl, control_t));
  lhip_->setPos(skeleton.lfemur_->getPos());
  rhip_ = pelvis_->addChild(Object(ModelId::BallControl, control_t));
  rhip_->setPos(skeleton.rfemur_->getPos());

  mat4 ball_t = glm::scale(vec3(3));
  ltoe_ = root_->addChild(Object(ModelId::BallControl, ball_t));
  ltoe_->setPos(skeleton.lfoot_->posToAncestor(root_, skeleton.toe_pos_));
  rtoe_ = root_->addChild(Object(ModelId::BallControl, ball_t));
  rtoe_->setPos(skeleton.rfoot_->posToAncestor(root_, skeleton.toe_pos_));

  lheel_ = ltoe_->addChild(Object(ModelId::BallControl, ball_t));
  lheel_->setPos(skeleton.heel_pos_ - skeleton.toe_pos_);
  rheel_ = rtoe_->addChild(Object(ModelId::BallControl, ball_t));
  rheel_->setPos(skeleton.heel_pos_ - skeleton.toe_pos_);

  lball_ = lheel_->addChild(Object(ModelId::BallControl, ball_t));
  lball_->setPos(skeleton.ball_pos_ - skeleton.heel_pos_);
  rball_ = rheel_->addChild(Object(ModelId::BallControl, ball_t));
  rball_->setPos(skeleton.ball_pos_ - skeleton.heel_pos_);

  lankle_ = lball_->addChild(Object(ModelId::BallControl, control_t));
  lankle_->setPos(-skeleton.ball_pos_);
  rankle_ = rball_->addChild(Object(ModelId::BallControl, control_t));
  rankle_->setPos(-skeleton.ball_pos_);

  zero_p_ = Pose::freeze(*this);

  larm_ = IkChain(
      lsho_, lhand_, skeleton.lbicep_, skeleton.lforearm_, skeleton.bicep_l_,
      skeleton.forearm_l_, vec3(0, 1, 0));
  rarm_ = IkChain(
      rsho_, rhand_, skeleton.rbicep_, skeleton.rforearm_, skeleton.bicep_l_,
      skeleton.forearm_l_, vec3(0, -1, 0));

  lleg_ = IkChain(
      lhip_, lankle_, skeleton.lfemur_, skeleton.lshin_, skeleton.femur_l_,
      skeleton.shin_l_, vec3(1, 0, 0));
  rleg_ = IkChain(
      rhip_, rankle_, skeleton.rfemur_, skeleton.rshin_, skeleton.femur_l_,
      skeleton.shin_l_, vec3(1, 0, 0));

  // Marks the root position/direction.
  mat4 root_control_t = glm::scale(vec3(10, 1, 30));
  root_->addChild(Object(ModelId::BoxControl, root_control_t));
}

void BipedRig::setMaterial(MaterialId material) {
  root_->setMaterial(material);
  cog_->setMaterial(material);
  neck_->setMaterial(material);
  head_->setMaterial(material);
  lsho_->setMaterial(material);
  rsho_->setMaterial(material);
  lhand_->setMaterial(material);
  rhand_->setMaterial(material);
  pelvis_->setMaterial(material);
  lhip_->setMaterial(material);
  rhip_->setMaterial(material);
  ltoe_->setMaterial(material);
  lheel_->setMaterial(material);
  lball_->setMaterial(material);
  lankle_->setMaterial(material);
  rtoe_->setMaterial(material);
  rheel_->setMaterial(material);
  rball_->setMaterial(material);
  rankle_->setMaterial(material);
}

void BipedRig::solveIk() {
  larm_.solve();
  rarm_.solve();
  lleg_.solve();
  rleg_.solve();
}

void BipedRig::applyPose(const Pose& pose) {
  for (uint32_t i = 0; i < static_cast<uint32_t>(BoneId::COUNT); i++) {
    BoneId bone_id = static_cast<BoneId>(i);
    if (pose.bone_mask && !pose.bone_mask->contains(bone_id)) {
      continue;
    }
    getBone(bone_id)->setTransform(pose.getTransform(bone_id));
  }

  std::vector<IkChain*> all_iks = {&larm_, &rarm_, &lleg_, &rleg_};
  for (auto* ik : all_iks) {
    auto ik_it = pose.ik_dirs.find(ik);
    if (ik_it != pose.ik_dirs.end()) {
      ik->rot_axis = ik_it->second;
    }
  }
}

Object* BipedRig::getBone(BoneId bone) const {
  switch (bone) {
    case BoneId::Cog:
      return cog_;
    case BoneId::Neck:
      return neck_;
    case BoneId::Head:
      return head_;
    case BoneId::Lhand:
      return lhand_;
    case BoneId::Rhand:
      return rhand_;
    case BoneId::Pelvis:
      return pelvis_;
    case BoneId::Ltoe:
      return ltoe_;
    case BoneId::Rtoe:
      return rtoe_;
    case BoneId::Lball:
      return lball_;
    case BoneId::Rball:
      return rball_;
    default:
      ASSERT(false);
      return nullptr;
  }
}
