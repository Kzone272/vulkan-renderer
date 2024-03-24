#include "biped-rig.h"

#include "skelly.h"

void BipedSkeleton::makeBones(const SkellySizes& sizes, Object* root) {
  root_ = root;

  femur_l_ = sizes.femur;
  shin_l_ = sizes.shin;
  bicep_l_ = sizes.bicep;
  forearm_l_ = sizes.forearm;
  toe_pos_ = sizes.toe;

  cog_ = root_->addChild(Object(ModelId::None, glm::scale(vec3(5))));
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
    Object* root, Object* target, Object* b1, Object* b2, float b1_l,
    float b2_l, vec3 rot_axis)
    : root(root),
      target(target),
      b1(b1),
      b2(b2),
      b1_l(b1_l),
      b2_l(b2_l),
      rot_axis(rot_axis) {
  point_zero = glm::normalize(targetPos() - rootPos());
}

vec3 IkChain::rootPos() {
  return root->getPos();
}
vec3 IkChain::targetPos() {
  return root->getParent()->posToLocal(target->getParent(), target->getPos());
}

void IkChain::solve() {
  solveTwoBoneIk(
      *b1, b1_l, *b2, b2_l, rootPos(), targetPos(), point_zero, rot_axis);
}

void BipedRig::updateSkeleton(BipedSkeleton& skl) {
  skl.cog_->setTransform(cog_->getTransform());
  skl.torso_->setTransform(neck_->getTransform());
  skl.head_->setTransform(head_->getTransform());
  skl.pelvis_->setTransform(pelvis_->getTransform());

  solveIk();

  skl.lfoot_->setRot(glm::identity<glm::quat>());
  auto l_flat = glm::quat_cast(skl.lfoot_->toLocal(root_));
  skl.lfoot_->setRot(lfoot_->getRot() * l_flat);

  skl.rfoot_->setRot(glm::identity<glm::quat>());
  auto r_flat = glm::quat_cast(skl.rfoot_->toLocal(root_));
  skl.rfoot_->setRot(rfoot_->getRot() * r_flat);
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

  lhand_ = root_->addChild(Object(ModelId::BallControl, control_t));
  lhand_->setPos(skeleton.lhand_->posToAncestor(root_));
  rhand_ = root_->addChild(Object(ModelId::BallControl, control_t));
  rhand_->setPos(skeleton.rhand_->posToAncestor(root_));

  mat4 pelvis_t = glm::scale(vec3(15, 1, 15));
  pelvis_ = cog_->addChild(Object(ModelId::BoxControl, pelvis_t));
  cog_->setPos(skeleton.cog_->getPos());

  lhip_ = pelvis_->addChild(Object(ModelId::BallControl, control_t));
  lhip_->setPos(skeleton.lfemur_->getPos());
  rhip_ = pelvis_->addChild(Object(ModelId::BallControl, control_t));
  rhip_->setPos(skeleton.rfemur_->getPos());

  lfoot_ = root_->addChild(Object(ModelId::BallControl, control_t));
  lfoot_->setPos(skeleton.lfoot_->posToAncestor(root_));
  rfoot_ = root_->addChild(Object(ModelId::BallControl, control_t));
  rfoot_->setPos(skeleton.rfoot_->posToAncestor(root_));

  mat4 toe_t = glm::scale(vec3(3));
  ltoe_ = lfoot_->addChild(Object(ModelId::BallControl, toe_t));
  ltoe_->setPos(skeleton.toe_pos_);
  rtoe_ = rfoot_->addChild(Object(ModelId::BallControl, toe_t));
  rtoe_->setPos(skeleton.toe_pos_);

  zero_p_ = Pose::freeze(*this);

  larm_ = IkChain(
      lsho_, lhand_, skeleton.lbicep_, skeleton.lforearm_, skeleton.bicep_l_,
      skeleton.forearm_l_, vec3(0, 1, 0));
  rarm_ = IkChain(
      rsho_, rhand_, skeleton.rbicep_, skeleton.rforearm_, skeleton.bicep_l_,
      skeleton.forearm_l_, vec3(0, -1, 0));

  lleg_ = IkChain(
      lhip_, lfoot_, skeleton.lfemur_, skeleton.lshin_, skeleton.femur_l_,
      skeleton.shin_l_, vec3(1, 0, 0));
  rleg_ = IkChain(
      rhip_, rfoot_, skeleton.rfemur_, skeleton.rshin_, skeleton.femur_l_,
      skeleton.shin_l_, vec3(1, 0, 0));

  // Marks the root position/direction.
  mat4 root_control_t = glm::scale(vec3(10, 1, 30));
  root_->addChild(Object(ModelId::BoxControl, root_control_t));
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
    getBone(bone_id)->setTransform(pose.getBoneConst(bone_id));
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
    case BoneId::Lfoot:
      return lfoot_;
    case BoneId::Rfoot:
      return rfoot_;
    default:
      ASSERT(false);
      return nullptr;
  }
}
