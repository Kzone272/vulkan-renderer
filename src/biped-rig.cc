#include "biped-rig.h"

#include "skelly.h"

void BipedSkeleton::makeBones(const SkellySizes& sizes, Object* root) {
  root_ = root;

  femur_l_ = sizes.femur;
  shin_l_ = sizes.shin;
  bicep_l_ = sizes.bicep;
  forearm_l_ = sizes.forearm;

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
    Object& bone1, float b1_l, Object& bone2, float b2_l, const vec3& b1_pos,
    const vec3& target, const vec3& main_axis, const vec3& rot_axis,
    const vec3& dir, const vec3& dir_zero) {
  // target and b1_pos in the same space.
  vec3 toward = target - b1_pos;
  float toward_l = glm::length(toward);
  toward = glm::normalize(toward);

  glm::quat point = glm::rotation(main_axis, toward);

  vec3 start = point * dir_zero;
  vec3 start_p = glm::proj(start, toward);
  vec3 start_n = glm::normalize(start - start_p);
  vec3 dir_p = glm::proj(dir, toward);
  vec3 dir_n = glm::normalize(dir - dir_p);
  quat spin = glm::rotation(start_n, dir_n);

  auto [j1, j2] = solveIk(b1_l, b2_l, toward_l);
  bone1.setRot(spin * point * glm::angleAxis(j1, rot_axis));
  bone2.setRot(glm::angleAxis(j2, rot_axis));
}

}  // namespace

IkChain::IkChain(
    Object* start, Object* target, Object* pole, Object* b1, Object* b2,
    float b1_l, float b2_l)
    : start(start),
      target(target),
      pole(pole),
      lca(Object::lca(start, target)),
      b1(b1),
      b2(b2),
      b1_l(b1_l),
      b2_l(b2_l) {
  DASSERT(lca != nullptr);
  point_zero = glm::normalize(targetPos() - startPos());

  vec3 pole_dir = pole->getPos();
  vec3 pole_dir_p = glm::proj(pole_dir, point_zero);
  this->dir_zero = glm::normalize(pole_dir - pole_dir_p);

  rot_axis = glm::cross(dir_zero, point_zero);
}

vec3 IkChain::startPos() {
  return start->getPos();
}
vec3 IkChain::targetPos() {
  return start->getParent()->posToLocal(lca, target->posToAncestor(lca));
}

void IkChain::solve() {
  vec3 dir_local = start->toLocal(lca) * vec4(pole->getPos(), 0);
  solveTwoBoneIk(
      *b1, b1_l, *b2, b2_l, startPos(), targetPos(), point_zero, rot_axis,
      dir_local, dir_zero);
}

void BipedRig::updateSkeleton(BipedSkeleton& skl) {
  curr_pose_.computeRootMatrices();

  skl.cog_->setTransform(cog_->getTransform());
  skl.torso_->setTransform(neck_->getTransform());
  skl.head_->setTransform(head_->getTransform());
  skl.pelvis_->setTransform(pelvis_->getTransform());
  skl.ltoes_->setTransform(lball_->getTransform());
  skl.rtoes_->setTransform(rball_->getTransform());

  solveIk();

  skl.lfoot_->setRot(glm::identity<glm::quat>());
  auto l_flat = glm::quat_cast(skl.lfoot_->toLocal(root_));
  skl.lfoot_->setRot(l_flat * lankle_->getRot());

  skl.rfoot_->setRot(glm::identity<glm::quat>());
  auto r_flat = glm::quat_cast(skl.rfoot_->toLocal(root_));
  skl.rfoot_->setRot(r_flat * rankle_->getRot());
}

void BipedRig::addBone(Id bone, Id parent, vec3 pos) {
  skl_.addBone(bone, parent);
  zero_pose_.setPos(bone, pos);
}

void BipedRig::makeRig(const BipedSkeleton& skeleton, Object* root) {
  root_ = root;

  zero_pose_ = {&skl_};

  addBone(Id::Cog, Id::NoParent, skeleton.cog_->getPos());
  addBone(Id::Neck, Id::Cog, skeleton.torso_->getPos());
  addBone(Id::Head, Id::Neck, skeleton.head_->getPos());
  addBone(Id::Lsho, Id::Neck, skeleton.lbicep_->getPos());
  addBone(Id::Rsho, Id::Neck, skeleton.rbicep_->getPos());
  addBone(Id::Lhand, Id::Neck, skeleton.lhand_->posToAncestor(skeleton.torso_));
  addBone(Id::Rhand, Id::Neck, skeleton.rhand_->posToAncestor(skeleton.torso_));
  addBone(Id::Pelvis, Id::Cog, skeleton.cog_->getPos());
  addBone(Id::Lhip, Id::Pelvis, skeleton.lfemur_->getPos());
  addBone(Id::Rhip, Id::Pelvis, skeleton.rfemur_->getPos());
  addBone(Id::Lankle, Id::NoParent, skeleton.lfoot_->posToAncestor(root_));
  addBone(Id::Rankle, Id::NoParent, skeleton.rfoot_->posToAncestor(root_));
  addBone(Id::Lball, Id::Lankle, skeleton.ltoes_->getPos());
  addBone(Id::Rball, Id::Rankle, skeleton.rtoes_->getPos());
  addBone(Id::Lelbow, Id::Neck, vec3(0, 0, -1));
  addBone(Id::Relbow, Id::Neck, vec3(0, 0, -1));
  addBone(Id::Lknee, Id::NoParent, vec3(0, 0, 1));
  addBone(Id::Rknee, Id::NoParent, vec3(0, 0, 1));

  zero_pose_.computeRootMatrices();

  larm_ = IkChain(
      lsho_, lhand_, lelbow_, skeleton.lbicep_, skeleton.lforearm_,
      skeleton.bicep_l_, skeleton.forearm_l_);
  rarm_ = IkChain(
      rsho_, rhand_, relbow_, skeleton.rbicep_, skeleton.rforearm_,
      skeleton.bicep_l_, skeleton.forearm_l_);

  lleg_ = IkChain(
      lhip_, lankle_, lknee_, skeleton.lfemur_, skeleton.lshin_,
      skeleton.femur_l_, skeleton.shin_l_);
  rleg_ = IkChain(
      rhip_, rankle_, rknee_, skeleton.rfemur_, skeleton.rshin_,
      skeleton.femur_l_, skeleton.shin_l_);

  // Marks the root position/direction.
  // mat4 root_control_t = glm::scale(vec3(10, 1, 30));
  // root_->addChild(Object(ModelId::BoxControl, root_control_t));
}

void BipedRig::getSceneObjects(
    const mat4& parent, std::vector<SceneObject>& objs,
    const std::set<ModelId>& hidden) {
  static mat4 control_t = glm::scale(vec3(5));
  static mat4 pelvis_t = glm::scale(vec3(15, 1, 15));
  static mat4 small_t = glm::scale(vec3(3));

  for (size_t i = 0; i < Id::COUNT; i++) {
    ModelId model =
        i == Id::Pelvis ? ModelId::BoxControl : ModelId::BallControl;
    mat4& model_t = control_t;
    if (i == Id::Pelvis) {
      model_t = pelvis_t;
    } else if (i == Id::Lankle || i == Id::Rankle) {
      model_t = small_t;
    }

    if (model == ModelId::None || hidden.contains(model)) {
      continue;
    }

    objs.push_back({
        model,
        mat_,
        parent * curr_pose_.getRootMatrix(i) * model_t,
    });
  }
}

void BipedRig::setMaterial(MaterialId material) {
  mat_ = material;
}

void BipedRig::solveIk() {
  larm_.solve();
  rarm_.solve();
  lleg_.solve();
  rleg_.solve();
}

void BipedRig::applyPose(const Pose& pose) {
  curr_pose_ = pose;
}
