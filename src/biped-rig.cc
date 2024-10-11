#include "biped-rig.h"

#include "skelly.h"

BipedSkeleton::BipedSkeleton() {
  skl_.addBone(Id::cog, Id::NoParent);
  skl_.addBone(Id::pelvis, Id::cog);
  skl_.addBone(Id::torso, Id::cog);
  skl_.addBone(Id::lbicep, Id::torso);
  skl_.addBone(Id::lforearm, Id::lbicep);
  skl_.addBone(Id::lhand, Id::lforearm);
  skl_.addBone(Id::rbicep, Id::torso);
  skl_.addBone(Id::rforearm, Id::rbicep);
  skl_.addBone(Id::rhand, Id::rforearm);
  skl_.addBone(Id::head, Id::torso);
  skl_.addBone(Id::lfemur, Id::pelvis);
  skl_.addBone(Id::lshin, Id::lfemur);
  skl_.addBone(Id::lfoot, Id::lshin);
  skl_.addBone(Id::ltoes, Id::lfoot);
  skl_.addBone(Id::rfemur, Id::pelvis);
  skl_.addBone(Id::rshin, Id::rfemur);
  skl_.addBone(Id::rfoot, Id::rshin);
  skl_.addBone(Id::rtoes, Id::rfoot);
}

void BipedSkeleton::setBone(Id bone, const vec3& pos, const mat4& model_t) {
  zero_pose_.setPos(bone, pos);
  model_ts_[bone] = model_t;
}

void BipedSkeleton::makeBones(const SkellySizes& sizes) {
  setBone(Id::cog, vec3(0, sizes.pelvis_y, 0), mat4(1));
  models_[Id::cog] = ModelId::None;

  mat4 pelvis_t = glm::scale(vec3(sizes.pelvis_w, -sizes.pelvis_h, 15));
  setBone(Id::pelvis, vec3(0), pelvis_t);

  mat4 torso_t = glm::scale(vec3(sizes.shoulders_w, -15, 15));
  setBone(Id::torso, vec3(0, sizes.shoulders_y, 0), torso_t);

  mat4 arm_rot = glm::toMat4(glm::angleAxis(glm::radians(90.f), vec3(0, 0, 1)));
  mat4 bicep_t =
      glm::scale(vec3(sizes.bicep, sizes.bone_w, sizes.bone_w)) * arm_rot;
  setBone(Id::lbicep, sizes.sho_pos, bicep_t);

  vec3 forearm_pos = {-sizes.bicep, 0, 0};
  mat4 forearm_t =
      glm::scale(vec3(sizes.forearm, sizes.bone_w, sizes.bone_w)) * arm_rot;
  setBone(Id::lforearm, forearm_pos, forearm_t);

  vec3 hand_pos = {-sizes.forearm, 0, 0};
  mat4 hand_t = glm::scale(vec3(sizes.hand_l, 7, 9)) * arm_rot;
  setBone(Id::lhand, hand_pos, hand_t);

  mat4 head_t =
      glm::translate(vec3(0, 0, 2)) * glm::scale(vec3(20, sizes.head_h, 20));
  setBone(Id::head, vec3(0, sizes.neck, 5), head_t);

  mat4 femur_t = glm::scale(vec3(sizes.bone_w, -sizes.femur, sizes.bone_w));
  setBone(Id::lfemur, sizes.hip_pos, femur_t);

  mat4 shin_t = glm::scale(vec3(sizes.bone_w, -sizes.shin, sizes.bone_w));
  vec3 shin_pos = vec3(0, -sizes.femur, 0);
  setBone(Id::lshin, shin_pos, shin_t);

  mat4 foot_t = glm::translate(sizes.heel + vec3(-2, 0, 0)) *
                glm::scale(vec3(12, 4, sizes.foot_l - sizes.toes_l)) *
                glm::translate(vec3(0, 0, 0.5));
  vec3 foot_pos = vec3(0, -sizes.shin, 0);
  setBone(Id::lfoot, foot_pos, foot_t);

  mat4 toes_t = glm::translate(vec3(-2, 0, 0)) *
                glm::scale(vec3(12, 4, sizes.toes_l)) *
                glm::translate(vec3(0, 0, 0.5));
  vec3 toes_pos = sizes.ball;
  setBone(Id::ltoes, toes_pos, toes_t);

  // Add opposite limbs with flipped positions.
  mat4 flip = glm::scale(vec3(-1, 1, 1));
  mat3 flip3 = mat3(flip);

  setBone(Id::rfemur, flip3 * sizes.hip_pos, femur_t);
  setBone(Id::rshin, flip3 * shin_pos, shin_t);
  setBone(Id::rfoot, flip3 * foot_pos, flip * foot_t);
  setBone(Id::rtoes, flip3 * toes_pos, flip * toes_t);
  setBone(Id::rbicep, flip3 * sizes.sho_pos, flip * bicep_t);
  setBone(Id::rforearm, flip3 * forearm_pos, flip * forearm_t);
  setBone(Id::rhand, flip3 * hand_pos, flip * hand_t);

  zero_pose_.computeRootMatrices();
  curr_pose_ = zero_pose_.getRootMatrices();
}

void BipedSkeleton::setMaterial(MaterialId material) {
  mat_ = material;
}

void BipedSkeleton::getSceneObjects(
    const mat4& parent, std::vector<SceneObject>& objs,
    const std::set<ModelId>& hidden) {
  for (size_t i = 0; i < Id::COUNT; i++) {
    auto model = models_[i];
    if (model == ModelId::None || hidden.contains(model)) {
      continue;
    }

    objs.push_back({
        model,
        mat_,
        parent * curr_pose_[i] * model_ts_[i],
    });
  }
}

namespace {

// Source: https://iquilezles.org/articles/simpleik/
// dir is the normal of the plane this IK chain is on.
// i.e. The knee rotation axis.
vec3 solve(const vec3& p, float r1, float r2, const vec3& dir) {
  vec3 q = p * (0.5f + 0.5f * (r1 * r1 - r2 * r2) / glm::dot(p, p));

  float s = r1 * r1 - glm::dot(q, q);
  s = std::max(s, 0.f);
  q += sqrt(s) * glm::normalize(glm::cross(p, dir));

  return q;
}

// Source: https://iquilezles.org/articles/simpleik/
vec3 solve(const vec3& a, const vec3& b, float l1, float l2, const vec3& dir) {
  return a + solve(b - a, l1, l2, dir);
}

// Returns pair of angles for bone1 and bone2.
std::pair<float, float> solveIk(float bone1, float bone2, float target) {
  if (target >= bone1 + bone2) {
    return {0.f, 0.f};
  }

  float b1 = -cosineLaw(bone1, target, bone2);
  float b2 = glm::radians(180.f) - cosineLaw(bone1, bone2, target);
  return {b1, b2};
}

// start, target, pole, main_axis, and rot_axis all need to be in the same
// space.
void solveTwoBoneIk(
    float b1_l, float b2_l, const vec3& start, const vec3& target,
    const vec3& pole, const vec3& main_axis, const vec3& rot_axis, mat4& b1_m,
    mat4& b2_m) {
  vec3 toward = glm::normalize(target - start);
  vec3 plane_n = glm::normalize(glm::cross(pole - start, toward));
  vec3 knee = solve(start, target, b1_l, b2_l, plane_n);

  vec3 to_knee = glm::normalize(knee - start);
  quat point1 = glm::rotation(main_axis, to_knee);
  quat point2 = glm::rotation(to_knee, glm::normalize(target - knee));

  vec3 mod = glm::inverse(point1) * plane_n;
  vec3 mod_p = glm::proj(mod, main_axis);
  vec3 mod_n = glm::normalize(mod - mod_p);
  quat spin = glm::rotation(rot_axis, mod_n);

  quat b1_rot = point1 * spin;
  b1_m = glm::translate(start) * glm::toMat4(b1_rot);
  b2_m = glm::translate(knee) * glm::toMat4(point2 * b1_rot);
}

}  // namespace

IkChain::IkChain(
    BipedRig::Id start, BipedRig::Id target, BipedRig::Id pole,
    const Pose& rig_zero_pose, BipedSkeleton::Id b1, BipedSkeleton::Id b2,
    float b1_l, float b2_l)
    : start(start),
      target(target),
      pole(pole),
      b1(b1),
      b2(b2),
      b1_l(b1_l),
      b2_l(b2_l) {
  vec3 start_pos = rig_zero_pose.getRootPos(start);
  vec3 target_pos = rig_zero_pose.getRootPos(target);
  point_zero = glm::normalize(target_pos - start_pos);

  vec3 pole_pos = rig_zero_pose.getRootPos(pole);
  rot_axis = glm::normalize(glm::cross(pole_pos - start_pos, point_zero));
}

void IkChain::solve(const Pose& rig_pose, std::vector<mat4>& anim_pose) {
  solveTwoBoneIk(
      b1_l, b2_l, rig_pose.getRootPos(start), rig_pose.getRootPos(target),
      rig_pose.getRootPos(pole), point_zero, rot_axis, anim_pose[b1],
      anim_pose[b2]);
}

BipedRig::BipedRig() {
  skl_.addBone(Id::Cog, Id::NoParent);
  skl_.addBone(Id::Neck, Id::Cog);
  skl_.addBone(Id::Head, Id::Neck);
  skl_.addBone(Id::Lsho, Id::Neck);
  skl_.addBone(Id::Rsho, Id::Neck);
  skl_.addBone(Id::Lhand, Id::Neck);
  skl_.addBone(Id::Rhand, Id::Neck);
  skl_.addBone(Id::Pelvis, Id::Cog);
  skl_.addBone(Id::Lhip, Id::Pelvis);
  skl_.addBone(Id::Rhip, Id::Pelvis);
  skl_.addBone(Id::Lankle, Id::NoParent);
  skl_.addBone(Id::Rankle, Id::NoParent);
  skl_.addBone(Id::Lball, Id::Lankle);
  skl_.addBone(Id::Rball, Id::Rankle);
  skl_.addBone(Id::Lelbow, Id::Neck);
  skl_.addBone(Id::Relbow, Id::Neck);
  skl_.addBone(Id::Lknee, Id::NoParent);
  skl_.addBone(Id::Rknee, Id::NoParent);

  models_[Id::Pelvis] = ModelId::BoxControl;
  model_ts_ = {Id::COUNT, glm::scale(vec3(5))};
  model_ts_[Id::Pelvis] = glm::scale(vec3(15, 1, 15));
  static mat4 small_t = glm::scale(vec3(3));
  model_ts_[Id::Lball] = small_t;
  model_ts_[Id::Rball] = small_t;
  models_[Id::Lknee] = ModelId::BoxControl;
  models_[Id::Rknee] = ModelId::BoxControl;
  models_[Id::Lelbow] = ModelId::BoxControl;
  models_[Id::Relbow] = ModelId::BoxControl;
  model_ts_[Id::Lknee] = small_t;
  model_ts_[Id::Rknee] = small_t;
  model_ts_[Id::Lelbow] = small_t;
  model_ts_[Id::Relbow] = small_t;
}

BipedSkeleton::Id BipedRig::map(Id rig_id) {
  // Bones that directly map their local-space transforms from the rig to the
  // animation skeleton. That is bones with the same hierarchy chain. Other
  // bones' transforms are based on IK.
  static std::map<Id, BipedSkeleton::Id> map = {
      {Id::Cog, BipedSkeleton::Id::cog},
      {Id::Neck, BipedSkeleton::Id::torso},
      {Id::Head, BipedSkeleton::Id::head},
      {Id::Lsho, BipedSkeleton::Id::lbicep},
      {Id::Rsho, BipedSkeleton::Id::rbicep},
      {Id::Pelvis, BipedSkeleton::Id::pelvis},
      {Id::Lhip, BipedSkeleton::Id::lfemur},
      {Id::Rhip, BipedSkeleton::Id::rfemur},
      {Id::Lball, BipedSkeleton::Id::ltoes},
      {Id::Rball, BipedSkeleton::Id::rtoes},
  };

  BipedSkeleton::Id anim_id = BipedSkeleton::Id::NoParent;
  auto it = map.find(rig_id);
  if (it != map.end()) {
    anim_id = it->second;
  }
  return anim_id;
}

void BipedRig::updateSkeleton(BipedSkeleton& skl) {
  curr_pose_.computeRootMatrices();

  std::vector<mat4> anim_pose(BipedSkeleton::Id::COUNT);

  for (size_t i = 0; i < Id::COUNT; i++) {
    auto anim_id = map((Id)i);
    if (anim_id != -1) {
      anim_pose[anim_id] = curr_pose_.getRootMatrix(i);
    }
  }
  anim_pose[BipedSkeleton::Id::lfoot] = curr_pose_.getRootMatrix(Id::Lankle);
  anim_pose[BipedSkeleton::Id::rfoot] = curr_pose_.getRootMatrix(Id::Rankle);
  anim_pose[BipedSkeleton::Id::lhand] = curr_pose_.getRootMatrix(Id::Lhand);
  anim_pose[BipedSkeleton::Id::rhand] = curr_pose_.getRootMatrix(Id::Rhand);

  solveIk(anim_pose);

  // Hack. Rotate hands so they would be "unrotated" in local space.
  mat4 lforearm_rot =
      glm::toMat4(glm::quat_cast(anim_pose[BipedSkeleton::Id::lforearm]));
  anim_pose[BipedSkeleton::Id::lhand] =
      anim_pose[BipedSkeleton::Id::lhand] * lforearm_rot;
  mat4 rforearm_rot =
      glm::toMat4(glm::quat_cast(anim_pose[BipedSkeleton::Id::rforearm]));
  anim_pose[BipedSkeleton::Id::rhand] =
      anim_pose[BipedSkeleton::Id::rhand] * rforearm_rot;

  skl.setPose(std::move(anim_pose));
}

void BipedRig::setBone(Id bone, vec3 pos) {
  zero_pose_.setPos(bone, pos);
}

void BipedRig::makeRig(const Pose& anim_pose) {
  for (size_t i = 0; i < Id::COUNT; i++) {
    Id rig_id = (Id)i;
    auto anim_id = map(rig_id);
    if (anim_id != -1) {
      setBone(rig_id, anim_pose.getPos(anim_id));
    }
  }

  mat4 to_torso =
      glm::inverse(anim_pose.getRootMatrix(BipedSkeleton::Id::torso));
  vec3 lhand_pos =
      to_torso * vec4(anim_pose.getRootPos(BipedSkeleton::Id::lhand), 1);
  vec3 rhand_pos =
      to_torso * vec4(anim_pose.getRootPos(BipedSkeleton::Id::rhand), 1);
  setBone(Id::Lhand, lhand_pos);
  setBone(Id::Rhand, rhand_pos);

  setBone(Id::Lankle, anim_pose.getRootPos(BipedSkeleton::Id::lfoot));
  setBone(Id::Rankle, anim_pose.getRootPos(BipedSkeleton::Id::rfoot));

  setBone(Id::Lelbow, zero_pose_.getPos(Id::Lsho) + vec3(0, 0, -100));
  setBone(Id::Relbow, zero_pose_.getPos(Id::Rsho) + vec3(0, 0, -100));
  setBone(Id::Lknee, zero_pose_.getPos(Id::Lankle) + vec3(0, 0, 100));
  setBone(Id::Rknee, zero_pose_.getPos(Id::Rankle) + vec3(0, 0, 100));

  zero_pose_.computeRootMatrices();

  auto dist = [&](BipedSkeleton::Id id1, BipedSkeleton::Id id2) -> float {
    vec3 pos1 = anim_pose.getRootPos(id1);
    vec3 pos2 = anim_pose.getRootPos(id2);
    return glm::length(pos1 - pos2);
  };
  float lbicep_l = dist(BipedSkeleton::Id::lbicep, BipedSkeleton::Id::lforearm);
  float lforearm_l =
      dist(BipedSkeleton::Id::lforearm, BipedSkeleton::Id::lhand);
  float rbicep_l = dist(BipedSkeleton::Id::rbicep, BipedSkeleton::Id::rforearm);
  float rforearm_l =
      dist(BipedSkeleton::Id::rforearm, BipedSkeleton::Id::rhand);

  larm_ = IkChain(
      Id::Lsho, Id::Lhand, Id::Lelbow, zero_pose_, BipedSkeleton::Id::lbicep,
      BipedSkeleton::Id::lforearm, lbicep_l, lforearm_l);
  rarm_ = IkChain(
      Id::Rsho, Id::Rhand, Id::Relbow, zero_pose_, BipedSkeleton::Id::rbicep,
      BipedSkeleton::Id::rforearm, rbicep_l, rforearm_l);

  float lfemur_l = dist(BipedSkeleton::Id::lfemur, BipedSkeleton::Id::lshin);
  float lshin_l = dist(BipedSkeleton::Id::lshin, BipedSkeleton::Id::lfoot);
  float rfemur_l = dist(BipedSkeleton::Id::rfemur, BipedSkeleton::Id::rshin);
  float rshin_l = dist(BipedSkeleton::Id::rshin, BipedSkeleton::Id::rfoot);

  lleg_ = IkChain(
      Id::Lhip, Id::Lankle, Id::Lknee, zero_pose_, BipedSkeleton::Id::lfemur,
      BipedSkeleton::Id::lshin, lfemur_l, lshin_l);
  rleg_ = IkChain(
      Id::Rhip, Id::Rankle, Id::Rknee, zero_pose_, BipedSkeleton::Id::rfemur,
      BipedSkeleton::Id::rshin, rfemur_l, rshin_l);
}

void BipedRig::getSceneObjects(
    const mat4& parent, std::vector<SceneObject>& objs,
    const std::set<ModelId>& hidden) {
  static mat4 control_t = glm::scale(vec3(5));
  static mat4 pelvis_t = glm::scale(vec3(15, 1, 15));
  static mat4 small_t = glm::scale(vec3(3));

  for (size_t i = 0; i < Id::COUNT; i++) {
    ModelId model = models_[i];
    if (model == ModelId::None || hidden.contains(model)) {
      continue;
    }

    objs.push_back({
        model,
        mat_,
        parent * curr_pose_.getRootMatrix(i) * model_ts_[i],
    });
  }
}

void BipedRig::setMaterial(MaterialId material) {
  mat_ = material;
}

void BipedRig::solveIk(std::vector<mat4>& anim_pose) {
  larm_.solve(curr_pose_, anim_pose);
  rarm_.solve(curr_pose_, anim_pose);
  lleg_.solve(curr_pose_, anim_pose);
  rleg_.solve(curr_pose_, anim_pose);
}

void BipedRig::applyPose(const Pose& pose) {
  curr_pose_ = pose;
}
