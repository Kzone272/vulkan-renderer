Planned changes to Pose/Rig/Skeleton

cosntexpr size_t kNoParent = -1;
Skeleton() {
  addBone(size_t b, size_t p) {
    DASSERT(b == parents.size());
    parents.push(b);
  }

  parent(size_t i) {
    return parents[i];
  }
  size_t count() {
    returns parents.size();
  }

  std::vector<vec3> pos;
  std::vector<size_t> parents;
}

Pose() {
  const mat4& getMatrix(size_t i) {
    return ts_[i].matrix();
  }

  void computeRootMatrices() {
    for (auto i = 0; i < skl_.count(); i++) {
      computeRootMatrix(i);
      mat4& m = root_ms_[i];
      m = getMatrix(i);
      size_t p = skl_.parent(i);
      if (p != kNoParent) {
        m = root_ms_[p] * m;
      }
    }
  }

  getRootMatrix(size_t i) {
    return root_ms_[i];
  }
  vec3 getRootPos(size_t i) {
    return vec3(getRootMatrix(i) * vec3(0,0,0,1));
  }

  std::vector<Transform> ts_;
  std::vector<mat4> root_ms_;
  Skeleton skl_;
}

BipedAnim() {
  enum class Id {
    cog,
    pelvis,
    torso,
    lbicep,
    lforearm,
    lhand,
    rbicep,
    rforearm,
    rhand,
    head,
    lfemur,
    lshin,
    lfoot,
    ltoes,
    rfemur,
    rshin,
    rfoot,
    rtoes,
    COUNT,
  };

  addBone(Id b, Id p, vec3 pos) {
    bi = static_cast<size_t>(b);
    skl_.add(bi, static_cast<size_t>(p));
    zero_pose_.setPos(bi, pos);
  }

  BipedBones() {
    pose_ = {skl_, Id::COUNT};

    addBone(-1, Id::cog, cog_pos)
    addBone(Id::cog, Id::pelvis, pelvis_pos);
    addBone(Id::pelvis, Id::torso, torso_pos);
    addBone(Id::torso, Id::lbicep, lbicep_pos);
    ...
  }

  Skeleton skl_;
  Pose zero_pose_;
  std::vector<mat4> ts_;
}

BipedRig() {
  enum class Id size_t {
    cog,
    neck,
    head,
    lsho,  // (sho)ulder
    rsho,
    lhand,
    rhand,
    pelvis,
    lhip,
    rhip,
    lankle,
    lball,  // ball of foot
    rankle,
    rball,
    lelbow,
    relbow,
    lknee,
    rknee,
    COUNT,
  };
  static const size_t kBoneCount = static_cast<size_t>(Id::COUNT);

  addBone(Id b, Id p, vec3 pos) {
    bi = static_cast<size_t>(b);
    skl_.add(bi, static_cast<size_t>(p));
    zero_pose_.setPos(bi, pos);
  }

  size_t map(BipedRig::Id id) {
    static std::map<Id, BipedAnim::Id> map = {
      {Id::cog, BipedAnim::Id::cog},
      {Id::neck, BipedAnim::Id::torso},
      ...
    };

    i = -1;
    auto it = map.find(id);
    if (it != end) {
      i = static_cast<size_t>(it->second);
    }
    return i;
  }

  BipedRig(Pose anim_pose) {
    anim_pose.computeRootMatrices()

    addBone(kNoParent, Id::cog, anim_pose.getPos(map(Id::cog)));
    addBone(Id::cog, Id::neck, anim_pose.getPos(map(Id::neck)));
    addBone(Id::neck, Id::head, anim_pose.getPos(map(Id::head)));
    addBone(Id::neck, Id::lsho, anim_pose.getPos(map(Id::lsho)));
    ...
    addBone(kNoParent, Id::lankle, anim_pose.getRootPos(BipedAnim::Id::lfoot));
    addBone(Id::lankle, Id::lball, anim_pose.getPos(map(Id::lball)));
    ...
  }

  applyPose(Pose rig_pose) {
    rig_pose.computeRootMatrices();
    for (size_t i = 0; i < Id::COUNT; i++) {
      auto rig_id = static_cast<Id>(i);
      anim_i = map(rig_id);
      if (anim_i != -1) {
        anim_pose_[anim_i] = rig_pose.getRootMatrix(rig_id);
      }
    }
    solveIk();
  }

  solveIk() {
    larm_.solve(&anim_pose_);
  }

  Pose zero_pose_;
  std::vector<mat4> anim_pose_;
  Skeleton skl_;
}

SkeletonObject() {
  setTransforms(std::vector<mat4> mats) {
    bone_ts_ = mats;
  }

  getSceneObjects(
      const mat4& parent, std::vector<SceneObject>& objs,
      const std::set<ModelId>& hidden) {
    for (const bone : bones_) {
        objs.push_back({
          models_[bone],
          material_,
          parent * bone_ts[bone] * model_ts_[bone],
        });
    }
  }

  std::vector<ModelId> models_;
  std::vector<mat4> bone_ts_;
  std::vector<mat4> model_ts_;
}