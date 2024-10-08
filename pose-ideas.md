Planned changes to Pose/Rig/Skeleton

cosntexpr uint32_t noParent = -1;
Skeleton() {
  addBone(uint32_t b, vec3 pos, uint32_t p) {
    ASSERT(b == parents.size() -1);
    pos.push(b);
    parents.push(b);
  }

  parent(uint32_t i) {
    return parents[i];
  }
  size_t count() {
    returns parents.size();
  }

  std::vector<vec3> pos;
  std::vector<uint32_t> parents;
}

Pose() {
  const mat4& getMatrix(uint32_t i) {
    return ts_[i].matrix();
  }

  void computeRootMatrices() {
    for (auto i = 0; i < skl_.count(); i++) {
      computeRootMatrix(i);
      mat4& m = root_ms_[i];
      m = getMatrix(i);
      uint32_t p = skl_.parent(i);
      if (p != kNoParent) {
        m = root_ms_[p] * m;
      }
    }
  }

  std::vector<Transform> ts_;
  std::vector<mat4> root_ms_;
  Skeleton skl_;
}

BipedBones() {
  enum class B {
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

  addBone(B p, B b, vec3 pos) {
    skl_.add(static_cast<uint32_t>(b), pos, static_cast<uint32_t>(p));
  }

  BipedBones() {
    addBone(-1, B::cog, cog_pos)
    addBone(B::cog, B::pelvis, pelvis_pos);
    addBone(B::pelvis, B::torso, torso_pos);
    addBone(B::torso, B::lbicep, lbicep_pos);
    ...
  }

  Skeleton skl_;
}

BipedRig() {
  enum class B {
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
    COUNT,
  };

  addBone(B p, B b, vec3 pos) {
    skl_.add(static_cast<uint32_t>(b), pos, static_cast<uint32_t>(p));
  }

  BipedRip() {
    addBone(-1, B::cog, cog_pos)
    addBone(B::cog, B::neck, neck_pos);
    addBone(B::neck, B::head, head_pos);
    addBone(B::neck, B::lsho, lsho_pos);
    ...
  }

  applyToSkeleton() {
    
  }

  Skeleton skl_;
}


SkeletonObject() {
  setPose(Pose pose) {
    pose_ = pose;
  }

  getSceneObjects(
      const mat4& parent, std::vector<SceneObject>& objs,
      const std::set<ModelId>& hidden) {
    for (const bone : bones_) {
      objs.push_back({
        models_[bone],
        material_,
        parent * transform * model_ts_[bone],
    });
    }
  }

  Pose pose_;
  std::vector<ModelId> models_;
  std::vector<Transform> model_ts_;
}
