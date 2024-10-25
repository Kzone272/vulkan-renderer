#pragma once

#include <print>
#include <vector>

#include "asserts.h"
#include "glm-include.h"
#include "object.h"
#include "transform.h"
#include "vec-maths.h"

// TODO: Redundant with Transform.h. Delete one of them.
struct TData {
  TData() = default;

  quat rot = glm::identity<quat>();
  vec3 pos = vec3(0);
  vec3 scale = vec3(1);

  mat4 matrix() const {
    // Equivalent to, but faster than:
    //   glm::translate(pos) * glm::toMat4(rot) * glm::scale(scale);
    mat4 mat = glm::toMat4(rot);
    mat[0] *= scale[0];
    mat[1] *= scale[1];
    mat[2] *= scale[2];
    mat[3][0] = pos[0];
    mat[3][1] = pos[1];
    mat[3][2] = pos[2];
    return mat;
  }
};

// Stores data for all Objects in a flat structure. This makes iterating through
// them when flattening the tree much faster due to cache-friendly data access.
class WorldTree {
 public:
  WorldTree() {
  }

  void addChild(Object* child) {
    children_.push_back(child);
    in_order_ = false;
  }

  Object* makeObject(
      ModelId model = ModelId::None,
      std::optional<mat4> model_transform = std::nullopt) {
    auto unique_child = std::make_unique<Object>(this, model, model_transform);
    auto* ptr = unique_child.get();
    owned_children_.push_back(std::move(unique_child));
    addChild(ptr);
    return ptr;
  }

  void reg(Object* obj) {
    in_order_ = false;

    obj->setObjectIndex(objects_.size());
    objects_.emplace_back(obj);
    parents_.emplace_back(kRootParent);
    ts_.emplace_back();
    local_ms_.emplace_back(1.f);
    local_dirty_.emplace_back(true);
    root_ms_.emplace_back(1.f);
    root_dirty_.emplace_back(true);
    model_ms_.emplace_back(obj->getModelMatrix());
    models_.emplace_back(obj->getModel());
    mats_.emplace_back(obj->getMaterial());
  }

  void orderChanged() {
    in_order_ = false;
  }

  void setScale(size_t i, const vec3& scale) {
    ts_[i].scale = scale;
    local_dirty_[i] = true;
    root_dirty_[i] = true;
  }
  vec3 getScale(size_t i) const {
    return ts_[i].scale;
  }

  void setRot(size_t i, glm::quat rot) {
    ts_[i].rot = rot;
    local_dirty_[i] = true;
    root_dirty_[i] = true;
  }
  glm::quat getRot(size_t i) const {
    return ts_[i].rot;
  }

  void setPos(size_t i, const vec3& pos) {
    ts_[i].pos = pos;
    local_dirty_[i] = true;
    root_dirty_[i] = true;
  }
  vec3 getPos(size_t i) const {
    return ts_[i].pos;
  }

  mat4 matrix(size_t i) const {
    return ts_[i].matrix();
  }

  void setMaterial(size_t i, MaterialId mat) {
    mats_[i] = mat;
  }

  void getModels(std::vector<std::pair<Object*, ModelId>>& pairs) {
    if (!in_order_) {
      order();
    }

    for (size_t i = 0; i < count_; i++) {
      pairs.emplace_back(objects_[i], models_[i]);
    }
  }

  void updateMats() {
    if (!in_order_) {
      order();
    }

    for (size_t i = 0; i < count_; i++) {
      mat4& local = local_ms_[i];
      if (local_dirty_[i]) {
        local = ts_[i].matrix();
      }

      size_t parent = parents_[i];
      mat4& root = root_ms_[i];
      if (parent == kRootParent) {
        root = local;
      } else if (root_dirty_[i] || root_dirty_[parent]) {
        fastMult(root_ms_[parent], local, root);
        root_dirty_[i] = true;  // If this updates, make sure children update.
      }
    }
    std::fill(local_dirty_.begin(), local_dirty_.end(), false);
    std::fill(root_dirty_.begin(), root_dirty_.end(), false);
  }

  void getSceneObjects(
      std::vector<SceneObject>& objs, const std::set<ModelId>& hidden) {
    for (size_t i = 0; i < count_; i++) {
      auto model = models_[i];
      if (model == ModelId::None || hidden.contains(model)) {
        continue;
      }

      mat4 temp;
      fastMult(root_ms_[i], model_ms_[i], temp);
      objs.emplace_back(model, mats_[i], temp);
    }
  }

 private:
  void order() {
    objects_.clear();
    for (auto* child : children_) {
      traverse(child);
    }
    count_ = objects_.size();

    parents_.clear();
    model_ms_.clear();
    models_.clear();
    mats_.clear();

    std::vector<TData> new_ts;
    new_ts.reserve(count_);
    std::vector<mat4> new_locals;
    new_locals.reserve(count_);
    std::vector<bool> new_local_dirtys;
    new_local_dirtys.reserve(count_);
    std::vector<mat4> new_roots;
    new_roots.reserve(count_);
    std::vector<bool> new_root_dirtys;
    new_root_dirtys.reserve(count_);

    for (size_t i = 0; i < count_; i++) {
      auto* obj = objects_[i];
      size_t prev_ind = obj->getObjectIndex();
      new_ts.emplace_back(ts_[prev_ind]);
      new_locals.emplace_back(local_ms_[prev_ind]);
      new_local_dirtys.emplace_back(local_dirty_[prev_ind]);
      new_roots.emplace_back(root_ms_[prev_ind]);
      new_root_dirtys.emplace_back(root_dirty_[prev_ind]);

      obj->setObjectIndex(i);
      auto* parent = obj->getParent();
      parents_.emplace_back(parent ? parent->getObjectIndex() : kRootParent);
      model_ms_.emplace_back(obj->getModelMatrix());
      models_.emplace_back(obj->getModel());
      mats_.emplace_back(obj->getMaterial());
    }

    ts_ = std::move(new_ts);
    local_ms_ = std::move(new_locals);
    local_dirty_ = std::move(new_local_dirtys);
    root_ms_ = std::move(new_roots);
    root_dirty_ = std::move(new_root_dirtys);
    in_order_ = true;
  }

  void traverse(Object* obj) {
    objects_.push_back(obj);
    for (auto* child : obj->children()) {
      traverse(child);
    }
  }

  const size_t kRootParent = -1;

  std::vector<Object*> children_;
  std::vector<std::unique_ptr<Object>> owned_children_;

  bool in_order_ = false;
  size_t count_ = 0;
  std::vector<Object*> objects_;
  std::vector<size_t> parents_;
  std::vector<TData> ts_;
  std::vector<mat4> local_ms_;
  std::vector<bool> local_dirty_;
  std::vector<mat4> root_ms_;
  std::vector<bool> root_dirty_;
  std::vector<mat4> model_ms_;
  std::vector<ModelId> models_;
  std::vector<MaterialId> mats_;
};
