#pragma once

#include <map>
#include <print>
#include <vector>

#include "asserts.h"
#include "entities.h"
#include "glm-include.h"
#include "object.h"
#include "transform.h"
#include "vec-maths.h"

// Stores data for all Objects in a flat structure. This makes iterating through
// them when flattening the tree much faster due to cache-friendly data access.
class WorldTree {
 public:
  WorldTree() {
  }

  // void addChild(Object* child) {
  //   children_.push_back(child);
  //   in_order_ = false;
  //   drawsDirty_ = true;
  // }

  // Object* makeObject(
  //     ModelId model = ModelId::None,
  //     std::optional<mat4> model_transform = std::nullopt) {
  //   auto unique_child = std::make_unique<Object>(this, model,
  //   model_transform); auto* ptr = unique_child.get();
  //   owned_children_.push_back(std::move(unique_child));
  //   addChild(ptr);
  //   return ptr;
  // }

  // void reg(Object* obj) {
  //   in_order_ = false;
  //   drawsDirty_ = true;

  //   size_t objInd = objects_.size();
  //   obj->setObjectIndex(objInd);
  //   objects_.emplace_back(obj);
  //   parents_.emplace_back(kRootParent);
  //   ts_.emplace_back();
  //   local_ms_.emplace_back(1.f);
  //   local_dirty_.emplace_back(true);
  //   root_ms_.emplace_back(1.f);
  //   root_dirty_.emplace_back(true);
  //   model_ms_.emplace_back(obj->getModelMatrix());
  //   obj_ms_.emplace_back(1.f);
  //   objDraws_.emplace_back(obj->getModel(), obj->getMaterial(), objInd);
  // }

  RangeId createRange(uint32_t count) {
    RangeInfo range{
        .rangeIndex = rangeObjectCount_,
        .count = count,
    };
    rangeObjectCount_ += count;
    rangeMatrices_.insert(rangeMatrices_.end(), count, mat4(1));

    rangeDraws_.resize(rangeObjectCount_);
    uint32_t offset = range.rangeIndex;
    for (uint32_t i = 0; i < count; i++) {
      rangeDraws_[i] = DrawData{ModelId::None, kMaterialIdNone, offset + i};
    }
    drawsDirty_ = true;

    auto id = nextRangeId_++;
    ranges_.emplace(id, range);
    return id;
  }

  void updateMatrices(RangeId id, std::vector<mat4> drawMats) {
    auto& range = getRange(id);
    DASSERT(drawMats.size() == range.count);

    for (size_t i = 0; i < range.count; i++) {
      rangeMatrices_[range.rangeIndex + i] = drawMats[i];
    }
  }

  void updateMaterials(RangeId id, MaterialId material) {
    auto& range = getRange(id);
    for (size_t i = 0; i < range.count; i++) {
      rangeDraws_[range.rangeIndex + i].material = material;
    }
    drawsDirty_ = true;
  }

  void updateModels(RangeId id, std::vector<ModelId> models) {
    auto& range = getRange(id);
    DASSERT(models.size() == range.count);

    for (size_t i = 0; i < range.count; i++) {
      rangeDraws_[range.rangeIndex + i].model = models[i];
    }
    drawsDirty_ = true;
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
    objDraws_[i].material = mat;
    drawsDirty_ = true;
  }

  void getModels(std::vector<std::pair<Object*, ModelId>>& pairs) {
    if (!in_order_) {
      order();
    }

    for (size_t i = 0; i < count_; i++) {
      pairs.emplace_back(objects_[i], objDraws_[i].model);
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

      if (local_dirty_[i] || root_dirty_[i]) {
        fastMult(root, model_ms_[i], obj_ms_[i]);
      }
    }
    std::fill(local_dirty_.begin(), local_dirty_.end(), false);
    std::fill(root_dirty_.begin(), root_dirty_.end(), false);
  }

  void getSceneObjects(
      std::vector<DrawData>& draws, std::vector<mat4>& objects,
      const std::set<ModelId>& hidden) {
    objects.reserve(obj_ms_.size() + rangeMatrices_.size());
    objects.insert(objects.end(), obj_ms_.begin(), obj_ms_.end());
    objects.insert(objects.end(), rangeMatrices_.begin(), rangeMatrices_.end());
  }

  std::vector<DrawData> getDraws() {
    size_t count = objDraws_.size() + rangeDraws_.size();

    std::vector<DrawData> draws;
    draws.reserve(count);
    draws.insert(draws.end(), objDraws_.begin(), objDraws_.end());
    draws.insert(draws.end(), rangeDraws_.begin(), rangeDraws_.end());

    for (size_t i = 0; i < count; i++) {
      draws[i].objInd = i;
    }

    return std::move(draws);
  }

  bool drawsDirty_ = true;

 private:
  void order() {
    objects_.clear();
    for (auto* child : children_) {
      traverse(child);
    }
    count_ = objects_.size();

    std::vector<size_t> prev_inds(count_);
    for (size_t i = 0; i < count_; i++) {
      auto* obj = objects_[i];
      prev_inds[i] = obj->getObjectIndex();
      obj->setObjectIndex(i);
    }

    reassign(ts_, prev_inds);
    reassign(local_ms_, prev_inds);
    reassign(local_dirty_, prev_inds);
    reassign(root_ms_, prev_inds);
    reassign(root_dirty_, prev_inds);
    reassign(model_ms_, prev_inds);
    reassign(parents_, prev_inds);

    // TODO: Properly deal with ranges.
    reassign(obj_ms_, prev_inds);
    reassign(objDraws_, prev_inds);
    drawsDirty_ = true;

    in_order_ = true;
  }

  template <class T>
  void reassign(
      std::vector<T>& oldValues, const std::vector<size_t>& prevInds) {
    auto count = prevInds.size();
    std::vector<T> newValues(count);
    for (size_t i = 0; i < count; i++) {
      newValues[i] = oldValues[prevInds[i]];
    }
    oldValues = std::move(newValues);
  }

  void traverse(Object* obj) {
    objects_.push_back(obj);
    for (auto* child : obj->children()) {
      traverse(child);
    }
  }

  struct RangeInfo {
    uint32_t rangeIndex;
    uint32_t count;
  };

  RangeInfo& getRange(RangeId id) {
    auto it = ranges_.find(id);
    DASSERT(it != ranges_.end());
    return it->second;
  }

  const size_t kRootParent = -1;

  std::vector<Object*> children_;
  std::vector<std::unique_ptr<Object>> owned_children_;

  RangeId nextRangeId_ = 0;
  std::map<RangeId, RangeInfo> ranges_;
  uint32_t rangeObjectCount_ = 0;
  std::vector<DrawData> rangeDraws_;
  std::vector<mat4> rangeMatrices_;

  bool in_order_ = false;
  uint32_t count_ = 0;
  std::vector<Object*> objects_;
  std::vector<uint32_t> parents_;
  std::vector<TData> ts_;
  std::vector<mat4> local_ms_;
  std::vector<bool> local_dirty_;
  std::vector<mat4> root_ms_;
  std::vector<bool> root_dirty_;
  std::vector<mat4> model_ms_;

  std::vector<mat4> obj_ms_;        // output object matrices
  std::vector<DrawData> objDraws_;  // output draws
};
