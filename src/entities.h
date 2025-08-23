#pragma once

#include <array>
#include <map>
#include <variant>
#include <vector>

#include "asserts.h"
#include "glm-include.h"
#include "world-tree.h"

typedef uint32_t EntityId;
typedef uint32_t EntityIndex;

inline const uint32_t kNoEntry = -1;

struct Component {
  virtual void newEntity(EntityIndex i) = 0;
  virtual void deleteEntity(EntityIndex i) = 0;

  std::vector<EntityIndex> indices_;
};

// BEGIN EXAMPLES

struct FooData {
  int a;
  int b;
};

void eraseIndex(EntityIndex i, auto& list) {
  std::swap(list[i], list[list.size() - 1]);
  list.erase(list.end() - 1);
}

struct FooComponent : public Component {
  void setFoo(EntityIndex i, FooData&& foo) {
    if (indices_[i] == kNoEntry) {
      indices_[i] = foos_.size();
      foos_.emplace_back(foo);
    } else {
      foos_[indices_[i]] = foo;
    }
  }

  FooData& getFoo(EntityIndex i) {
    DASSERT(indices_[i] != kNoEntry);
    return foos_[indices_[i]];
  }

  void deleteEntity(EntityIndex i) override {
    if (indices_[i] != kNoEntry) {
      eraseIndex(i, foos_);
      indices_[i] = kNoEntry;
    }
  }

  std::vector<FooData> foos_;
};

struct BarComponent : public Component {
  int bar_;

  void deleteEntity(EntityIndex i) override {
    indices_[i] = kNoEntry;
  }
};

struct Updater {
  virtual void update(float deltaS) = 0;
};

struct UpdaterA {
  void update(float deltaS) {
    acc += deltaS;
  }
  float acc = 0;
};

struct UpdaterB {
  void update(float deltaS) {
    acc += deltaS;
    val = sin(acc);
  }
  float acc = 0;
  float val = 0;
};

struct UpdaterComponent : public Component {
  using UpdaterType = std::variant<UpdaterA, UpdaterB>;

  void addUpdater(EntityIndex i, UpdaterType&& updater) {
    indices_[i] = updaters_.size();
    updaters_.emplace_back(updater);
  }

  void updateAll(float deltaS) {
    for (auto& updater : updaters_) {
      std::visit(
          [deltaS](auto& typedUpdater) { typedUpdater.update(deltaS); },
          updater);
    }
  }

  void deleteEntity(EntityIndex i) override {
    if (indices_[i] != kNoEntry) {
      eraseIndex(i, updaters_);
      indices_[i] = kNoEntry;
    }
  }

  std::vector<UpdaterType> updaters_;
};

// END EXAMPLES

// struct TransformComponent : public Component {
//   void add(EntityIndex i) {
//     indices_[i] = ts_.size();
//   }

//   TData& getTransform(EntityIndex i) {
//     auto tInd = indices_[i];
//     localDirty_[tInd] = true;
//     rootDirty_[tInd] = true;
//     return ts_[tInd];
//   }

//   void deleteEntity(EntityIndex i) override {
//     auto tInd = indices_[i];
//     if (tInd != kNoEntry) {
//       dataIndices_[tInd] = kNoEntry;
//       indices_[i] = kNoEntry;
//     }
//   }
// };

// struct DrawComponent : public Component {
//   void add(EntityIndex i) {
//     indices_[i] = draws_.size();

//     dataIndices_.emplace_back(i);
//     draws_.emplace_back(ModelId::None, kMaterialIdNone, i);
//     dirty_ = true;
//   }

//   void deleteEntity(EntityIndex i) override {
//     auto dInd = indices_[i];
//     if (dInd != kNoEntry) {
//       std::swap(draws_[dInd], draws_.back());
//       std::swap(dataIndices_[dInd], dataIndices_.back());
//       draws_.erase(draws_.end() - 1);
//       auto moved = dataIndices_[dInd];
//       dataIndices_.erase(dataIndices_.end() - 1);
//       indices_[moved] = moved;
//       // dataIndices_[dInd] = kNoEntry;
//       indices_[i] = kNoEntry;
//     }
//   }
// };

struct Entities {
  Entities() {
  }

  const uint32_t ALLOC_BATCH = 100;

  std::pair<EntityId, EntityIndex> newEntity() {
    EntityId id = nextId_++;
    EntityIndex index = nextIndex_++;
    count_++;
    entityIndices_.emplace(id, index);

    if (index >= valid_.size()) {
      resize(valid_.size() + ALLOC_BATCH);
    } else {
      reset(index);
    }
    valid_[index] = true;
    drawsDirty_ = true;

    for (auto* component : components_) {
      component->newEntity(index);
    }

    return std::make_pair(id, index);
  }

  void compress() {
    if (valid_.size() - count_ <= ALLOC_BATCH) {
      return;
    }
    
    std::vector<EntityIndex> prevInds;
    std::vector<EntityIndex> newInds(valid_.size());
    for (uint32_t i = 0; i < valid_.size(); i++) {
      if (valid_[i]) {
        newInds[i] = prevInds.size();
        prevInds.push_back(i);
      }
    }
    nextIndex_ = prevInds.size();
    // std::println("entities size: {}", nextIndex_); 

    // Update EntityId mappings.
    for (auto& entry : entityIndices_) {
      entry.second = newInds[entry.second];
    }
    
    // Update storage:
    reassign(valid_, prevInds);
    // Transform
    reassign(skipTransform_, prevInds);
    reassign(ts_, prevInds);
    reassign(localMs_, prevInds);
    reassign(localDirty_, prevInds);
    reassign(rootMs_, prevInds);
    reassign(rootDirty_, prevInds);
    reassign(modelMs_, prevInds);
    reassign(drawMs_, prevInds);
    reassign(parents_, prevInds);
    // Update parent indices
    for (size_t i = 0; i < parents_.size(); i++) {
      auto parent = parents_[i];
      if (parent != kNoEntry) {
        parents_[i] = newInds[parent];
      }
    }
    
    // Draw
    reassign(draws_, prevInds);
    for (size_t i = 0; i < draws_.size(); i++) {
      draws_[i].objInd = i;
    }
    drawsDirty_ = true;

  }

  template <class T>
  void reassign(
      std::vector<T>& storage, const std::vector<EntityIndex>& prevInds) {
    auto count = prevInds.size();
    for (uint32_t i = 0; i < count; i++) {
      storage[i] = storage[prevInds[i]];
    }
    storage.resize(count);
  }

  void resize(EntityIndex size) {
    valid_.resize(size, false);
    // Transform
    skipTransform_.resize(size, false);
    ts_.resize(size);
    localMs_.resize(size, mat4(1));
    localDirty_.resize(size, true);
    rootMs_.resize(size, mat4(1));
    rootDirty_.resize(size, true);
    modelMs_.resize(size, mat4(1));
    drawMs_.resize(size, mat4(1));
    parents_.resize(size, kNoEntry);
    // Draw
    draws_.resize(size);
  }

  void reset(EntityIndex i) {
    valid_[i] = false;
    // Transform
    skipTransform_[i] = false;
    ts_[i] = {};
    localMs_[i] = mat4(1.f);
    localDirty_[i] = true;
    rootMs_[i] = mat4(1.f);
    rootDirty_[i] = true;
    modelMs_[i] = mat4(1.f);
    drawMs_[i] = mat4(1.f);
    parents_[i] = kNoEntry;
    // Draw
    draws_[i] = {.objInd = i};
  }

  EntityIndex getIndex(EntityId id) {
    auto it = entityIndices_.find(id);
    ASSERT(it != entityIndices_.end());
    return it->second;
  }

  void deleteEntity(EntityId id) {
    auto index = getIndex(id);
    for (auto* component : components_) {
      component->deleteEntity(index);
    }
    entityIndices_.erase(id);
    count_--;
    valid_[index] = false;
    draws_[index].model = ModelId::None;
    drawsDirty_ = true;
  }

  EntityId makeObject(
      ModelId model = ModelId::None, MaterialId material = kMaterialIdNone,
      std::optional<mat4> modelMatrix = std::nullopt) {
    auto entPair = newEntity();
    auto [id, i] = entPair;

    if (modelMatrix) {
      modelMs_[i] = *modelMatrix;
    }
    draws_[i] = {model, material, i};

    return id;
  }

  void setModel(EntityId id, ModelId model) {
    auto i = getIndex(id);
    DASSERT(valid_[i]);
    draws_[i].model = model;
    drawsDirty_ = true;
  }

  void setMaterial(EntityId id, MaterialId material) {
    auto i = getIndex(id);
    DASSERT(valid_[i]);
    draws_[i].material = material;
    drawsDirty_ = true;
  }

  TData& getTransform(EntityId id) {
    auto i = getIndex(id);
    DASSERT(valid_[i]);
    localDirty_[i] = true;
    rootDirty_[i] = true;
    return ts_[i];
  }

  const vec3& getPos(EntityId id) {
    auto i = getIndex(id);
    DASSERT(valid_[i]);
    return ts_[i].pos;
  }

  void setPos(EntityId id, const vec3& pos) {
    auto i = getIndex(id);
    DASSERT(valid_[i]);
    ts_[i].pos = pos;
    localDirty_[i] = true;
    rootDirty_[i] = true;
  }

  const mat4& getDrawMatrix(EntityId id) {
    auto i = getIndex(id);
    DASSERT(valid_[i]);
    return drawMs_[i];
  }

  void setParent(EntityId child, EntityId parent) {
    auto i = getIndex(child);
    DASSERT(valid_[i]);
    parents_[i] = getIndex(parent);
  }

  void updateMats() {
    auto count = ts_.size();
    for (size_t i = 0; i < count; i++) {
      if (!valid_[i] || skipTransform_[i]) {
        continue;
      }

      mat4& local = localMs_[i];
      if (localDirty_[i]) {
        local = ts_[i].matrix();
      }

      size_t parent = parents_[i];
      mat4& root = rootMs_[i];
      if (parent == kNoEntry) {
        root = local;
      } else if (rootDirty_[i] || rootDirty_[parent]) {
        fastMult(rootMs_[parent], local, root);
        rootDirty_[i] = true;  // If this updates, make sure children update.
      }

      if (localDirty_[i] || rootDirty_[i]) {
        fastMult(root, modelMs_[i], drawMs_[i]);
      }
    }
    std::fill(localDirty_.begin(), localDirty_.end(), false);
    std::fill(rootDirty_.begin(), rootDirty_.end(), false);
  }

  // Begin In progress:

  struct RangeInfo {
    RangeId firstEntity;
    uint32_t count;
  };

  RangeId createRange(uint32_t count) {
    RangeInfo range{
        .count = count,
    };
    for (uint32_t i = 0; i < count; i++) {
      auto [id, index] = newEntity();
      if (i == 0) {
        range.firstEntity = id;
      }
    }

    ranges_.emplace(range.firstEntity, range);
    return range.firstEntity;
  }

  RangeInfo& getRange(RangeId id) {
    auto it = ranges_.find(id);
    DASSERT(it != ranges_.end());
    return it->second;
  }

  void updateMatrices(RangeId id, std::vector<mat4> drawMats) {
    auto& range = getRange(id);
    DASSERT(drawMats.size() == range.count);
    auto start = getIndex(range.firstEntity);

    for (size_t i = 0; i < range.count; i++) {
      drawMs_[start + i] = drawMats[i];
    }
  }

  void setSkipTransform(RangeId id, bool value) {
    auto& range = getRange(id);
    auto start = getIndex(range.firstEntity);

    for (size_t i = 0; i < range.count; i++) {
      skipTransform_[start + i] = value;
    }
  }

  void updateMaterials(RangeId id, MaterialId material) {
    auto& range = getRange(id);
    auto start = getIndex(range.firstEntity);

    for (size_t i = 0; i < range.count; i++) {
      draws_[start + i].material = material;
    }
    drawsDirty_ = true;
  }

  void updateModels(RangeId id, std::vector<ModelId> models) {
    auto& range = getRange(id);
    auto start = getIndex(range.firstEntity);

    for (size_t i = 0; i < range.count; i++) {
      draws_[start + i].model = models[i];
    }
    drawsDirty_ = true;
  }

  // End in-progress

  std::map<EntityId, EntityIndex> entityIndices_;
  EntityId nextId_ = 0;
  EntityIndex nextIndex_ = 0;
  uint32_t count_ = 0;

  // Contiguous storage ranges
  std::map<RangeId, RangeInfo> ranges_;

  // # All entity storage
  std::vector<bool> valid_;
  // ## Transform
  std::vector<bool> skipTransform_;
  std::vector<TData> ts_;
  std::vector<mat4> localMs_;
  std::vector<bool> localDirty_;
  std::vector<mat4> rootMs_;
  std::vector<bool> rootDirty_;
  std::vector<mat4> modelMs_;
  std::vector<mat4> drawMs_;  // output object matrices
  std::vector<EntityIndex> parents_;
  // ## Draw
  bool drawsDirty_ = true;
  std::vector<DrawData> draws_;  // output draws

  // FooComponent foo_;
  // BarComponent bar_;
  // UpdaterComponent upd_;
  // TransformComponent transform_;
  // DrawComponent draw_;
  std::vector<Component*> components_{};
  // std::vector<Component*> components_{&transform_, &draw_};
  // &foo_, &bar_, &upd_, &transform_, &draw_};
};
