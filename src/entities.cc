#include "entities.h"

#include <variant>

#include "asserts.h"
#include "vec-maths.h"

// BEGIN EXAMPLES

// struct Updater {
//   virtual void update(float deltaS) = 0;
// };

// struct UpdaterA {
//   void update(float deltaS) {
//     acc += deltaS;
//   }
//   float acc = 0;
// };

// struct UpdaterB {
//   void update(float deltaS) {
//     acc += deltaS;
//     val = sin(acc);
//   }
//   float acc = 0;
//   float val = 0;
// };

// struct UpdateComponent : public Component {
//   using UpdaterType = std::variant<UpdaterA, UpdaterB>;

//   void addUpdater(EntityIndex i, UpdaterType&& updater) {
//     indices_[i] = updaters_.size();
//     updaters_.emplace_back(updater);
//   }

//   void updateAll(float deltaS) {
//     for (auto& updater : updaters_) {
//       std::visit(
//           [deltaS](auto& typedUpdater) { typedUpdater.update(deltaS); },
//           updater);
//     }
//   }

//   void deleteEntity(EntityIndex i) override {
//     if (indices_[i] != kNoEntry) {
//       eraseIndex(i, updaters_);
//       indices_[i] = kNoEntry;
//     }
//   }

//   std::vector<UpdaterType> updaters_;
// };

// END EXAMPLES

const uint32_t ALLOC_BATCH = 100;

std::pair<EntityId, EntityIndex> Entities::newEntity() {
  EntityId id = nextId_++;
  EntityIndex index = nextIndex_++;
  count_++;
  entityIndices_.emplace(id, index);

  if (index >= valid_.size()) {
    resize(valid_.size() + ALLOC_BATCH);
  }
  init(index);
  drawsDirty_ = true;

  for (auto* component : components_) {
    component->newEntity(id);
  }

  return std::make_pair(id, index);
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

void Entities::compress() {
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

  for (auto* component : components_) {
    component->compress();
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

void Entities::resize(EntityIndex size) {
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

void Entities::init(EntityIndex i) {
  valid_[i] = true;
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

EntityIndex Entities::getIndex(EntityId id) {
  auto it = entityIndices_.find(id);
  ASSERT(it != entityIndices_.end());
  return it->second;
}

void Entities::deleteEntity(EntityId id) {
  auto index = getIndex(id);
  for (auto* component : components_) {
    component->deleteEntity(id);
  }
  entityIndices_.erase(id);
  count_--;
  valid_[index] = false;
  draws_[index].model = ModelId::None;
  drawsDirty_ = true;
}

EntityId Entities::makeObject(
    ModelId model, MaterialId material,
    const std::optional<mat4>& modelMatrix) {
  auto entPair = newEntity();
  auto [id, i] = entPair;

  if (modelMatrix) {
    modelMs_[i] = *modelMatrix;
  }
  draws_[i].model = model;
  draws_[i].material = material;

  return id;
}

void Entities::setModel(EntityId id, ModelId model) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  draws_[i].model = model;
  drawsDirty_ = true;
}

void Entities::setMaterial(EntityId id, MaterialId material) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  draws_[i].material = material;
  drawsDirty_ = true;
}

void Entities::setModelMatrix(EntityId id, const mat4& matrix) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  modelMs_[i] = matrix;
}

TData& Entities::getTransform(EntityId id) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  localDirty_[i] = true;
  rootDirty_[i] = true;
  return ts_[i];
}

const vec3& Entities::getPos(EntityId id) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  return ts_[i].pos;
}

void Entities::setPos(EntityId id, const vec3& pos) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  ts_[i].pos = pos;
  localDirty_[i] = true;
  rootDirty_[i] = true;
}

const mat4& Entities::getDrawMatrix(EntityId id) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  return drawMs_[i];
}

void Entities::setParent(EntityId child, EntityId parent) {
  auto i = getIndex(child);
  DASSERT(valid_[i]);
  parents_[i] = getIndex(parent);
}

void Entities::updateMats() {
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

void Entities::setUpdater(EntityId id, UpdateComponent::UpdateFn&& updater) {
  auto i = getIndex(id);
  DASSERT(valid_[i]);
  update_.setUpdater(id, std::move(updater));
}

void Entities::update(float deltaS) {
  update_.update(deltaS);
}

RangeId Entities::createRange(uint32_t count) {
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

RangeInfo& Entities::getRange(RangeId id) {
  auto it = ranges_.find(id);
  DASSERT(it != ranges_.end());
  return it->second;
}

void Entities::deleteRange(RangeId id) {
  auto it = ranges_.find(id);
  if (it == ranges_.end()) {
    return;
  }

  auto& range = it->second;
  for (uint32_t i = 0; i < range.count; i++) {
    deleteEntity(range.firstEntity + i);
  }
  ranges_.erase(it);
}

void Entities::updateMatrices(RangeId id, std::vector<mat4> drawMats) {
  auto& range = getRange(id);
  DASSERT(drawMats.size() == range.count);
  auto start = getIndex(range.firstEntity);

  for (size_t i = 0; i < range.count; i++) {
    drawMs_[start + i] = drawMats[i];
  }
}

void Entities::setSkipTransform(RangeId id, bool value) {
  auto& range = getRange(id);
  auto start = getIndex(range.firstEntity);

  for (size_t i = 0; i < range.count; i++) {
    skipTransform_[start + i] = value;
  }
}

void Entities::updateMaterials(RangeId id, MaterialId material) {
  auto& range = getRange(id);
  auto start = getIndex(range.firstEntity);

  for (size_t i = 0; i < range.count; i++) {
    draws_[start + i].material = material;
  }
  drawsDirty_ = true;
}

void Entities::updateModels(RangeId id, std::vector<ModelId> models) {
  auto& range = getRange(id);
  auto start = getIndex(range.firstEntity);

  for (size_t i = 0; i < range.count; i++) {
    draws_[start + i].model = models[i];
  }
  drawsDirty_ = true;
}

std::span<TData> Entities::getTransforms(RangeId id) {
  auto& range = getRange(id);
  auto start = getIndex(range.firstEntity);
  for (uint32_t i = 0; i < range.count; i++) {
    localDirty_[start + i] = true;
    rootDirty_[start + i] = true;
  }

  return std::span(ts_.begin() + start, range.count);
}

// # UpdateComponent

void UpdateComponent::newEntity(EntityId id) {
}

void UpdateComponent::deleteEntity(EntityId id) {
  auto it = inds_.find(id);
  if (it != inds_.end()) {
    updaters_[it->second] = nullptr;
    inds_.erase(it);
  }
}

void UpdateComponent::compress() {
  if (updaters_.size() == inds_.size()) {
    return;
  }

  std::vector<UpdateFn> newUpdaters;
  newUpdaters.reserve(updaters_.size());

  for (auto& [id, ind] : inds_) {
    newUpdaters.push_back(std::move(updaters_[ind]));
    ind = newUpdaters.size() - 1;
  }
  updaters_ = std::move(newUpdaters);
}

void UpdateComponent::setUpdater(EntityId id, UpdateFn&& updater) {
  auto it = inds_.find(id);
  if (it != inds_.end()) {
    updaters_[it->second] = updater;
  } else {
    inds_.emplace(id, static_cast<uint32_t>(updaters_.size()));
    updaters_.push_back(updater);
  }
}

void UpdateComponent::update(float deltaS) {
  for (auto& updater : updaters_) {
    if (!updater) {
      continue;
    }
    updater(deltaS);
  }
}
