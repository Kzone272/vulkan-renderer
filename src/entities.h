#pragma once

#include <map>
#include <span>
#include <vector>

#include "glm-include.h"
#include "render-objects.h"

typedef uint32_t EntityId;
typedef uint32_t EntityIndex;
inline const uint32_t kNoEntry = -1;

typedef uint32_t RangeId;
inline const RangeId kRangeIdNone = -1;

struct Component {
  virtual void newEntity(EntityIndex i) = 0;
  virtual void deleteEntity(EntityIndex i) = 0;
  virtual void compress(const std::vector<EntityIndex>& prevInds) = 0;
};

struct UpdateComponent : public Component {
  using UpdateFn = std::function<void(float)>;

  virtual void newEntity(EntityIndex i) override;
  virtual void deleteEntity(EntityIndex i) override;
  virtual void compress(const std::vector<EntityIndex>& prevInds) override;

  void setUpdater(EntityIndex i, UpdateFn&& updater);
  void update(float deltaS);

  std::vector<uint32_t> inds_;
  std::vector<UpdateFn> updaters_;
};

struct RangeInfo {
  RangeId firstEntity;
  uint32_t count;
};

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

struct Entities {
  Entities() {
  }

  std::pair<EntityId, EntityIndex> newEntity();

  void compress();
  void resize(EntityIndex size);
  void init(EntityIndex i);

  EntityIndex getIndex(EntityId id);
  void deleteEntity(EntityId id);

  EntityId makeObject(
      ModelId model = ModelId::None, MaterialId material = kMaterialIdNone,
      const std::optional<mat4>& modelMatrix = std::nullopt);

  void setModel(EntityId id, ModelId model);
  void setMaterial(EntityId id, MaterialId material);
  void setModelMatrix(EntityId id, const mat4& matrix);
  TData& getTransform(EntityId id);
  void setPos(EntityId id, const vec3& pos);
  const vec3& getPos(EntityId id);
  const mat4& getDrawMatrix(EntityId id);
  void setParent(EntityId child, EntityId parent);

  void updateMats();

  void setUpdater(EntityId id, UpdateComponent::UpdateFn&& updater);
  void update(float deltaS);

  RangeId createRange(uint32_t count);
  RangeInfo& getRange(RangeId id);
  void deleteRange(RangeId id);

  void updateMatrices(RangeId id, std::vector<mat4> drawMats);
  void setSkipTransform(RangeId id, bool value);
  void updateMaterials(RangeId id, MaterialId material);
  void updateModels(RangeId id, std::vector<ModelId> models);
  std::span<TData> getTransforms(RangeId id);

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

  UpdateComponent update_;
  std::vector<Component*> components_{&update_};
};
