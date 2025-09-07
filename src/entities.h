#pragma once

#include <map>
#include <span>
#include <unordered_map>
#include <vector>

#include "glm-include.h"
#include "render-objects.h"
#include "time-sampler.h"

typedef uint32_t EntityIndex;
inline const uint32_t kNoEntry = -1;

template <class T>
struct IdType {
  IdType() : value(kNoEntry) {
  }
  explicit IdType(uint32_t id) : value(id) {
  }

  operator uint32_t() {
    return value;
  }

  bool operator==(const IdType<T>& other) const {
    return value == other.value;
  }
  bool operator<(const IdType<T>& other) const {
    return value < other.value;
  }

  uint32_t value = kNoEntry;
};

template<class T>
struct std::hash<IdType<T>> {
  size_t operator()(const IdType<T>& id) const {
    return std::hash<uint32_t>()(id.value);
  }
};

struct EntityTag{};
using EntityId = IdType<EntityTag>;

struct RangeTag{};
using RangeId = IdType<RangeTag>;


struct Component {
  virtual void newEntity(EntityId id) = 0;
  virtual void deleteEntity(EntityId id) = 0;
  virtual void compress() = 0;
};

struct UpdateComponent : public Component {
  using UpdateFn = std::function<void(float)>;

  virtual void newEntity(EntityId id) override;
  virtual void deleteEntity(EntityId id) override;
  virtual void compress() override;

  void setUpdater(EntityId id, UpdateFn&& updater);
  void update(float deltaS);
  TimeSampler& getTime(EntityId id);

  std::unordered_map<EntityId, uint32_t> inds_;
  std::vector<UpdateFn> updaters_;
  std::vector<TimeSampler> times_;
};

struct RangeInfo {
  EntityId firstEntity;
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
  void setDrawMatrix(EntityId id, const mat4& matrix);
  const mat4& getDrawMatrix(EntityId id);
  void setParent(EntityId child, EntityId parent);
  void setSkipTransform(EntityId id, bool value);

  void updateMats();

  void setUpdater(EntityId id, UpdateComponent::UpdateFn&& updater);
  void update(float deltaS);
  TimeSampler& getTime(EntityId id);

  RangeId createRange(uint32_t count);
  RangeInfo& getRange(RangeId id);
  void deleteRange(RangeId id);

  void updateMatrices(RangeId id, std::vector<mat4> drawMats);
  void setSkipTransform(RangeId id, bool value);
  void updateMaterials(RangeId id, MaterialId material);
  void updateModels(RangeId id, std::vector<ModelId> models);
  std::span<TData> getTransforms(RangeId id);

  std::map<EntityId, EntityIndex> entityIndices_;
  uint32_t nextId_ = 0;
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
