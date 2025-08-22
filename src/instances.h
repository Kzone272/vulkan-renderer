#pragma once

#include "render-objects.h"

typedef uint32_t ObjectId;
inline const uint32_t kObjectIdNone = -1;

typedef uint32_t InstanceId;
inline const uint32_t kInstanceIdNone = -1;

class Instances {
 public:
  InstanceId getInstance(int objId) {
    auto it = objs_.find(objId);
    DASSERT(it != objs_.end());
    return it->second;
  }

  void setInstance(InstanceId instance, ModelId model, MaterialId material) {
    models_[instance] = model;
    mats_[instance] = material;
  }

  // void setInstances(
  //     InstanceId firstInstance, const std::vector<ModelId>& models,
  //     const std::vector<MaterialId>& mats) {
  //   DASSERT(models.size() == mats.size());
  //   DASSERT(firstInstance + models.size() < models_.size());

  //   std::copy(models.first(), models.end(), models_.begin() + firstInstance);
  //   std::copy(mats.first(), mats.end(), mats_.begin() + firstInstance);
  //   // for (InstanceId i = firstInstance; i < models.size(); i++) {
  //   //   models_[i] = models[i];
  //   //   mats_[i] = materials[i];
  //   // }
  // }

 private:
  std::map<ObjectId, InstanceId> objs_;
  std::vector<ModelId> models_;
  std::vector<MaterialId> mats_;
};
