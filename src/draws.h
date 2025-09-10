#pragma once

#include "render-objects.h"
#include "render-objects.h"
#include "buffers.h"

struct Materials;

struct Draws {

  void init(const VulkanState& vs, Materials* mats);
  void update(const DrawState& ds, const FrameState& fs);
  
  struct DrawCall {
    MaterialId material = kMaterialIdNone;
    ScenePipeline matPipeline = ScenePipeline::Basic;
    vk::DescriptorSet matDesc = {};
    ModelId model = ModelId::None;
    uint32_t objInd = -1;
  };

  struct InstanceDraws {
    uint32_t firstInstance = 0;
    uint32_t instances = 0;
    ScenePipeline matPipeline = ScenePipeline::Basic;
    vk::DescriptorSet matDesc = {};
    ModelId model = ModelId::None;
  };
  std::vector<InstanceDraws> instDraws_;

  Materials* mats_ = nullptr;

  DynamicBuf transformBuf_;
  DynamicBuf objectBuf_;
};
