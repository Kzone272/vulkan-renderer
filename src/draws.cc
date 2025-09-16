#include "draws.h"

#include "materials.h"

void Draws::init(const VulkanState& vs, Materials* mats) {
  mats_ = mats;
  objectBuf_ = createDynamicBuffer(
      vs, kMaxObjects * sizeof(ObjectData),
      vk::BufferUsageFlagBits::eStorageBuffer);
  transformBuf_ = createDynamicBuffer(
      vs, kMaxObjects * sizeof(mat4), vk::BufferUsageFlagBits::eStorageBuffer);
}

void Draws::update(const DrawState& ds, const FrameState& fs) {
  updateDynamicBuf(
      ds, transformBuf_, std::span(fs.transforms),
      vk::PipelineStageFlagBits::eVertexShader,
      vk::AccessFlagBits::eShaderRead);

  if (!fs.drawsNeedUpdate) {
    return;
  }

  // Gather valid draw calls.
  std::vector<DrawCall> drawCalls;
  drawCalls.reserve(fs.draws.size());
  for (size_t i = 0; i < fs.draws.size(); i++) {
    auto& draw = fs.draws[i];
    if (draw.material == kMaterialIdNone || draw.model == ModelId::None) {
      continue;
    }

    auto matPipeline = mats_->getPipeline(draw.material);
    auto matDesc = mats_->getDesc(draw.material);

    drawCalls.emplace_back(
        draw.material, matPipeline, matDesc, draw.model, draw.objInd);
  }

  // Sort by material, then by model.
  std::sort(drawCalls.begin(), drawCalls.end(), [](auto& left, auto& right) {
    if (left.matPipeline != right.matPipeline) {
      return left.matPipeline < right.matPipeline;
    } else if (left.matDesc != right.matDesc) {
      return left.matDesc < right.matDesc;
    } else {
      return left.model < right.model;
    }
  });

  std::vector<ObjectData> objects(drawCalls.size());
  for (size_t i = 0; i < drawCalls.size(); i++) {
    objects[i].index = drawCalls[i].objInd;
    objects[i].matIndex = drawCalls[i].material;
  }

  instDraws_.clear();
  for (uint32_t i = 0; i < drawCalls.size(); i++) {
    auto& draw = drawCalls[i];

    bool isNew = true;
    if (instDraws_.size()) {
      auto& lastDraw = instDraws_.back();
      if (lastDraw.matPipeline == draw.matPipeline &&
          lastDraw.matDesc == draw.matDesc && lastDraw.model == draw.model) {
        lastDraw.instances++;
        isNew = false;
      }
    }

    if (isNew) {
      instDraws_.emplace_back(i, 1, draw.matPipeline, draw.matDesc, draw.model);
    }
  }

  updateDynamicBuf(
      ds, objectBuf_, std::span(objects),
      vk::PipelineStageFlagBits::eVertexShader,
      vk::AccessFlagBits::eShaderRead);
}
