#include "descriptors.h"

#include <memory>
#include <print>
#include <variant>
#include <vector>

#include "asserts.h"
#include "render-state.h"

vk::WriteDescriptorSet bindWrite(
    const vk::DescriptorSet& set, const Binding& bind) {
  return {
      .dstSet = set,
      .dstBinding = bind.index,
      .dstArrayElement = 0,
      .descriptorType = bind.type,
  };
}

vk::WriteDescriptorSet uboWrite(
    const vk::DescriptorSet& set, const Binding& bind,
    vk::DescriptorBufferInfo* info) {
  auto write = bindWrite(set, bind);
  write.setBufferInfo(*info);
  return write;
}

vk::WriteDescriptorSet samplerWrite(
    const vk::DescriptorSet& set, const Binding& bind,
    vk::DescriptorImageInfo* info) {
  auto write = bindWrite(set, bind);
  write.setImageInfo(*info);
  return write;
}

void updateDescSet(
    const vk::DescriptorSet& set, const DescLayout& layout,
    DescSetUpdates updates, std::vector<vk::WriteDescriptorSet>& writes) {
  ASSERT(layout.binds.size() == updates.size());
  for (uint32_t i = 0; i < updates.size(); i++) {
    auto& update = updates[i];
    auto& bind = layout.binds[i];
    switch (bind.type) {
      case vk::DescriptorType::eUniformBuffer:
      case vk::DescriptorType::eStorageBuffer:
        DASSERT(std::holds_alternative<vk::DescriptorBufferInfo*>(update));
        writes.push_back(
            uboWrite(set, bind, std::get<vk::DescriptorBufferInfo*>(update)));
        break;
      case vk::DescriptorType::eCombinedImageSampler:
        DASSERT(std::holds_alternative<vk::DescriptorImageInfo*>(update));
        writes.push_back(samplerWrite(
            set, bind, std::get<vk::DescriptorImageInfo*>(update)));
        break;
      default:
        std::println("Unsupported descriptor type! {}", (int)bind.type);
        ASSERT(false);
    }
  }
}

std::vector<vk::DescriptorSet> allocDescSets(
    const VulkanState& vs, vk::DescriptorSetLayout layout, size_t count) {
  std::vector<vk::DescriptorSetLayout> layouts(count, layout);
  vk::DescriptorSetAllocateInfo ai{
      .descriptorPool = vs.desc_pool,
  };
  ai.setSetLayouts(layouts);

  auto desc_sets = vs.device.allocateDescriptorSets(ai).value;
  DASSERT(desc_sets.size() == count);

  return desc_sets;
}

vk::DescriptorSet allocDescSet(
    const VulkanState& vs, vk::DescriptorSetLayout layout) {
  auto desc_sets = allocDescSets(vs, layout, 1);
  return desc_sets[0];
}

void DescLayout::init(const VulkanState& vs) {
  uint32_t i = 0;
  std::vector<vk::DescriptorSetLayoutBinding> bindings;
  for (auto& bind : binds) {
    bind.index = i++;
    bindings.push_back({
        .binding = bind.index,
        .descriptorType = bind.type,
        .descriptorCount = bind.count,
        .stageFlags = stages,
    });
  }

  vk::DescriptorSetLayoutCreateInfo layout_ci{};
  layout_ci.setBindings(bindings);
  layout = vs.device.createDescriptorSetLayoutUnique(layout_ci).value;
}

void DescLayout::alloc(const VulkanState& vs, int count) {
  sets = allocDescSets(vs, *layout, count);
}

void DescLayout::updateUboBind(
    uint32_t index, const std::vector<vk::DescriptorBufferInfo*>& infos,
    std::vector<vk::WriteDescriptorSet>& writes) {
  DASSERT(sets.size() == infos.size());
  auto& bind = binds[index];
  DASSERT(
      bind.type == vk::DescriptorType::eUniformBuffer ||
      bind.type == vk::DescriptorType::eStorageBuffer);

  for (uint32_t i = 0; i < sets.size(); i++) {
    writes.push_back(uboWrite(sets[i], bind, infos[i]));
  }
}

void DescLayout::updateSamplerBind(
    uint32_t index, vk::DescriptorImageInfo* info,
    std::vector<vk::WriteDescriptorSet>& writes) {
  auto& bind = binds[index];
  DASSERT(bind.type == vk::DescriptorType::eCombinedImageSampler);

  for (auto& set : sets) {
    writes.push_back(samplerWrite(set, bind, info));
  }
}

void DescLayout::updateSamplersBind(
    uint32_t index, const std::vector<vk::DescriptorImageInfo>& infos,
    std::vector<vk::WriteDescriptorSet>& writes) {
  auto& bind = binds[index];
  DASSERT(bind.type == vk::DescriptorType::eCombinedImageSampler);
  DASSERT(bind.count == infos.size());

  for (auto& set : sets) {
    auto write = bindWrite(set, bind);
    write.setImageInfo(infos);
    writes.push_back(write);
  }
}
