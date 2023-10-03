#pragma once

#include "descriptors.h"

#include <memory>
#include <variant>
#include <vector>

#include "asserts.h"
#include "strings.h"
#include "vulkan-include.h"

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
        log("Unsupported descriptor type! {}", (int)bind.type);
        ASSERT(false);
    }
  }
}

std::vector<vk::DescriptorSet> allocDescSets(
    vk::Device device, vk::DescriptorPool pool, vk::DescriptorSetLayout layout,
    size_t count) {
  std::vector<vk::DescriptorSetLayout> layouts(count, layout);
  vk::DescriptorSetAllocateInfo ai{
      .descriptorPool = pool,
  };
  ai.setSetLayouts(layouts);

  auto desc_sets = device.allocateDescriptorSets(ai).value;
  DASSERT(desc_sets.size() == count);

  return desc_sets;
}

vk::DescriptorSet allocDescSet(
    vk::Device device, vk::DescriptorPool pool,
    vk::DescriptorSetLayout layout) {
  auto desc_sets = allocDescSets(device, pool, layout, 1);
  return desc_sets[0];
}

void DescLayout::init(vk::Device& device) {
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
  layout = device.createDescriptorSetLayoutUnique(layout_ci).value;
}

void DescLayout::alloc(
    vk::Device& device, vk::DescriptorPool& pool, int count) {
  sets = allocDescSets(device, pool, *layout, count);
}

void DescLayout::updateUboBind(
    uint32_t index, const std::vector<vk::DescriptorBufferInfo*>& infos,
    std::vector<vk::WriteDescriptorSet>& writes) {
  DASSERT(sets.size() == infos.size());
  auto& bind = binds[index];
  DASSERT(bind.type == vk::DescriptorType::eUniformBuffer);

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
