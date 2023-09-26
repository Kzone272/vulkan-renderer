#pragma once

#include <memory>
#include <variant>
#include <vector>

#include "assert.h"
#include "render-objects.h"
#include "strings.h"
#include "vulkan-include.h"

struct Binding {
  vk::DescriptorType type;
  uint32_t count = 1;
};

struct DescLayout {
  std::vector<Binding> binds;
  vk::ShaderStageFlags stages;
  vk::UniqueDescriptorSetLayout layout;
};

void createDescLayout(vk::Device& device, DescLayout& layout) {
  std::vector<vk::DescriptorSetLayoutBinding> binds;

  uint32_t i = 0;
  for (auto& bind : layout.binds) {
    binds.push_back({
        .binding = i++,
        .descriptorType = bind.type,
        .descriptorCount = bind.count,
        .stageFlags = layout.stages,
    });
  }

  vk::DescriptorSetLayoutCreateInfo layout_ci{};
  layout_ci.setBindings(binds);
  layout.layout = device.createDescriptorSetLayoutUnique(layout_ci).value;
}

void updateDescSet(
    const vk::Device& device, const vk::DescriptorSet& desc,
    const DescLayout& layout,
    std::vector<
        std::variant<vk::DescriptorImageInfo*, vk::DescriptorBufferInfo*>>&&
        updates) {
  assert(layout.binds.size() == updates.size());

  std::vector<vk::WriteDescriptorSet> writes;

  for (uint32_t i = 0; i < updates.size(); i++) {
    auto& update = updates[i];
    auto& bind = layout.binds[i];

    vk::WriteDescriptorSet write{
        .dstSet = desc,
        .dstBinding = i,
        .dstArrayElement = 0,
        .descriptorType = bind.type,
    };

    switch (bind.type) {
      case vk::DescriptorType::eCombinedImageSampler: {
        assert(std::holds_alternative<vk::DescriptorImageInfo*>(update));
        write.setImageInfo(*std::get<vk::DescriptorImageInfo*>(update));
        break;
      }
      case vk::DescriptorType::eUniformBuffer: {
        assert(std::holds_alternative<vk::DescriptorBufferInfo*>(update));
        write.setBufferInfo(*std::get<vk::DescriptorBufferInfo*>(update));
        break;
      }
      default:
        log("Unsupported descriptor type! {}", (int)bind.type);
        assert(false);
    }

    writes.push_back(write);
  }

  device.updateDescriptorSets(writes, nullptr);
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
