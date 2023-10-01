#pragma once

#include <variant>
#include <vector>

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

void createDescLayout(vk::Device& device, DescLayout& layout);

typedef std::vector<
    std::variant<vk::DescriptorImageInfo*, vk::DescriptorBufferInfo*>>
    DescSetUpdates;

void updateDescSet(
    const vk::Device& device, const vk::DescriptorSet& desc,
    const DescLayout& layout, DescSetUpdates updates);

std::vector<vk::DescriptorSet> allocDescSets(
    vk::Device device, vk::DescriptorPool pool, vk::DescriptorSetLayout layout,
    size_t count);

vk::DescriptorSet allocDescSet(
    vk::Device device, vk::DescriptorPool pool, vk::DescriptorSetLayout layout);
