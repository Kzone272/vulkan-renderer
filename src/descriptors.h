#pragma once

#include <variant>
#include <vector>

#include "render-state.h"
#include "vulkan-include.h"

struct Binding {
  vk::DescriptorType type;
  uint32_t index;
  uint32_t count = 1;
};

struct DescLayout {
  // Input
  std::vector<Binding> binds;
  vk::ShaderStageFlags stages;
  // Output
  vk::UniqueDescriptorSetLayout layout;
  std::vector<vk::DescriptorSet> sets;

  void init(const VulkanState& vs);
  void alloc(const VulkanState& vs, int count);
  void updateUboBind(
      uint32_t index, const std::vector<vk::DescriptorBufferInfo*>& infos,
      std::vector<vk::WriteDescriptorSet>& writes);
  void updateSamplerBind(
      uint32_t index, vk::DescriptorImageInfo* info,
      std::vector<vk::WriteDescriptorSet>& writes);
  void updateSamplersBind(
      uint32_t index, const std::vector<vk::DescriptorImageInfo>& infos,
      std::vector<vk::WriteDescriptorSet>& writes);
};

typedef std::variant<vk::DescriptorImageInfo*, vk::DescriptorBufferInfo*>
    DescSetUpdate;
typedef std::vector<DescSetUpdate> DescSetUpdates;

void updateDescSet(
    const vk::DescriptorSet& set, const DescLayout& layout,
    DescSetUpdates updates, std::vector<vk::WriteDescriptorSet>& writes);

std::vector<vk::DescriptorSet> allocDescSets(
    const VulkanState& vs, vk::DescriptorSetLayout layout, size_t count);

vk::DescriptorSet allocDescSet(
    const VulkanState& vs, vk::DescriptorSetLayout layout);
