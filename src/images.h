#pragma once

#include "vulkan-include.h"

struct Texture {
  // Inputs
  vk::Extent2D size;
  vk::Format format;
  uint32_t mip_levels = 1;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  // Outputs
  vk::UniqueImage image;
  vk::UniqueDeviceMemory image_mem;
  vk::UniqueImageView image_view;
  vk::DescriptorImageInfo info;
};

void createImage(
    const VulkanState& vs, Texture& texture, vk::ImageTiling tiling,
    vk::ImageUsageFlags usage, vk::MemoryPropertyFlags props,
    vk::ImageAspectFlags aspect, vk::Sampler sampler);
vk::UniqueImageView createImageView(
    const VulkanState& vs, vk::Image img, vk::Format format,
    uint32_t mip_levels, vk::ImageAspectFlags aspect_flags);

uint32_t findMemoryType(
    uint32_t type_filter, vk::MemoryPropertyFlags props,
    const vk::PhysicalDeviceMemoryProperties& device_mem_props);
