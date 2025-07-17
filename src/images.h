#pragma once

#include "render-objects.h"
#include "render-state.h"
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
  TextureId id = kTextureIdNone;
};

void createImage(
    const VulkanState& vs, Texture& texture, vk::ImageTiling tiling,
    vk::ImageUsageFlags usage, vk::MemoryPropertyFlags props,
    vk::ImageAspectFlags aspect, vk::Sampler sampler);
vk::UniqueImageView createImageView(
    const VulkanState& vs, vk::Image img, vk::Format format,
    uint32_t mip_levels, vk::ImageAspectFlags aspect_flags);
