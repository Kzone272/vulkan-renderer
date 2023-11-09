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

class ImageFactory {
 public:
  void init(
      const vk::Device& device,
      const vk::PhysicalDeviceMemoryProperties* mem_props) {
    device_ = device;
    mem_props_ = mem_props;
  }

  void createImage(
      Texture& texture, vk::ImageTiling tiling, vk::ImageUsageFlags usage,
      vk::MemoryPropertyFlags props, vk::ImageAspectFlags aspect,
      vk::Sampler sampler);
  vk::UniqueImageView createImageView(
      vk::Image img, vk::Format format, uint32_t mip_levels,
      vk::ImageAspectFlags aspect_flags);

 private:
  vk::Device device_;
  const vk::PhysicalDeviceMemoryProperties* mem_props_;
};

uint32_t findMemoryType(
    uint32_t type_filter, vk::MemoryPropertyFlags props,
    const vk::PhysicalDeviceMemoryProperties& device_mem_props);
