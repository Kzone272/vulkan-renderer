#pragma once

#include "materials.h"
#include "shaders.h"
#include "vma-usage.h"

// This will be owned by Renderer, but passed to other classes after all
// properties are set.
struct VulkanState {
  const uint32_t kMaxFramesInFlight;
  vk::PhysicalDevice physical_device;
  vk::Device device;
  vma::Allocator vma = nullptr;
  vk::DescriptorPool desc_pool;
  vk::Sampler linear_sampler;
  vk::Sampler clamp_sampler;
  vk::Sampler nearest_sampler;
  vk::PhysicalDeviceProperties device_props;
  vk::PhysicalDeviceMemoryProperties mem_props;
  vk::Format depth_format;
  vk::Extent2D swap_size;
  vk::Format swap_format;
  std::vector<vk::ImageView> swap_views;
  Shaders shaders;
  Materials materials;
};

struct DrawState {
  vk::CommandBuffer cmd = {};
  int frame = -1;  // always % kMaxFramesInFlight
  uint32_t img_ind = 0;
};
