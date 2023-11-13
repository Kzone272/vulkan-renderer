#pragma once

#include "shaders.h"

// This will be owned by Renderer, but passed to other classes after all
// properties are set.
struct VulkanState {
  const uint32_t kMaxFramesInFlight;
  vk::Device device;
  vk::DescriptorPool desc_pool;
  vk::Sampler linear_sampler;
  vk::PhysicalDeviceProperties device_props;
  vk::PhysicalDeviceMemoryProperties mem_props;
  Shaders shaders;
  vk::Format depth_format;
  vk::Extent2D swap_size;
  vk::Format swap_format;
  std::vector<vk::ImageView> swap_views;
};

struct DrawState {
  vk::CommandBuffer cmd = {};
  int frame = -1;
  uint32_t img_ind = 0;
};
