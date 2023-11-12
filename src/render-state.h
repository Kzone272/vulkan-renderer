#pragma once

// This will be owned by Renderer, but passed to other classes after all
// properties are set.
struct VulkanState {
  vk::Device device;
  vk::DescriptorPool desc_pool;
  vk::PhysicalDeviceProperties device_props;
  vk::PhysicalDeviceMemoryProperties mem_props;
  const uint32_t kMaxFramesInFlight;
};

struct DrawState {
  vk::CommandBuffer cmd = {};
  int frame = -1;
  uint32_t img_ind = 0;
};
