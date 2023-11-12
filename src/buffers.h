#pragma once

#include "render-state.h"
#include "vulkan-include.h"

struct Buffer {
  vk::UniqueBuffer buf;
  vk::UniqueDeviceMemory mem;
  void* mapped = nullptr;
  vk::DescriptorBufferInfo info;
};

struct DynamicBuf {
  Buffer staging;
  Buffer device;
};

// TODO: Take in a Buffer?
void createBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
    vk::UniqueDeviceMemory& buf_mem);

DynamicBuf createDynamicBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage);

uint32_t findMemoryType(
    const VulkanState& vs, uint32_t type_filter, vk::MemoryPropertyFlags props);
