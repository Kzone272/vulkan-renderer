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
std::vector<vk::DescriptorBufferInfo*> uboInfos(std::vector<DynamicBuf>& dbufs);
void updateDynamicBuf(
    vk::CommandBuffer cmd, DynamicBuf& dbuf, void* data, size_t data_size,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access);

template <class T>
void updateDynamicBuf(
    vk::CommandBuffer cmd, DynamicBuf& dbuf, std::span<T> data,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  updateDynamicBuf(
      cmd, dbuf, (void*)data.data(), data.size_bytes(), dst_stage, dst_access);
}

uint32_t findMemoryType(
    const VulkanState& vs, uint32_t type_filter, vk::MemoryPropertyFlags props);
