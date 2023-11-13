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

uint32_t findMemoryType(
    const VulkanState& vs, uint32_t type_filter, vk::MemoryPropertyFlags props);

template <class T>
void updateDynamicBuf(
    vk::CommandBuffer cmd, DynamicBuf& dbuf, std::span<T> data,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  size_t size = std::min((size_t)dbuf.staging.info.range, data.size_bytes());
  memcpy(dbuf.staging.mapped, data.data(), size);

  vk::BufferCopy copy{
      .srcOffset = dbuf.staging.info.offset,
      .dstOffset = dbuf.device.info.offset,
      .size = size,
  };
  cmd.copyBuffer(*dbuf.staging.buf, *dbuf.device.buf, copy);

  vk::BufferMemoryBarrier barrier{
      .srcAccessMask = vk::AccessFlagBits::eTransferWrite,
      .dstAccessMask = dst_access,
      .buffer = *dbuf.device.buf,
      .offset = dbuf.device.info.offset,
      .size = size,
  };
  cmd.pipelineBarrier(
      vk::PipelineStageFlagBits::eTransfer, dst_stage, {}, nullptr, barrier,
      nullptr);
}
