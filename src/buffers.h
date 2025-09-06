#pragma once

#include "vma-usage.h"
#include "vulkan-include.h"

struct Buffer {
  vma::UniqueBuffer buf = {};
  vma::UniqueAllocation alloc = {};
  vk::MemoryPropertyFlags props = {};
  void* mapped = nullptr;
  vk::DescriptorBufferInfo info = {};
};

struct DynamicBuf {
  DynamicBuf() = default;
  ~DynamicBuf();
  // Move only
  DynamicBuf(DynamicBuf&& other) = default;
  DynamicBuf& operator=(DynamicBuf&& other) = default;
  DynamicBuf(const DynamicBuf& other) = delete;
  DynamicBuf& operator=(const DynamicBuf& other) = delete;

  const vk::DescriptorBufferInfo* info() const {
    return &device.info;
  }

  std::vector<Buffer> staging;
  Buffer device;
  vma::Allocator vma;
};

struct VulkanState;
struct DrawState;

Buffer createBuffer(
    const VulkanState& vs, const vk::BufferCreateInfo& buffer_ci,
    const vma::AllocationCreateInfo& alloc_ci);
Buffer createStagingBuffer(const VulkanState& vs, vk::DeviceSize size);
Buffer createDeviceBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage);

void copyBufferToBuffer(
    const vk::CommandBuffer& cmd, const Buffer& src, const Buffer& dst,
    size_t size, vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access);

DynamicBuf createDynamicBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage);
std::vector<vk::DescriptorBufferInfo*> uboInfos(std::vector<DynamicBuf>& dbufs);
void updateDynamicBuf(
    const DrawState& ds, DynamicBuf& dbuf, void* data, size_t data_size,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access);

template <class T>
void updateDynamicBuf(
    const DrawState& ds, DynamicBuf& dbuf, std::span<T> data,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  updateDynamicBuf(
      ds, dbuf, (void*)data.data(), data.size_bytes(), dst_stage, dst_access);
}

uint32_t findMemoryType(
    const VulkanState& vs, uint32_t type_filter, vk::MemoryPropertyFlags props);

// Deprecated: Doesn't use VMA.
void createBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
    vk::UniqueDeviceMemory& buf_mem);
