#include "buffers.h"

DynamicBuf::~DynamicBuf() {
  if (staging.alloc) {
    vma.unmapMemory(*staging.alloc);
  }
}

Buffer createBuffer(
    const VulkanState& vs, const vk::BufferCreateInfo& buffer_ci,
    const vma::AllocationCreateInfo& alloc_ci) {
  Buffer buffer;
  auto result = vs.vma.createBufferUnique(buffer_ci, alloc_ci).value;
  buffer.buf = std::move(result.first);
  buffer.alloc = std::move(result.second);
  buffer.info.range = buffer_ci.size;
  buffer.info.buffer = *buffer.buf;
  buffer.props = vs.vma.getAllocationMemoryProperties(*buffer.alloc);
  return std::move(buffer);
}

Buffer createStagingBuffer(const VulkanState& vs, vk::DeviceSize size) {
  vk::BufferCreateInfo buffer_ci{
      .size = size,
      .usage = vk::BufferUsageFlagBits::eTransferSrc,
  };
  vma::AllocationCreateInfo alloc_ci{
      .usage = vma::MemoryUsage::eAutoPreferHost,
      .flags = vma::AllocationCreateFlagBits::eMapped |
               vma::AllocationCreateFlagBits::eHostAccessSequentialWrite,
  };
  return createBuffer(vs, buffer_ci, alloc_ci);
}

Buffer createDeviceBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage) {
  vk::BufferCreateInfo buffer_ci{
      .size = size,
      .usage = usage,
  };
  vma::AllocationCreateInfo alloc_ci{
      .usage = vma::MemoryUsage::eAutoPreferDevice,
  };
  return createBuffer(vs, buffer_ci, alloc_ci);
}

DynamicBuf createDynamicBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage) {
  DynamicBuf dbuf;
  dbuf.vma = vs.vma;

  dbuf.staging = createStagingBuffer(vs, size);
  dbuf.staging.mapped = vs.vma.mapMemory(*dbuf.staging.alloc).value;

  dbuf.device = createDeviceBuffer(
      vs, size, usage | vk::BufferUsageFlagBits::eTransferDst);

  return std::move(dbuf);
}

void updateDynamicBuf(
    vk::CommandBuffer cmd, DynamicBuf& dbuf, void* data, size_t data_size,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  size_t size = std::min((size_t)dbuf.staging.info.range, data_size);
  memcpy(dbuf.staging.mapped, data, size);
  if (!(dbuf.staging.props & vk::MemoryPropertyFlagBits::eHostCoherent)) {
    dbuf.vma.flushAllocation(*dbuf.staging.alloc, 0, size);
  }

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

std::vector<vk::DescriptorBufferInfo*> uboInfos(
    std::vector<DynamicBuf>& dbufs) {
  std::vector<vk::DescriptorBufferInfo*> infos;
  for (auto& dbuf : dbufs) {
    infos.push_back(&dbuf.device.info);
  }
  return infos;
}

uint32_t findMemoryType(
    const VulkanState& vs, uint32_t type_filter,
    vk::MemoryPropertyFlags props) {
  for (uint32_t i = 0; i < vs.mem_props.memoryTypeCount; i++) {
    if (type_filter & (1 << i) &&
        (vs.mem_props.memoryTypes[i].propertyFlags & props) == props) {
      return i;
    }
  }
  ASSERT(false);
  return 0;
}

// Deprecated: Doesn't use VMA.
void createBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
    vk::UniqueDeviceMemory& buf_mem) {
  vk::BufferCreateInfo buffer_ci{
      .size = size,
      .usage = usage,
      .sharingMode = vk::SharingMode::eExclusive,
  };
  buf = vs.device.createBufferUnique(buffer_ci).value;

  vk::MemoryRequirements mem_reqs = vs.device.getBufferMemoryRequirements(*buf);

  vk::MemoryAllocateInfo alloc_info{
      .allocationSize = mem_reqs.size,
      .memoryTypeIndex = findMemoryType(vs, mem_reqs.memoryTypeBits, props),
  };
  buf_mem = vs.device.allocateMemoryUnique(alloc_info).value;
  std::ignore = vs.device.bindBufferMemory(*buf, *buf_mem, 0);
}