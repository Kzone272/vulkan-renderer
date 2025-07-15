#include "buffers.h"

DynamicBuf::~DynamicBuf() {
  for (auto& staging_buf : staging) {
    if (staging_buf.alloc) {
      vma.unmapMemory(*staging_buf.alloc);
    }
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
      .flags = vma::AllocationCreateFlagBits::eMapped |
               vma::AllocationCreateFlagBits::eHostAccessSequentialWrite,
      .usage = vma::MemoryUsage::eAutoPreferHost,
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

  for (uint32_t i = 0; i < vs.kMaxFramesInFlight; i++) {
    auto staging_buf = createStagingBuffer(vs, size);
    staging_buf.mapped = vs.vma.mapMemory(*staging_buf.alloc).value;
    dbuf.staging.push_back(std::move(staging_buf));
  }

  dbuf.device = createDeviceBuffer(
      vs, size, usage | vk::BufferUsageFlagBits::eTransferDst);

  return std::move(dbuf);
}

void copyBufferToBuffer(
    const vk::CommandBuffer& cmd, const Buffer& src, const Buffer& dst,
    size_t size, vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  vk::BufferCopy copy{
      .srcOffset = src.info.offset,
      .dstOffset = dst.info.offset,
      .size = size,
  };
  cmd.copyBuffer(*src.buf, *dst.buf, copy);

  vk::BufferMemoryBarrier barrier{
      .srcAccessMask = vk::AccessFlagBits::eTransferWrite,
      .dstAccessMask = dst_access,
      .buffer = *dst.buf,
      .offset = dst.info.offset,
      .size = size,
  };
  cmd.pipelineBarrier(
      vk::PipelineStageFlagBits::eTransfer, dst_stage, {}, nullptr, barrier,
      nullptr);
}

void updateDynamicBuf(
    const DrawState& ds, DynamicBuf& dbuf, void* data, size_t data_size,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  auto& staging = dbuf.staging[ds.frame];

  size_t size = std::min((size_t)staging.info.range, data_size);
  memcpy(staging.mapped, data, size);
  if (!(staging.props & vk::MemoryPropertyFlagBits::eHostCoherent)) {
    dbuf.vma.flushAllocation(*staging.alloc, 0, size);
  }

  copyBufferToBuffer(ds.cmd, staging, dbuf.device, size, dst_stage, dst_access);
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
