#include "buffers.h"

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

DynamicBuf createDynamicBuffer(
    const VulkanState& vs, vk::DeviceSize size, vk::BufferUsageFlags usage) {
  DynamicBuf dbuf;
  createBuffer(
      vs, size, vk::BufferUsageFlagBits::eTransferSrc,
      vk::MemoryPropertyFlagBits::eHostVisible |
          vk::MemoryPropertyFlagBits::eHostCoherent,
      dbuf.staging.buf, dbuf.staging.mem);
  dbuf.staging.info.range = size;
  dbuf.staging.info.buffer = *dbuf.staging.buf;

  dbuf.staging.mapped =
      vs.device.mapMemory(*dbuf.staging.mem, dbuf.staging.info.offset, size)
          .value;

  createBuffer(
      vs, size, usage | vk::BufferUsageFlagBits::eTransferDst,
      vk::MemoryPropertyFlagBits::eDeviceLocal, dbuf.device.buf,
      dbuf.device.mem);
  dbuf.device.info.range = size;
  dbuf.device.info.buffer = *dbuf.device.buf;

  return std::move(dbuf);
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
