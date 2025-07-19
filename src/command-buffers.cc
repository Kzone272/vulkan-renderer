#include "command-buffers.h"

#include "render-state.h"

vk::CommandBuffer beginSingleTimeCommands(const VulkanState& vs) {
  vk::CommandBufferAllocateInfo alloc_info{
      .commandPool = vs.cmd_pool,
      .level = vk::CommandBufferLevel::ePrimary,
      .commandBufferCount = 1,
  };

  vk::CommandBuffer cmd_buf =
      vs.device.allocateCommandBuffers(alloc_info).value[0];

  vk::CommandBufferBeginInfo begin_info{
      .flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit};
  std::ignore = cmd_buf.begin(begin_info);

  return cmd_buf;
}

void endSingleTimeCommands(const VulkanState& vs, vk::CommandBuffer cmd_buf) {
  std::ignore = cmd_buf.end();

  vk::SubmitInfo submit{};
  submit.setCommandBuffers(cmd_buf);
  std::ignore = vs.gfx_q.submit(submit);
  std::ignore = vs.gfx_q.waitIdle();

  vs.device.freeCommandBuffers(vs.cmd_pool, cmd_buf);
}
