#include "images.h"

#include "asserts.h"
#include "buffers.h"

void createImage(
    const VulkanState& vs, Texture& texture, vk::ImageTiling tiling,
    vk::ImageUsageFlags usage, vk::MemoryPropertyFlags props,
    vk::ImageAspectFlags aspect, vk::Sampler sampler) {
  vk::ImageCreateInfo img_ci{
      .imageType = vk::ImageType::e2D,
      .format = texture.format,
      .extent =
          {.width = texture.size.width,
           .height = texture.size.height,
           .depth = 1},
      .mipLevels = texture.mip_levels,
      .arrayLayers = 1,
      .samples = texture.samples,
      .tiling = tiling,
      .usage = usage,
      .sharingMode = vk::SharingMode::eExclusive,
      .initialLayout = vk::ImageLayout::eUndefined,
  };
  texture.image = vs.device.createImageUnique(img_ci).value;

  vk::MemoryRequirements mem_reqs =
      vs.device.getImageMemoryRequirements(*texture.image);

  vk::MemoryAllocateInfo alloc_info{
      .allocationSize = mem_reqs.size,
      .memoryTypeIndex = findMemoryType(vs, mem_reqs.memoryTypeBits, props),
  };
  texture.image_mem = vs.device.allocateMemoryUnique(alloc_info).value;
  std::ignore =
      vs.device.bindImageMemory(*texture.image, *texture.image_mem, 0);

  texture.image_view = createImageView(
      vs, *texture.image, texture.format, texture.mip_levels, aspect);
  texture.info = {
      .sampler = sampler,
      .imageView = *texture.image_view,
      .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
  };
}

vk::UniqueImageView createImageView(
    const VulkanState& vs, vk::Image img, vk::Format format,
    uint32_t mip_levels, vk::ImageAspectFlags aspect_flags) {
  vk::ImageViewCreateInfo ci{
      .image = img,
      .viewType = vk::ImageViewType::e2D,
      .format = format,
      .subresourceRange = {
          .aspectMask = aspect_flags,
          .baseMipLevel = 0,
          .levelCount = mip_levels,
          .baseArrayLayer = 0,
          .layerCount = 1,
      }};
  return vs.device.createImageViewUnique(ci).value;
}
