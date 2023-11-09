#include "images.h"

#include "asserts.h"

void ImageFactory::createImage(
    Texture& texture, vk::ImageTiling tiling, vk::ImageUsageFlags usage,
    vk::MemoryPropertyFlags props, vk::ImageAspectFlags aspect,
    vk::Sampler sampler) {
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
  texture.image = device_.createImageUnique(img_ci).value;

  vk::MemoryRequirements mem_reqs =
      device_.getImageMemoryRequirements(*texture.image);

  vk::MemoryAllocateInfo alloc_info{
      .allocationSize = mem_reqs.size,
      .memoryTypeIndex =
          findMemoryType(mem_reqs.memoryTypeBits, props, *mem_props_),
  };
  texture.image_mem = device_.allocateMemoryUnique(alloc_info).value;
  std::ignore = device_.bindImageMemory(*texture.image, *texture.image_mem, 0);

  texture.image_view = createImageView(
      *texture.image, texture.format, texture.mip_levels, aspect);
  texture.info = {
      .sampler = sampler,
      .imageView = *texture.image_view,
      .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
  };
}

vk::UniqueImageView ImageFactory::createImageView(
    vk::Image img, vk::Format format, uint32_t mip_levels,
    vk::ImageAspectFlags aspect_flags) {
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
  return device_.createImageViewUnique(ci).value;
}

uint32_t findMemoryType(
    uint32_t type_filter, vk::MemoryPropertyFlags props,
    const vk::PhysicalDeviceMemoryProperties& device_mem_props) {
  for (uint32_t i = 0; i < device_mem_props.memoryTypeCount; i++) {
    if (type_filter & (1 << i) &&
        (device_mem_props.memoryTypes[i].propertyFlags & props) == props) {
      return i;
    }
  }
  ASSERT(false);
  return 0;
}
