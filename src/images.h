#pragma once

#include "render-objects.h"
#include "vulkan-include.h"

struct Texture {
  // Inputs
  vk::Extent2D size;
  vk::Format format;
  uint32_t mip_levels = 1;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  // Outputs
  vk::UniqueImage image;
  vk::UniqueDeviceMemory image_mem;
  vk::UniqueImageView image_view;
  vk::DescriptorImageInfo info;
  TextureId id = kTextureIdNone;
};

struct VulkanState;

void createImage(
    const VulkanState& vs, Texture& texture, vk::ImageTiling tiling,
    vk::ImageUsageFlags usage, vk::MemoryPropertyFlags props,
    vk::ImageAspectFlags aspect, vk::Sampler sampler);
vk::UniqueImageView createImageView(
    const VulkanState& vs, vk::Image img, vk::Format format,
    uint32_t mip_levels, vk::ImageAspectFlags aspect_flags);

struct SDL_Surface;
SDL_Surface* loadImage(std::string_view texture_path);

void transitionImageLayout(
    vk::CommandBuffer cmd_buf, vk::Image img, vk::Format format,
    uint32_t mip_levels, vk::ImageLayout old_layout,
    vk::ImageLayout new_layout);
void copyBufferToImage(
    vk::CommandBuffer cmd_buf, vk::Buffer buf, vk::Image img, uint32_t width,
    uint32_t height);
void generateMipmaps(
    vk::CommandBuffer cmd_buf, vk::Image img, int32_t width, int32_t height,
    uint32_t mip_levels);
