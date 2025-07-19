#include "images.h"

#include <print>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>

#include "asserts.h"
#include "buffers.h"
#include "render-state.h"

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

SDL_Surface* loadImage(std::string_view texture_path) {
  SDL_Surface* texture_surface = IMG_Load(texture_path.data());
  ASSERT(texture_surface);
  ASSERT(texture_surface->pixels);
  // Vulkan likes images to have alpha channels. The SDL byte order is also
  // defined opposite to vk::Format.
  SDL_PixelFormatEnum desired_fmt = SDL_PIXELFORMAT_ARGB8888;
  if (texture_surface->format->format != desired_fmt) {
    std::println(
        "converting image pixel format from {} to {}",
        SDL_GetPixelFormatName(texture_surface->format->format),
        SDL_GetPixelFormatName(desired_fmt));
    auto* new_surface =
        SDL_ConvertSurfaceFormat(texture_surface, desired_fmt, 0);
    SDL_FreeSurface(texture_surface);
    texture_surface = new_surface;
  }

  return texture_surface;
}

bool hasStencilComponent(vk::Format format) {
  return format == vk::Format::eD32SfloatS8Uint ||
         format == vk::Format::eD24UnormS8Uint;
}

void transitionImageLayout(
    vk::CommandBuffer cmd_buf, vk::Image img, vk::Format format,
    uint32_t mip_levels, vk::ImageLayout old_layout,
    vk::ImageLayout new_layout) {
  vk::ImageMemoryBarrier barrier{
      .oldLayout = old_layout,
      .newLayout = new_layout,
      .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .image = img,
      .subresourceRange = {
          .baseMipLevel = 0,
          .levelCount = mip_levels,
          .baseArrayLayer = 0,
          .layerCount = 1,
      }};

  if (new_layout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
    barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
    if (hasStencilComponent(format)) {
      barrier.subresourceRange.aspectMask |= vk::ImageAspectFlagBits::eStencil;
    }
  } else {
    barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
  }

  vk::PipelineStageFlags src_stage;
  vk::PipelineStageFlags dst_stage;
  if (old_layout == vk::ImageLayout::eUndefined &&
      new_layout == vk::ImageLayout::eTransferDstOptimal) {
    barrier.srcAccessMask = {};
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;
    src_stage = vk::PipelineStageFlagBits::eTopOfPipe;
    dst_stage = vk::PipelineStageFlagBits::eTransfer;
  } else if (
      old_layout == vk::ImageLayout::eTransferDstOptimal &&
      new_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
    src_stage = vk::PipelineStageFlagBits::eTransfer;
    dst_stage = vk::PipelineStageFlagBits::eFragmentShader;
  } else if (
      old_layout == vk::ImageLayout::eUndefined &&
      new_layout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
    barrier.srcAccessMask = {};
    barrier.dstAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentRead |
                            vk::AccessFlagBits::eDepthStencilAttachmentWrite;
    src_stage = vk::PipelineStageFlagBits::eTopOfPipe;
    dst_stage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
  } else {
    std::println(
        "Unsupported layout transition! ({} -> {})", (int)old_layout,
        (int)new_layout);
    ASSERT(false);
  }

  cmd_buf.pipelineBarrier(src_stage, dst_stage, {}, nullptr, nullptr, barrier);
}

void copyBufferToImage(
    vk::CommandBuffer cmd_buf, vk::Buffer buf, vk::Image img, uint32_t width,
    uint32_t height) {
  vk::BufferImageCopy region{
      .bufferOffset = 0,
      .bufferRowLength = 0,
      .bufferImageHeight = 0,
      .imageSubresource =
          {
              .aspectMask = vk::ImageAspectFlagBits::eColor,
              .mipLevel = 0,
              .baseArrayLayer = 0,
              .layerCount = 1,
          },
      .imageOffset = {0, 0, 0},
      .imageExtent = {width, height, 1},
  };

  cmd_buf.copyBufferToImage(
      buf, img, vk::ImageLayout::eTransferDstOptimal, region);
}

void generateMipmaps(
    vk::CommandBuffer cmd_buf, vk::Image img, int32_t width, int32_t height,
    uint32_t mip_levels) {
  vk::ImageMemoryBarrier barrier{
      .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .image = img,
      .subresourceRange =
          {
              .aspectMask = vk::ImageAspectFlagBits::eColor,
              .levelCount = 1,
              .baseArrayLayer = 0,
              .layerCount = 1,
          },
  };

  int32_t mip_width = width;
  int32_t mip_height = height;
  for (uint32_t i = 0; i < mip_levels - 1; i++) {
    // Transition mip i to to be a copy source.
    barrier.subresourceRange.baseMipLevel = i;
    barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal;
    barrier.newLayout = vk::ImageLayout::eTransferSrcOptimal;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferRead;
    cmd_buf.pipelineBarrier(
        vk::PipelineStageFlagBits::eTransfer,
        vk::PipelineStageFlagBits::eTransfer, {}, nullptr, nullptr, barrier);

    int32_t src_width = mip_width;
    int32_t src_height = mip_height;
    if (src_width > 1) {
      mip_width /= 2;
    }
    if (src_height > 1) {
      mip_height /= 2;
    }

    // Blit from mip i to mip i+1 at half the size.
    vk::ImageBlit blit{
        .srcSubresource =
            {
                .aspectMask = vk::ImageAspectFlagBits::eColor,
                .mipLevel = i,
                .baseArrayLayer = 0,
                .layerCount = 1,
            },
        .srcOffsets = {{
            vk::Offset3D{0, 0, 0},
            vk::Offset3D{src_width, src_height, 1},
        }},
        .dstSubresource =
            {
                .aspectMask = vk::ImageAspectFlagBits::eColor,
                .mipLevel = i + 1,
                .baseArrayLayer = 0,
                .layerCount = 1,
            },
        .dstOffsets = {{
            vk::Offset3D{0, 0, 0},
            vk::Offset3D{mip_width, mip_height, 1},
        }}};
    cmd_buf.blitImage(
        img, vk::ImageLayout::eTransferSrcOptimal, img,
        vk::ImageLayout::eTransferDstOptimal, blit, vk::Filter::eLinear);

    // Transition mip i to be shader readable.
    barrier.oldLayout = vk::ImageLayout::eTransferSrcOptimal;
    barrier.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferRead;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
    cmd_buf.pipelineBarrier(
        vk::PipelineStageFlagBits::eTransfer,
        vk::PipelineStageFlagBits::eFragmentShader, {}, nullptr, nullptr,
        barrier);
  }
  // Transition last mip level to be shader readable.
  barrier.subresourceRange.baseMipLevel = mip_levels - 1;
  barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal;
  barrier.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
  barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
  barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
  cmd_buf.pipelineBarrier(
      vk::PipelineStageFlagBits::eTransfer,
      vk::PipelineStageFlagBits::eFragmentShader, {}, nullptr, nullptr,
      barrier);
}
