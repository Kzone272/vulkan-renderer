#pragma once

#include "vulkan-include.h"

struct Texture {
  vk::UniqueImage image;
  vk::UniqueDeviceMemory image_mem;
  vk::UniqueImageView image_view;
  vk::Format format;
  uint32_t mip_levels = 1;
};

struct Model {
  vk::UniqueBuffer vert_buf;
  vk::UniqueDeviceMemory vert_buf_mem;
  vk::UniqueBuffer ind_buf;
  vk::UniqueDeviceMemory ind_buf_mem;
  uint32_t index_count;
  std::unique_ptr<Texture> texture;
};
