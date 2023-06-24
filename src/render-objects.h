#pragma once

#include "vulkan-include.h"

struct Texture {
  vk::UniqueImage image;
  vk::UniqueDeviceMemory image_mem;
  vk::UniqueImageView image_view;
  vk::Format format;
  uint32_t mip_levels = 1;
};
