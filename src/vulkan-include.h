#pragma once

#include "asserts.h"

#define VULKAN_HPP_ASSERT ASSERT
#define VULKAN_HPP_NO_EXCEPTIONS
#define VULKAN_HPP_NO_STRUCT_CONSTRUCTORS  // allow designated initializers
#include <vulkan/vulkan.hpp>

// This is still needed for a few references.
#include <vulkan/vulkan.h>

// This will be owned by Renderer, but passed to other classes after all
// properties are set.
struct VulkanState {
  vk::Device device;
  vk::DescriptorPool desc_pool;
  vk::PhysicalDeviceProperties device_props;
  vk::PhysicalDeviceMemoryProperties mem_props;
} vs_;
