#pragma once

#include "asserts.h"

#ifdef DEBUG
#define VULKAN_HPP_ASSERT ASSERT
#endif  // DEBUG

#define VULKAN_HPP_NO_EXCEPTIONS
#define VULKAN_HPP_NO_STRUCT_CONSTRUCTORS  // allow designated initializers
#include <vulkan/vulkan.hpp>

// This is still needed for a few references.
#include <vulkan/vulkan.h>
