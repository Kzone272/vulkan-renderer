#pragma once

// TODO: I want to print a stacktrace in this function. I'll probably use
//       boost-stacktrace.
#define vulkanHppAssert(x)                                               \
  if (!(x)) {                                                            \
    printf("Assertion failed in \"%s\", line %d\n", __FILE__, __LINE__); \
    exit(1);                                                             \
  } else  // This 'else' exists to catch the user's following semicolon

#define VULKAN_HPP_ASSERT vulkanHppAssert
#define VULKAN_HPP_NO_EXCEPTIONS
#define VULKAN_HPP_NO_STRUCT_CONSTRUCTORS  // allow designated initializers
#include <vulkan/vulkan.hpp>
