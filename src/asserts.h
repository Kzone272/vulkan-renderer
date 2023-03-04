#pragma once

#include <iostream>

#include "defines.h"

// ASSERT(condition) checks if the condition is met, and if not, calls
// ABORT with an error message indicating the module and line where
// the error occurred.
#ifndef ASSERT
#define ASSERT(x)                                                          \
  if (!(x)) {                                                              \
    char buf[2048];                                                        \
    snprintf(buf, 2048, "Assertion failed in \"%s\", line %d\n", __FILE__, \
             __LINE__);                                                    \
    std::cerr << buf << std::endl;                                         \
    exit(1);                                                               \
  } else  // This 'else' exists to catch the user's following semicolon
#endif

// DASSERT(condition) is just like ASSERT, except that it only is
// functional in DEBUG mode, but does nothing when in a non-DEBUG
// (optimized, shipping) build.
#ifdef DEBUG
#define DASSERT(x) ASSERT(x)
#else
#define DASSERT(x) /* DASSERT does nothing when not debugging */
#endif

// ASSERT that a vulkan API call returns VK_SUCCESS.
#define VKASSERT(x)                                                       \
  if ((x) != VK_SUCCESS) {                                                \
    char buf[2048];                                                       \
    snprintf(buf, 2048, "Failed VkResult: (%d), in \"%s\", line %d\n", x, \
             __FILE__, __LINE__);                                         \
    std::cerr << buf << std::endl;                                        \
    exit(1);                                                              \
  } else  // This 'else' exists to catch the user's following semicolon
