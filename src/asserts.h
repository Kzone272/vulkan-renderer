#pragma once

#include <iostream>
#include <stdexcept>

#include "defines.h"

namespace asserts {
char buf[2048];
}

// ASSERT(condition) checks if the condition is met, and if not, calls
// ABORT with an error message indicating the module and line where
// the error occurred.
#ifndef ASSERT
#define ASSERT(x)                                                              \
  if (!(x)) {                                                                  \
    snprintf(                                                                  \
        asserts::buf, 2048, "Assertion failed in \"%s\", line %d\n", __FILE__, \
        __LINE__);                                                             \
    throw std::logic_error(asserts::buf);                                      \
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
#define VKASSERT(x)                                                           \
  if ((x) != VK_SUCCESS) {                                                    \
    snprintf(                                                                 \
        asserts::buf, 2048, "Failed VkResult: (%d), in \"%s\", line %d\n", x, \
        __FILE__, __LINE__);                                                  \
    std::cerr << asserts::buf << std::endl;                                   \
    exit(1);                                                                  \
  } else  // This 'else' exists to catch the user's following semicolon
