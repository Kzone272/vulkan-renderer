#pragma once

#include <iostream>
#include <stdexcept>

#include "defines.h"

void ASSERTfn(bool x, std::string assertion);

// ASSERT(condition) checks if the condition is met, and if not, throws an
// exception with an error message indicating the failed assertion.
#ifndef ASSERT
#define ASSERT(x)    \
  if (!(x)) {        \
    ASSERTfn(x, #x); \
  } else  // This 'else' exists to catch the user's following semicolon
#endif

// DASSERT(condition) is just like ASSERT, except that it is only
// functional in DEBUG mode, but does nothing when in a non-DEBUG
// (optimized, shipping) build.
#ifdef DEBUG
#define DASSERT(x) ASSERT(x)
#else
#define DASSERT(x) /* DASSERT does nothing when not debugging */
#endif
