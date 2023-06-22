#pragma once

#include <iostream>
#include <stdexcept>

#include "defines.h"

// ASSERT(condition) checks if the condition is met, and if not, calls
// ABORT with an error message indicating the module and line where
// the error occurred.
void ASSERT(bool x);

// DASSERT(condition) is just like ASSERT, except that it is only
// functional in DEBUG mode, but does nothing when in a non-DEBUG
// (optimized, shipping) build.
#ifdef DEBUG
#define DASSERT(x) ASSERT(x)
#else
#define DASSERT(x) /* DASSERT does nothing when not debugging */
#endif
