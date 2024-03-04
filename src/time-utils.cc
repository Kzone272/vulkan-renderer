#include "time-utils.h"

Time addMs(Time t, float add_ms) {
  return t + std::chrono::duration_cast<Clock::duration>(FloatMs{add_ms});
}