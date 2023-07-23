#pragma once

#include <chrono>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Time = std::chrono::time_point<Clock>;
using FloatMs = std::chrono::duration<float, std::chrono::milliseconds::period>;
using FloatS = std::chrono::duration<float, std::chrono::seconds::period>;

Time addMs(Time t, float add_ms) {
  return t + std::chrono::duration_cast<Clock::duration>(FloatMs{add_ms});
}
