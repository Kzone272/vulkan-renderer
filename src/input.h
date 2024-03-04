#pragma once

#include <SDL.h>

#include <set>

#include "glm-include.h"

// Persistent state of input devices, computed from events.
struct InputState {
  struct Mouse {
    bool left = false;
    bool middle = false;
    bool right = false;
    int x = 0;
    int y = 0;
    bool moved = false;
    int xrel = 0;
    int yrel = 0;
    int scrollyrel = 0;
  } mouse;

  struct Keyboard {
    std::set<int32_t> down;
    std::set<int32_t> pressed;  // Pressed this frame
  } kb;

  struct Movement {
    vec2 dir;  // in world space
  } move;
};

namespace Keys {
const int32_t Shift = SDLK_LSHIFT;
}  // namespace Keys

void resetRelativeInput(InputState& state);
void processInputState(const SDL_Event& event, InputState& state);
vec2 getWasdDir(const InputState& input);
