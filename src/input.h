#pragma once

#include <SDL.h>

#include "asserts.h"

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
};

void resetRelativeInput(InputState& state) {
  state.mouse.moved = false;
  state.mouse.xrel = 0;
  state.mouse.yrel = 0;
  state.mouse.scrollyrel = 0;
}

void processInputState(const SDL_Event& event, InputState& state) {
  if (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP) {
    bool down = event.type == SDL_MOUSEBUTTONDOWN;

    int button = event.button.button;
    if (button == SDL_BUTTON_LEFT) {
      state.mouse.left = down;
    } else if (button == SDL_BUTTON_MIDDLE) {
      state.mouse.middle = down;
    } else if (button == SDL_BUTTON_RIGHT) {
      state.mouse.right = down;
    }
    // Ignore other buttons.
  } else if (event.type == SDL_MOUSEMOTION) {
    state.mouse.x = event.motion.x;
    state.mouse.y = event.motion.y;
    state.mouse.moved = true;
    state.mouse.xrel = event.motion.xrel;
    state.mouse.yrel = event.motion.yrel;
  } else if (event.type == SDL_MOUSEWHEEL) {
    int y = event.wheel.y;
    if (y > 0) {
      state.mouse.scrollyrel = 1;
    } else if (y < 0) {
      state.mouse.scrollyrel = -1;
    }
  }
}
