#pragma once

#include <SDL.h>

#include <set>

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

  struct Keyboard {
    std::set<int32_t> down;
    std::set<int32_t> pressed;  // Pressed this frame
  } kb;
};

namespace Keys {
const int32_t Shift = SDLK_LSHIFT;
}

std::set<int32_t> tracked_keys = {'w', 'a', 's', 'd', ' ', Keys::Shift};

void resetRelativeInput(InputState& state) {
  state.mouse.moved = false;
  state.mouse.xrel = 0;
  state.mouse.yrel = 0;
  state.mouse.scrollyrel = 0;
  state.kb.pressed.clear();
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
    // Accumulate because there may have been multiple move events to process
    // since the last update update.
    state.mouse.xrel += event.motion.xrel;
    state.mouse.yrel += event.motion.yrel;
  } else if (event.type == SDL_MOUSEWHEEL) {
    int y = event.wheel.y;
    if (y > 0) {
      state.mouse.scrollyrel = 1;
    } else if (y < 0) {
      state.mouse.scrollyrel = -1;
    }
  } else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
    int32_t key = event.key.keysym.sym;
    if (tracked_keys.contains(key)) {
      if (event.type == SDL_KEYDOWN) {
        if (!state.kb.down.contains(key)) {
          state.kb.pressed.insert(key);
        }
        state.kb.down.insert(key);
      } else {
        state.kb.down.erase(key);
      }
    }
  }
}

vec2 getWasdDir(const InputState& input) {
  vec2 dir{0};
  if (input.kb.down.contains('w')) {
    dir += vec2(0, 1);
  }
  if (input.kb.down.contains('s')) {
    dir += vec2(0, -1);
  }
  if (input.kb.down.contains('a')) {
    dir += vec2(-1, 0);
  }
  if (input.kb.down.contains('d')) {
    dir += vec2(1, 0);
  }

  if (dir != vec2(0)) {
    dir = glm::normalize(dir);
  }
  return dir;
}
