#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

using std::cerr;
using std::cout;
using std::endl;

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;

class HelloTriangleApp {
 public:
  void run() {
    initWindow();
    initVulkan();
    mainLoop();
    cleanup();
  }

 private:
  void initWindow() {
    ASSERT(SDL_Init(SDL_INIT_VIDEO) == 0);

    SDL_WindowFlags window_flags = SDL_WINDOW_VULKAN;
    window_ =
        SDL_CreateWindow("Vulkan Tutorial", SDL_WINDOWPOS_CENTERED,
                         SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, window_flags);
    if (!window_) {
      std::string error{SDL_GetError()};
      cerr << error << endl;
      ASSERT(false);
    }
  }

  void initVulkan() {}

  void mainLoop() {
    SDL_Event event;
    while (true) {
      SDL_PollEvent(&event);
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE) {
        break;
      }
    }
  }

  void cleanup() {
    SDL_DestroyWindow(window_);
    SDL_Quit();
  }

  SDL_Window* window_ = nullptr;
};
