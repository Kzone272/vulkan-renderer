#include <iostream>

#include <SDL2/SDL.h>
#undef main // this is needed in Windows
#include <vulkan/vulkan.h>

#include "utils.h"

using std::cout;
using std::endl;

void vulkan_test() {
    uint32_t extensionCount = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

    cout << extensionCount << " extensions supported" << endl;
}

int main() {
  utils::PrintMatrix();
  vulkan_test();

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    cout << "Failed to initialize the SDL2 library" << endl;
    return -1;
  }

  SDL_Window *window = SDL_CreateWindow("SDL2 Window",
                                        SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED,
                                        680, 480,
                                        SDL_WINDOW_RESIZABLE);
  if (!window) {
    cout << "Failed to create window" << endl;
    return -1;
  }

  SDL_Surface *window_surface = SDL_GetWindowSurface(window);
  if (!window_surface) {
    cout << "Failed to get the surface from the window" << endl;
    return -1;
  }
  SDL_UpdateWindowSurface(window);

  SDL_Event event;
  while (true) {
    SDL_PollEvent(&event);
    if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE) {
      break;
    }
  }

  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
