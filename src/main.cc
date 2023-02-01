#include <iostream>

#include <SDL2/SDL.h>
#undef main // this is needed in Windows

#include "utils.h"

int main() {
  utils::PrintMatrix();

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cout << "Failed to initialize the SDL2 library\n";
    return -1;
  }

  SDL_Window *window = SDL_CreateWindow("SDL2 Window",
                                        SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED,
                                        680, 480,
                                        SDL_WINDOW_RESIZABLE);
  if (!window) {
    std::cout << "Failed to create window\n";
    return -1;
  }

  SDL_Surface *window_surface = SDL_GetWindowSurface(window);
  if (!window_surface) {
    std::cout << "Failed to get the surface from the window\n";
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
