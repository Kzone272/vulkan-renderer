#include "render-interface.h"

#include "renderer.h"

std::unique_ptr<RenderInterface> RenderInterface::CreateRenderer(
    SDL_Window* window, uint32_t width, uint32_t height) {
  return std::make_unique<Renderer>(window, width, height);
}
