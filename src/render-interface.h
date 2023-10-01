#pragma once

#include <memory>

#include "frame-state.h"
#include "render-objects.h"

class Renderer;
struct SDL_Window;

class RenderInterface {
 public:
  static std::unique_ptr<RenderInterface> CreateRenderer(
      SDL_Window* window, uint32_t width, uint32_t height);

  virtual void init(FrameState* frame_state) = 0;
  virtual void drawFrame(FrameState* frame_state) = 0;
  virtual void cleanup() = 0;
  virtual void resizeWindow(uint32_t width, uint32_t height) = 0;
  virtual void useModel(ModelId model_id, const ModelInfo& model_info) = 0;
  virtual MaterialId useMaterial(const MaterialInfo& mat_info) = 0;
  virtual void useMesh(
      ModelId model_id, const Mesh& mesh, MaterialId mat_id) = 0;
  // TODO: Return a TextureId instead.
  virtual Texture* getDrawingTexture() = 0;
  virtual void imguiNewFrame() = 0;
};
