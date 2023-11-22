#pragma once

#include <memory>
#include <vector>

#include "buffers.h"
#include "fbo.h"
#include "render-objects.h"
#include "render-state.h"

struct Model;
struct DescLayout;
struct Pipeline;

struct Pass {
  Fbo fbo;
  std::vector<std::unique_ptr<DescLayout>> los;
  std::vector<std::unique_ptr<Pipeline>> pls;

  void init(const VulkanState& vs);

  DescLayout* makeDescLayout() {
    los.push_back(std::make_unique<DescLayout>());
    return los.back().get();
  }

  Pipeline* makePipeline() {
    pls.push_back(std::make_unique<Pipeline>());
    return pls.back().get();
  }
};

struct Scene {
  Pass pass;
  std::vector<DynamicBuf> globals;
  DescLayout* global;
  DescLayout* material;
  Pipeline* draw;

  void init(const VulkanState& vs);
  DescLayout* outputSet() {
    return &pass.fbo.output_set;
  }
  void update(const DrawState& ds, const FrameState& fs);
  void render(
      const DrawState& ds, std::vector<SceneObject>& objects,
      const std::map<ModelId, std::unique_ptr<Model>>& loaded_models);
};

struct Edges {
  Pass pass;
  std::vector<DynamicBuf> debugs;
  DescLayout* inputs;
  Pipeline* draw;

  void init(
      const VulkanState& vs, DescLayout* image_set,
      // TODO: This is gross. This should probably be a Ubo owned by Edges.
      const std::vector<vk::DescriptorBufferInfo*>& scene_globals);
  DescLayout* outputSet() {
    return &pass.fbo.output_set;
  }
  void update(const DrawState& ds, const DebugData& debug);
  void render(const DrawState& ds, vk::DescriptorSet image_set);
};

struct Swap {
  Pass pass;
  DescLayout* sampler;
  Pipeline* draw;
  Pipeline* normals_draw;
  Pipeline* depth_draw;

  void init(const VulkanState& vs);
  // This doesn't end the render pass so Renderer can draw whatever else it
  // wants before ending it.
  void startRender(const DrawState& ds);
  void drawImage(const DrawState& ds, vk::DescriptorSet image_set);
  void drawNormals(const DrawState& ds, vk::DescriptorSet image_set);
  void drawDepth(
      const DrawState& ds, vk::DescriptorSet image_set, const mat4& inv_proj);
};

struct Drawing {
  Pass pass;
  Pipeline* draw;
  std::vector<DynamicBuf> debugs;
  DescLayout* inputs;

  void init(const VulkanState& vs);
  void update(const DrawState& ds, const DebugData& debug);
  void render(const DrawState& ds);
};

struct Voronoi {
  Pass pass;
  Pipeline* draw;
  std::vector<DynamicBuf> verts;
  size_t num_cells = 0;

  void init(const VulkanState& vs);
  void update(const DrawState& ds, const std::vector<Vertex2d>& cells);
  void render(const DrawState& ds);
};

struct Resolve {
  Pass pass;
  DescLayout* sampler;
  Pipeline* draw;

  void init(const VulkanState& vs);
  DescLayout* outputSet() {
    return &pass.fbo.output_set;
  }
  void render(const DrawState& ds, vk::DescriptorSet image_set);
};
