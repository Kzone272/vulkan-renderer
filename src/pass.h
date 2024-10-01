#pragma once

#include <memory>
#include <vector>

#include "buffers.h"
#include "fbo.h"
#include "render-objects.h"
#include "render-state.h"
#include "scene-data.h"

struct Model;
struct DescLayout;
struct Pipeline;

struct Pass {
  Fbo fbo;
  std::vector<std::unique_ptr<DescLayout>> los;
  std::vector<std::unique_ptr<Pipeline>> pls;

  void init(const VulkanState& vs);

  DescLayout* makeDescLayout();
  Pipeline* makePipeline();
};

struct Scene {
  Pass pass;
  std::vector<DynamicBuf> globals;
  DescLayout* global;
  DescLayout* material;
  Pipeline* draw;

  void init(const VulkanState& vs, vk::SampleCountFlagBits samples);
  DescLayout* outputSet() {
    return &pass.fbo.output_set;
  }
  void resize(const VulkanState& vs) {
    pass.fbo.resize(vs, vs.swap_size);
  }
  void update(const DrawState& ds, const FrameState& fs);
  void render(
      const DrawState& ds, std::vector<SceneObject>& objects,
      const std::map<ModelId, std::unique_ptr<Model>>& loaded_models,
      const std::vector<std::unique_ptr<Material>>& loaded_mats);
};

struct Edges {
  Pass pass;
  std::vector<DynamicBuf> debugs;
  DescLayout* inputs;
  DescLayout* sample_points_;
  Pipeline* fxaa_draw;
  Pipeline* msaa_draw;

  Pass pre_pass;
  DescLayout* pre_inputs;
  Pipeline* pre_draw;
  bool use_msaa = false;

  void init(
      const VulkanState& vs, DescLayout* scene_output,
      vk::SampleCountFlagBits scene_samples, DescLayout* sample_points,
      // TODO: This is gross. This should probably be a Ubo owned by Edges.
      const std::vector<vk::DescriptorBufferInfo*>& scene_globals);
  DescLayout* outputSet() {
    return &pass.fbo.output_set;
  }
  void resize(const VulkanState& vs) {
    pass.fbo.resize(vs, vs.swap_size);
    pre_pass.fbo.resize(vs, vs.swap_size);
  }
  void update(const DrawState& ds, const DebugData& debug);
  void render(const DrawState& ds, vk::DescriptorSet norm_depth_set);
  void renderMsaa(const DrawState& ds, vk::DescriptorSet norm_depth_set);
  void renderNormal(const DrawState& ds, vk::DescriptorSet norm_depth_set);
};

class JumpFlood {
 public:
  void init(const VulkanState& vs);
  vk::DescriptorSet lastOutputSet() {
    return passes_[last_].fbo.output_set.sets[0];
  }
  void resize(const VulkanState& vs);
  void render(const DrawState& ds, float spread, vk::DescriptorSet init_set);
  void renderStep(
      const DrawState& ds, uint32_t step_size, vk::DescriptorSet image_set);
  void initVoronoi(
      const DrawState& ds, float tweak, vk::DescriptorSet image_set);

 private:
  void next();

  int last_ = -1;
  int next_ = 0;
  std::array<Pass, 2> passes_;
  DescLayout* input_lo_ = nullptr;
  Pipeline* draw_ = nullptr;
  Pipeline* voronoi_seed_ = nullptr;
};

struct Swap {
  Pass pass;
  DescLayout* sampler;
  Pipeline* draw;
  Pipeline* normals_draw;
  Pipeline* depth_draw;
  Pipeline* jf_draw;
  Pipeline* uv_sample_draw;

  struct JfSdfPush {
    float width = 1;
    uint32_t mode = 0;
  };

  void init(const VulkanState& vs);
  void resize(const VulkanState& vs) {
    pass.fbo.swap_format = vs.swap_format;
    pass.fbo.swap_views = vs.swap_views;
    pass.fbo.resize(vs, vs.swap_size);
  }
  // This doesn't end the render pass so Renderer can draw whatever else it
  // wants before ending it.
  void startRender(const DrawState& ds);
  void drawImage(const DrawState& ds, vk::DescriptorSet image_set);
  void drawNormals(const DrawState& ds, vk::DescriptorSet image_set);
  void drawDepth(
      const DrawState& ds, vk::DescriptorSet image_set, const mat4& inv_proj);
  // Jump Flood Signed Distance Field
  void drawJfSdf(
      const DrawState& ds, float width, uint32_t mode,
      vk::DescriptorSet image_set);
  void drawUvSample(
      const DrawState& ds, vk::DescriptorSet uv_image, vk::DescriptorSet image);
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
  void resize(const VulkanState& vs) {
    pass.fbo.resize(vs, vs.swap_size);
  }
  void render(const DrawState& ds, vk::DescriptorSet image_set);
};

// Gets the positions of MSAA subsamples.
struct SampleQuery {
  Pass pass;
  Pipeline* draw;

  void init(const VulkanState& vs, vk::SampleCountFlagBits samples);
  DescLayout* outputSet() {
    return &pass.fbo.output_set;
  }
  void render(const DrawState& ds);
};
