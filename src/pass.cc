#include "pass.h"

#include "descriptors.h"
#include "draws.h"
#include "materials.h"
#include "pipelines.h"
#include "render-state.h"
#include "scene-data.h"

DescLayout* Pass::makeDescLayout() {
  los.push_back(std::make_unique<DescLayout>());
  return los.back().get();
}

Pipeline* Pass::makePipeline() {
  pls.push_back(std::make_unique<Pipeline>());
  return pls.back().get();
}

void Pass::init(const VulkanState& vs) {
  fbo.init(vs);

  for (auto& lo : los) {
    lo->init(vs);
    lo->alloc(vs, 1);
  }

  for (auto& pl : pls) {
    initPipeline(vs.device, fbo, *pl);
  }
}

void Scene::init(
    const VulkanState& vs, const DynamicBuf& globalBuf, Materials& mats,
    Draws* draws) {
  draws_ = draws;

  pass.fbo = {
      .size = vs.swap_size,
      .color_fmts =
          {vk::Format::eB8G8R8A8Srgb, vk::Format::eR32G32B32A32Sfloat},
      // Opaque Black, (Away Vector, FarZ)
      .clear_colors = {{0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, 1.f, 0.f}},
      .samples = vs.sceneSamples,
      .depth_fmt = vs.depth_format,
      .make_output_set = true,
      .output_sampler = vs.clamp_sampler,
  };

  // Bound per frame.
  global = pass.makeDescLayout();
  *global = {
      .binds =
          {
              {.type = vk::DescriptorType::eUniformBuffer},
              {.type = vk::DescriptorType::eStorageBuffer},
              {.type = vk::DescriptorType::eStorageBuffer},
              {.type = vk::DescriptorType::eStorageBuffer},
          },
      .stages =
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
  };

  vk::PipelineVertexInputStateCreateInfo vertex_in{};
  auto vert_binding = getBindingDesc<Vertex>();
  auto vert_attrs = getAttrDescs<Vertex>();
  vertex_in.setVertexBindingDescriptions(vert_binding);
  vertex_in.setVertexAttributeDescriptions(vert_attrs);

  auto* basic = pass.makePipeline();
  *basic = {
      .vert_shader = vs.shaders.get("scene-basic.vert.spv"),
      .frag_shader = vs.shaders.get("scene-basic.frag.spv"),
      .desc_layouts = {global, mats.layout()},
      .vert_in = vertex_in,
      .cull_mode = vk::CullModeFlagBits::eNone,
  };
  pipelines_.emplace(ScenePipeline::Basic, basic);

  auto* gradient = pass.makePipeline();
  *gradient = {
      .vert_shader = vs.shaders.get("scene-basic.vert.spv"),
      .frag_shader = vs.shaders.get("scene-gradient.frag.spv"),
      .desc_layouts = {global},
      .vert_in = vertex_in,
      .cull_mode = vk::CullModeFlagBits::eNone,
  };
  pipelines_.emplace(ScenePipeline::Gradient, gradient);

  pass.init(vs);

  std::vector<vk::WriteDescriptorSet> writes;
  global->updateUboBind(0, {globalBuf.info()}, writes);
  global->updateUboBind(1, {draws_->objectBuf_.info()}, writes);
  global->updateUboBind(2, {draws_->transformBuf_.info()}, writes);
  global->updateUboBind(3, {mats.bufferInfo()}, writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Scene::render(
    const DrawState& ds,
    const std::map<ModelId, std::unique_ptr<Model>>& loaded_models) {
  pass.fbo.beginRp(ds);

  Pipeline* currPipeline = nullptr;
  vk::DescriptorSet currDesc0 = {};
  vk::DescriptorSet currDesc1 = {};
  ModelId curr_model_id = ModelId::None;
  Model* curr_model = nullptr;

  for (auto& draw : draws_->instDraws_) {
    auto* drawPipeline = pipelines_.at(draw.matPipeline);
    if (currPipeline != drawPipeline) {
      currPipeline = drawPipeline;
      ds.cmd.bindPipeline(
          vk::PipelineBindPoint::eGraphics, *currPipeline->pipeline);
    }

    if (!currDesc0) {
      currDesc0 = global->sets[0];
      ds.cmd.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, *currPipeline->layout, 0,
          global->sets[0], nullptr);
    }

    if (draw.matPipeline == ScenePipeline::Basic && currDesc1 != draw.matDesc) {
      currDesc1 = draw.matDesc;
      ds.cmd.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, *currPipeline->layout, 1, currDesc1,
          nullptr);
    }

    if (curr_model_id != draw.model) {
      curr_model_id = draw.model;
      auto it = loaded_models.find(draw.model);
      ASSERT(it != loaded_models.end());
      curr_model = it->second.get();

      vk::DeviceSize offsets[] = {0};
      ds.cmd.bindVertexBuffers(0, *curr_model->vert_buf.buf, offsets);
      if (curr_model->index_count) {
        ds.cmd.bindIndexBuffer(
            *curr_model->ind_buf.buf, 0, vk::IndexType::eUint32);
      }
    }

    if (curr_model->index_count) {
      ds.cmd.drawIndexed(
          curr_model->index_count, draw.instances, 0, 0, draw.firstInstance);
    } else {
      ds.cmd.draw(
          curr_model->vertex_count, draw.instances, 0, draw.firstInstance);
    }
  }

  ds.cmd.endRenderPass();
}

void Edges::init(
    const VulkanState& vs, DescLayout* scene_output, bool use_msaa,
    DescLayout* sample_points, const DynamicBuf& globalBuf) {
  this->use_msaa = use_msaa;

  // Pre-pass setup
  pre_pass.fbo = {
      .size = vs.swap_size,
      .color_fmts = {vk::Format::eR8Uint},
      .make_output_set = true,
      .output_sampler = vs.nearest_sampler,
  };

  inputs = pre_pass.makeDescLayout();
  *inputs = {
      .binds =
          {{.type = vk::DescriptorType::eUniformBuffer},
           {.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  pre_draw = pre_pass.makePipeline();
  *pre_draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("edges-prepass.frag.spv"),
      .desc_layouts = {inputs, scene_output},
      .vert_in = {},
  };
  pre_pass.init(vs);

  pass.fbo = {
      .size = vs.swap_size,
      .color_fmts = {vk::Format::eR32G32Sfloat},
      .make_output_set = true,
      .output_sampler = vs.clamp_sampler,
  };

  fxaa_draw = pass.makePipeline();
  *fxaa_draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("edges-fxaa.frag.spv"),
      .desc_layouts = {inputs, &pre_pass.fbo.output_set},
      .vert_in = {},
  };

  if (use_msaa) {
    msaa_draw = pass.makePipeline();
    *msaa_draw = {
        .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
        .frag_shader = vs.shaders.get("edges.frag.spv"),
        .desc_layouts = {inputs, scene_output, sample_points},
        .vert_in = {},
    };
    sample_points_ = sample_points;
  }

  pass.init(vs);

  debug_buf = createDynamicBuffer(
      vs, sizeof(DebugData), vk::BufferUsageFlagBits::eUniformBuffer);

  std::vector<vk::WriteDescriptorSet> writes;
  inputs->updateUboBind(0, {globalBuf.info()}, writes);
  inputs->updateUboBind(1, {debug_buf.info()}, writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Edges::update(const DrawState& ds, const DebugData& debug) {
  auto stages = vk::PipelineStageFlagBits::eVertexShader |
                vk::PipelineStageFlagBits::eFragmentShader;
  updateDynamicBuf(
      ds, debug_buf, std::span<const DebugData>(&debug, 1), stages,
      vk::AccessFlagBits::eUniformRead);
}

void Edges::render(const DrawState& ds, vk::DescriptorSet norm_depth_set) {
  if (use_msaa) {
    renderMsaa(ds, norm_depth_set);
  } else {
    renderNormal(ds, norm_depth_set);
  }
}

void Edges::renderMsaa(const DrawState& ds, vk::DescriptorSet norm_depth_set) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *msaa_draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *msaa_draw->layout, 0,
      {inputs->sets[0], norm_depth_set, sample_points_->sets[0]}, nullptr);
  ds.cmd.draw(3, 1, 0, 0);

  ds.cmd.endRenderPass();
}

void Edges::renderNormal(
    const DrawState& ds, vk::DescriptorSet norm_depth_set) {
  {
    pre_pass.fbo.beginRp(ds);
    ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *pre_draw->pipeline);
    ds.cmd.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, *pre_draw->layout, 0,
        {inputs->sets[0], norm_depth_set}, nullptr);
    ds.cmd.draw(3, 1, 0, 0);
    ds.cmd.endRenderPass();
  }
  {
    pass.fbo.beginRp(ds);
    ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *fxaa_draw->pipeline);
    ds.cmd.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, *fxaa_draw->layout, 0,
        {inputs->sets[0], pre_pass.fbo.output_set.sets[0]}, nullptr);
    ds.cmd.draw(3, 1, 0, 0);
    ds.cmd.endRenderPass();
  }
}

void JumpFlood::init(const VulkanState& vs) {
  for (auto& pass : passes_) {
    pass.fbo = {
        .size = vs.swap_size,
        .color_fmts = {vk::Format::eR32G32Sfloat},
        .make_output_set = true,
        .output_sampler = vs.clamp_sampler,
    };

    if (!input_lo_) {
      input_lo_ = pass.makeDescLayout();
      *input_lo_ = {
          .binds = {{.type = vk::DescriptorType::eCombinedImageSampler}},
          .stages = vk::ShaderStageFlagBits::eFragment,
      };
    }
    if (!draw_) {
      vk::PushConstantRange step_push{
          .stageFlags = vk::ShaderStageFlagBits::eFragment,
          .offset = 0,
          .size = sizeof(uint32_t),
      };
      draw_ = pass.makePipeline();
      *draw_ = {
          .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
          .frag_shader = vs.shaders.get("jf-step.frag.spv"),
          .desc_layouts = {input_lo_},
          .push_ranges = {step_push},
      };
    }
    if (!voronoi_seed_) {
      vk::PushConstantRange tweak_push{
          .stageFlags = vk::ShaderStageFlagBits::eFragment,
          .offset = 0,
          .size = sizeof(float),
      };
      voronoi_seed_ = pass.makePipeline();
      *voronoi_seed_ = {
          .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
          .frag_shader = vs.shaders.get("voronoi-seed.frag.spv"),
          .desc_layouts = {input_lo_},
          .push_ranges = {tweak_push},
      };
    }

    pass.init(vs);
  }
}

void JumpFlood::resize(const VulkanState& vs) {
  for (auto& pass : passes_) {
    pass.fbo.resize(vs, vs.swap_size);
  }
}

void JumpFlood::render(
    const DrawState& ds, float spread, vk::DescriptorSet init_set) {
  int steps = std::floor(std::log2(std::ceil(spread))) + 1;
  uint32_t step_size = std::pow(2, steps - 1);
  for (int i = 0; i < steps; i++) {
    renderStep(ds, step_size, i == 0 ? init_set : lastOutputSet());
    step_size /= 2;
  }
}

void JumpFlood::renderStep(
    const DrawState& ds, uint32_t step_size, vk::DescriptorSet image_set) {
  passes_[next_].fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw_->pipeline);
  ds.cmd.pushConstants<uint32_t>(
      *draw_->layout, vk::ShaderStageFlagBits::eFragment, 0, step_size);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw_->layout, 0, {image_set},
      nullptr);
  ds.cmd.draw(3, 1, 0, 0);

  ds.cmd.endRenderPass();

  next();
}

void JumpFlood::initVoronoi(
    const DrawState& ds, float tweak, vk::DescriptorSet image_set) {
  passes_[next_].fbo.beginRp(ds);

  ds.cmd.bindPipeline(
      vk::PipelineBindPoint::eGraphics, *voronoi_seed_->pipeline);
  ds.cmd.pushConstants<float>(
      *voronoi_seed_->layout, vk::ShaderStageFlagBits::eFragment, 0, tweak);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *voronoi_seed_->layout, 0, {image_set},
      nullptr);
  ds.cmd.draw(3, 1, 0, 0);

  ds.cmd.endRenderPass();

  next();
}

void JumpFlood::next() {
  last_ = next_;
  next_ = (last_ + 1) % passes_.size();
}

void Swap::init(const VulkanState& vs, const DynamicBuf& globalBuf) {
  pass.fbo = {
      .size = vs.swap_size,
      .swap = true,
      .swap_format = vs.swap_format,
      .swap_views = vs.swap_views,
  };

  sampler = pass.makeDescLayout();
  *sampler = {
      .binds = {{.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("sample.frag.spv"),
      .desc_layouts = {sampler},
      .enable_blending = true,
  };

  normals_draw = pass.makePipeline();
  *normals_draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("normals.frag.spv"),
      .desc_layouts = {sampler},
  };

  vk::PushConstantRange depth_push{
      .stageFlags = vk::ShaderStageFlagBits::eFragment,
      .offset = 0,
      .size = sizeof(glm::mat4),
  };
  depth_draw = pass.makePipeline();
  *depth_draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("depth.frag.spv"),
      .desc_layouts = {sampler},
      .push_ranges = {depth_push},
  };

  vk::PushConstantRange jf_push{
      .stageFlags = vk::ShaderStageFlagBits::eFragment,
      .offset = 0,
      .size = sizeof(JfSdfPush),
  };
  jf_draw = pass.makePipeline();
  *jf_draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("jf-sdf.frag.spv"),
      .desc_layouts = {sampler},
      .push_ranges = {jf_push},
      .enable_blending = true,
  };

  uv_sample_draw = pass.makePipeline();
  *uv_sample_draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("uv-sample.frag.spv"),
      .desc_layouts = {sampler, sampler},
  };

  inputs2d_ = pass.makeDescLayout();
  *inputs2d_ = {
      .binds = {{.type = vk::DescriptorType::eUniformBuffer}},
      .stages =
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
  };

  vk::PushConstantRange draw2dPush{
      .stageFlags =
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
      .offset = 0,
      .size = sizeof(Draw2d),
  };

  pipeline2d_ = pass.makePipeline();
  *pipeline2d_ = {
      .vert_shader = vs.shaders.get("draw-2d.vert.spv"),
      .frag_shader = vs.shaders.get("draw-2d.frag.spv"),
      .desc_layouts = {inputs2d_},
      .push_ranges = {draw2dPush},
      .cull_mode = vk::CullModeFlagBits::eNone,
      .enable_blending = true,
  };
  pass.init(vs);

  std::vector<vk::WriteDescriptorSet> writes;
  inputs2d_->updateUboBind(0, {globalBuf.info()}, writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Swap::startRender(const DrawState& ds) {
  pass.fbo.beginRp(ds);
}

void Swap::drawImage(const DrawState& ds, vk::DescriptorSet image_set) {
  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 0, image_set, nullptr);
  ds.cmd.draw(3, 1, 0, 0);
}

void Swap::drawNormals(const DrawState& ds, vk::DescriptorSet image_set) {
  ds.cmd.bindPipeline(
      vk::PipelineBindPoint::eGraphics, *normals_draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *normals_draw->layout, 0, image_set,
      nullptr);
  ds.cmd.draw(3, 1, 0, 0);
}

void Swap::drawDepth(
    const DrawState& ds, vk::DescriptorSet image_set, const mat4& inv_proj) {
  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *depth_draw->pipeline);
  ds.cmd.pushConstants<mat4>(
      *depth_draw->layout, vk::ShaderStageFlagBits::eFragment, 0, inv_proj);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *depth_draw->layout, 0, image_set,
      nullptr);
  ds.cmd.draw(3, 1, 0, 0);
}

void Swap::drawJfSdf(
    const DrawState& ds, float width, uint32_t mode,
    vk::DescriptorSet image_set) {
  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *jf_draw->pipeline);
  JfSdfPush push = {width, mode};
  ds.cmd.pushConstants<JfSdfPush>(
      *jf_draw->layout, vk::ShaderStageFlagBits::eFragment, 0, push);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *jf_draw->layout, 0, image_set,
      nullptr);
  ds.cmd.draw(3, 1, 0, 0);
}

void Swap::draw2dDraws(
    const DrawState& ds, const std::vector<Draw2d>& draws2d) {
  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *pipeline2d_->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *pipeline2d_->layout, 0,
      inputs2d_->sets[0], nullptr);

  for (auto& draw : draws2d) {
    ds.cmd.pushConstants<Draw2d>(
        *pipeline2d_->layout,
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
        0, draw);
    ds.cmd.draw(6, 1, 0, 0);
  }
}

void Swap::drawUvSample(
    const DrawState& ds, vk::DescriptorSet uv_image, vk::DescriptorSet image) {
  ds.cmd.bindPipeline(
      vk::PipelineBindPoint::eGraphics, *uv_sample_draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *uv_sample_draw->layout, 0,
      {uv_image, image}, nullptr);
  ds.cmd.draw(3, 1, 0, 0);
}

void Drawing::init(const VulkanState& vs) {
  pass.fbo = {
      .size = {512, 512},
      .color_fmts = {vk::Format::eB8G8R8A8Unorm},
  };

  inputs = pass.makeDescLayout();
  *inputs = {
      .binds = {{.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("circle.frag.spv"),
      .desc_layouts = {inputs},
  };

  pass.init(vs);

  debug_buf = createDynamicBuffer(
      vs, sizeof(DebugData), vk::BufferUsageFlagBits::eUniformBuffer);

  // TODO: Generalize so this can be part of pass.init()
  std::vector<vk::WriteDescriptorSet> writes;
  inputs->updateUboBind(0, {debug_buf.info()}, writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Drawing::update(const DrawState& ds, const DebugData& debug) {
  updateDynamicBuf(
      ds, debug_buf, std::span<const DebugData>(&debug, 1),
      vk::PipelineStageFlagBits::eFragmentShader,
      vk::AccessFlagBits::eUniformRead);
}

void Drawing::render(const DrawState& ds) {
  pass.fbo.beginRp(ds);
  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 0, inputs->sets[0],
      nullptr);

  ds.cmd.draw(3, 1, 0, 0);
  ds.cmd.endRenderPass();
}

void Voronoi::init(const VulkanState& vs) {
  pass.fbo = {
      .size = {512, 512},
      .color_fmts = {vk::Format::eB8G8R8A8Unorm},
      .depth_fmt = vs.depth_format,
  };

  vk::PipelineVertexInputStateCreateInfo vert_in{};
  auto vert_bind = getBindingDesc<Vertex2d>();
  vert_bind.inputRate = vk::VertexInputRate::eInstance;
  vert_in.setVertexBindingDescriptions(vert_bind);
  auto vert_attrs = getAttrDescs<Vertex2d>();
  vert_in.setVertexAttributeDescriptions(vert_attrs);

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("voronoi.vert.spv"),
      .frag_shader = vs.shaders.get("voronoi.frag.spv"),
      .vert_in = vert_in,
  };

  // Support up to 100 voronoi cells for now.
  vk::DeviceSize size = 100 * sizeof(Vertex2d);
  verts = createDynamicBuffer(vs, size, vk::BufferUsageFlagBits::eVertexBuffer);

  pass.init(vs);
}

void Voronoi::update(const DrawState& ds, const std::vector<Vertex2d>& cells) {
  num_cells = cells.size();
  std::span<const Vertex2d> span = cells;
  updateDynamicBuf(
      ds, verts, span, vk::PipelineStageFlagBits::eVertexInput,
      vk::AccessFlagBits::eVertexAttributeRead);
}

void Voronoi::render(const DrawState& ds) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindVertexBuffers(0, *verts.device.buf, {0});
  ds.cmd.draw(6, num_cells, 0, 0);

  ds.cmd.endRenderPass();
}

void Resolve::init(const VulkanState& vs) {
  pass.fbo = {
      .size = vs.swap_size,
      .color_fmts = {vk::Format::eB8G8R8A8Unorm},
      .make_output_set = true,
  };

  sampler = pass.makeDescLayout();
  *sampler = {
      .binds = {{.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("resolve.frag.spv"),
      .desc_layouts = {sampler},
  };

  pass.init(vs);
}

void Resolve::render(const DrawState& ds, vk::DescriptorSet image_set) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 0, image_set, nullptr);
  ds.cmd.draw(3, 1, 0, 0);

  ds.cmd.endRenderPass();
}

void SampleQuery::init(const VulkanState& vs, vk::SampleCountFlagBits samples) {
  pass.fbo = {
      .size = {1, 1},
      .color_fmts = {vk::Format::eR32G32Sfloat},
      .samples = samples,
      .make_output_set = true,
  };

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("sample-query.frag.spv"),
      .sample_shading = true,
  };

  pass.init(vs);
}

void SampleQuery::render(const DrawState& ds) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.draw(3, 1, 0, 0);

  ds.cmd.endRenderPass();
}
