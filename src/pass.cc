#include "pass.h"

#include "descriptors.h"
#include "pipelines.h"
#include "scene-data.h"

void Pass::init(const VulkanState& vs) {
  fbo.init(vs);

  for (auto& lo : los) {
    lo->init(vs);
    lo->alloc(vs, vs.kMaxFramesInFlight);
  }

  for (auto& pl : pls) {
    initPipeline(vs.device, fbo, *pl);
  }
}

void Scene::init(const VulkanState& vs) {
  pass.fbo = {
      .size = vs.swap_size,
      .color_fmts =
          {vk::Format::eB8G8R8A8Unorm, vk::Format::eR32G32B32A32Sfloat},
      // Opaque Black, (Away Vector, FarZ)
      .clear_colors = {{0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, 1.f, 1.f}},
      .samples = vk::SampleCountFlagBits::e4,
      .depth_fmt = vs.depth_format,
      .make_output_set = true,
  };

  // Bound per frame.
  global = pass.makeDescLayout();
  *global = {
      .binds = {{.type = vk::DescriptorType::eUniformBuffer}},
      .stages =
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
  };
  // Bound per material
  // TODO: Don't allocate any here. These sets are allocated per material.
  material = pass.makeDescLayout();
  *material = {
      .binds =
          {{.type = vk::DescriptorType::eCombinedImageSampler},
           {.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  vk::PushConstantRange scene_push{
      .stageFlags = vk::ShaderStageFlagBits::eVertex,
      .offset = 0,
      .size = sizeof(PushData),
  };
  vk::PipelineVertexInputStateCreateInfo vertex_in{};
  auto vert_binding = getBindingDesc<Vertex>();
  auto vert_attrs = getAttrDescs<Vertex>();
  vertex_in.setVertexBindingDescriptions(vert_binding);
  vertex_in.setVertexAttributeDescriptions(vert_attrs);

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("scene.vert.spv"),
      .frag_shader = vs.shaders.get("scene.frag.spv"),
      .desc_layouts = {global, material},
      .push_ranges = {scene_push},
      .vert_in = vertex_in,
      .cull_mode = vk::CullModeFlagBits::eNone,
  };

  pass.init(vs);

  for (size_t i = 0; i < vs.kMaxFramesInFlight; i++) {
    globals.push_back(createDynamicBuffer(
        vs, sizeof(GlobalData), vk::BufferUsageFlagBits::eUniformBuffer));
  }

  std::vector<vk::WriteDescriptorSet> writes;
  global->updateUboBind(0, uboInfos(globals), writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Scene::update(const DrawState& ds, const FrameState& fs) {
  GlobalData data;
  data.view = fs.view;
  data.proj = fs.proj;
  data.inv_proj = glm::inverse(fs.proj);

  const size_t max_lights = std::size(data.lights);
  // Add the first max_lights lights to the frame UBO, and set the rest to
  // None.
  for (size_t i = 0; i < max_lights; i++) {
    if (i >= fs.lights.size()) {
      data.lights[i].type = Light::Type::None;
    } else {
      data.lights[i] = fs.lights[i];
    }
  }
  auto stages = vk::PipelineStageFlagBits::eVertexShader |
                vk::PipelineStageFlagBits::eFragmentShader;
  updateDynamicBuf(
      ds.cmd, globals[ds.frame], std::span<GlobalData>(&data, 1), stages,
      vk::AccessFlagBits::eUniformRead);
}

void Scene::render(
    const DrawState& ds, std::vector<SceneObject>& objects,
    const std::map<ModelId, std::unique_ptr<Model>>& loaded_models) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 0,
      global->sets[ds.frame], nullptr);

  // TODO: Sort by material, then by model.
  std::sort(objects.begin(), objects.end(), [](auto& left, auto& right) {
    return left.model < right.model;
  });

  ModelId curr_model_id = ModelId::None;
  Material* curr_material = nullptr;
  for (auto& obj : objects) {
    auto it = loaded_models.find(obj.model);
    ASSERT(it != loaded_models.end());
    auto* model = it->second.get();

    if (curr_material != model->material) {
      curr_material = model->material;
      ds.cmd.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, *draw->layout, 1,
          model->material->desc_set, nullptr);
    }

    if (curr_model_id != obj.model) {
      curr_model_id = obj.model;
      vk::DeviceSize offsets[] = {0};
      ds.cmd.bindVertexBuffers(0, *model->vert_buf, offsets);
      if (model->index_count) {
        ds.cmd.bindIndexBuffer(*model->ind_buf, 0, vk::IndexType::eUint32);
      }
    }

    PushData push_data{obj.transform};
    ds.cmd.pushConstants<PushData>(
        *draw->layout, vk::ShaderStageFlagBits::eVertex, 0, push_data);

    if (model->index_count) {
      ds.cmd.drawIndexed(model->index_count, 1, 0, 0, 0);
    } else {
      ds.cmd.draw(model->vertex_count, 1, 0, 0);
    }
  }

  ds.cmd.endRenderPass();
}

void Edges::init(
    const VulkanState& vs, DescLayout* image_set,
    const std::vector<vk::DescriptorBufferInfo*>& scene_globals) {
  pass.fbo = {
      .size = vs.swap_size,
      .color_fmts = {vk::Format::eB8G8R8A8Unorm},
      .make_output_set = true,
  };

  inputs = pass.makeDescLayout();
  *inputs = {
      .binds =
          {{.type = vk::DescriptorType::eUniformBuffer},
           {.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  draw = pass.makePipeline();
  *draw = {
      .vert_shader = vs.shaders.get("fullscreen.vert.spv"),
      .frag_shader = vs.shaders.get("edges.frag.spv"),
      .desc_layouts = {inputs, image_set},
      .vert_in = {},
  };

  pass.init(vs);

  for (size_t i = 0; i < vs.kMaxFramesInFlight; i++) {
    debugs.push_back(createDynamicBuffer(
        vs, sizeof(DebugData), vk::BufferUsageFlagBits::eUniformBuffer));
  }

  std::vector<vk::WriteDescriptorSet> writes;
  inputs->updateUboBind(0, scene_globals, writes);
  inputs->updateUboBind(1, uboInfos(debugs), writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Edges::update(const DrawState& ds, const DebugData& debug) {
  auto stages = vk::PipelineStageFlagBits::eVertexShader |
                vk::PipelineStageFlagBits::eFragmentShader;
  updateDynamicBuf(
      ds.cmd, debugs[ds.frame], std::span<const DebugData>(&debug, 1), stages,
      vk::AccessFlagBits::eUniformRead);
}

void Edges::render(const DrawState& ds, vk::DescriptorSet image_set) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 0,
      inputs->sets[ds.frame], nullptr);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 1, image_set, nullptr);
  ds.cmd.draw(3, 1, 0, 0);

  ds.cmd.endRenderPass();
}

void Swap::init(const VulkanState& vs) {
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

  pass.init(vs);
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

  for (size_t i = 0; i < vs.kMaxFramesInFlight; i++) {
    debugs.push_back(createDynamicBuffer(
        vs, sizeof(DebugData), vk::BufferUsageFlagBits::eUniformBuffer));
  }

  // TODO: Generalize so this can be part of pass.init()
  std::vector<vk::WriteDescriptorSet> writes;
  inputs->updateUboBind(0, uboInfos(debugs), writes);
  vs.device.updateDescriptorSets(writes, nullptr);
}

void Drawing::update(const DrawState& ds, const DebugData& debug) {
  updateDynamicBuf(
      ds.cmd, debugs[ds.frame], std::span<const DebugData>(&debug, 1),
      vk::PipelineStageFlagBits::eFragmentShader,
      vk::AccessFlagBits::eUniformRead);
}

void Drawing::render(const DrawState& ds) {
  pass.fbo.beginRp(ds);
  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw->layout, 0,
      inputs->sets[ds.frame], nullptr);

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
  for (int i = 0; i < vs.kMaxFramesInFlight; i++) {
    verts.push_back(
        createDynamicBuffer(vs, size, vk::BufferUsageFlagBits::eVertexBuffer));
  }

  pass.init(vs);
}

void Voronoi::update(const DrawState& ds, const std::vector<Vertex2d>& cells) {
  num_cells = cells.size();
  std::span<const Vertex2d> span = cells;
  updateDynamicBuf(
      ds.cmd, verts[ds.frame], span, vk::PipelineStageFlagBits::eVertexInput,
      vk::AccessFlagBits::eVertexAttributeRead);
}

void Voronoi::render(const DrawState& ds) {
  pass.fbo.beginRp(ds);

  ds.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw->pipeline);
  ds.cmd.bindVertexBuffers(0, *verts[ds.frame].device.buf, {0});
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
