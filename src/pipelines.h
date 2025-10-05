#pragma once

#include "vulkan-include.h"

struct DescLayout;
struct Fbo;

struct Pipeline {
  // Inputs
  vk::ShaderModule vert_shader;
  vk::ShaderModule frag_shader;
  std::vector<DescLayout*> desc_layouts;
  std::vector<vk::PushConstantRange> push_ranges;
  vk::PipelineVertexInputStateCreateInfo vert_in = {};
  vk::CullModeFlags cull_mode = vk::CullModeFlagBits::eBack;
  bool enable_blending = false;
  bool sample_shading = false;
  bool depthTest = true;
  bool depthWrite = true;
  // Outputs
  vk::UniquePipeline pipeline;
  vk::UniquePipelineLayout layout;
};

void initPipeline(vk::Device device, const Fbo& fbo, Pipeline& pl);
