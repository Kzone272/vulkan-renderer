#pragma once

#include "vulkan-include.h"

struct PipelineInfo {
  vk::ShaderModule vert_shader;
  vk::ShaderModule frag_shader;
  std::vector<vk::DescriptorSetLayout> desc_layouts;
  std::vector<vk::PushConstantRange> push_ranges;
  vk::PipelineVertexInputStateCreateInfo vert_in = {};
  vk::CullModeFlags cull_mode = vk::CullModeFlagBits::eBack;
  bool enable_blending = false;
};

struct Pipeline {
  vk::UniquePipeline pipeline;
  vk::UniquePipelineLayout layout;
};

struct Fbo;
Pipeline createPipeline(
    vk::Device device, const Fbo& fbo, const PipelineInfo& pli);
