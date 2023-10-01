#pragma once

#include "vulkan-include.h"

struct PipelineInfo {
  vk::ShaderModule vert_shader;
  vk::ShaderModule frag_shader;
  vk::RenderPass render_pass;
  std::vector<vk::DescriptorSetLayout> desc_layouts;
  std::vector<vk::PushConstantRange> push_ranges;
  vk::PipelineVertexInputStateCreateInfo vert_in = {};
  vk::CullModeFlags cull_mode = vk::CullModeFlagBits::eBack;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  bool depth_test = true;
};

struct Pipeline {
  vk::UniquePipeline pipeline;
  vk::UniquePipelineLayout layout;
};

Pipeline createPipeline(vk::Device device, const PipelineInfo& pli);
