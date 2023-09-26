#pragma once

#include "files.h"
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

Pipeline createPipeline(vk::Device device, const PipelineInfo& pli) {
  vk::PipelineShaderStageCreateInfo vert_ci{
      .stage = vk::ShaderStageFlagBits::eVertex,
      .module = pli.vert_shader,
      .pName = "main",
  };
  vk::PipelineShaderStageCreateInfo frag_ci{
      .stage = vk::ShaderStageFlagBits::eFragment,
      .module = pli.frag_shader,
      .pName = "main",
  };
  std::vector<vk::PipelineShaderStageCreateInfo> shader_stages = {
      vert_ci, frag_ci};

  std::array<vk::DynamicState, 2> dyn_states = {
      vk::DynamicState::eViewport,
      vk::DynamicState::eScissor,
  };
  vk::PipelineDynamicStateCreateInfo dyn_state{};
  dyn_state.setDynamicStates(dyn_states);

  vk::PipelineInputAssemblyStateCreateInfo input_assembly{
      .topology = vk::PrimitiveTopology::eTriangleList,
      .primitiveRestartEnable = VK_FALSE,
  };

  vk::PipelineViewportStateCreateInfo viewport_state{
      .viewportCount = 1,
      .scissorCount = 1,
  };

  vk::PipelineRasterizationStateCreateInfo rasterizer{
      .depthClampEnable = VK_FALSE,
      .rasterizerDiscardEnable = VK_FALSE,
      .polygonMode = vk::PolygonMode::eFill,
      .cullMode = pli.cull_mode,
      .frontFace = vk::FrontFace::eClockwise,
      .depthBiasEnable = VK_FALSE,
      .lineWidth = 1.0f,
  };

  vk::PipelineMultisampleStateCreateInfo multisampling{
      .rasterizationSamples = pli.samples,
      .sampleShadingEnable = VK_FALSE,
  };

  vk::PipelineColorBlendAttachmentState color_blend_att{
      .blendEnable = VK_TRUE,
      .srcColorBlendFactor = vk::BlendFactor::eSrcAlpha,
      .dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha,
      .colorBlendOp = vk::BlendOp::eAdd,
      .srcAlphaBlendFactor = vk::BlendFactor::eOne,
      .dstAlphaBlendFactor = vk::BlendFactor::eZero,
      .alphaBlendOp = vk::BlendOp::eAdd,
      .colorWriteMask =
          vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
          vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA,
  };

  vk::PipelineColorBlendStateCreateInfo color_blending{
      .logicOpEnable = VK_FALSE};
  color_blending.setAttachments(color_blend_att);

  vk::PipelineDepthStencilStateCreateInfo depth_ci{
      .depthTestEnable = VK_TRUE,
      .depthWriteEnable = VK_TRUE,
      .depthCompareOp = vk::CompareOp::eLess,
      .stencilTestEnable = VK_FALSE,
  };

  Pipeline pl;

  vk::PipelineLayoutCreateInfo pipeline_layout_ci{};
  pipeline_layout_ci.setSetLayouts(pli.desc_layouts);
  pipeline_layout_ci.setPushConstantRanges(pli.push_ranges);
  pl.layout = device.createPipelineLayoutUnique(pipeline_layout_ci).value;

  vk::GraphicsPipelineCreateInfo pipeline_ci{
      .pVertexInputState = &pli.vert_in,
      .pInputAssemblyState = &input_assembly,
      .pViewportState = &viewport_state,
      .pRasterizationState = &rasterizer,
      .pMultisampleState = &multisampling,
      .pDepthStencilState = pli.depth_test ? &depth_ci : nullptr,
      .pColorBlendState = &color_blending,
      .pDynamicState = &dyn_state,
      .layout = *pl.layout,
      .renderPass = pli.render_pass,
      .subpass = 0,
  };
  pipeline_ci.setStages(shader_stages);
  pl.pipeline = device.createGraphicsPipelineUnique(nullptr, pipeline_ci).value;
  return pl;
}
