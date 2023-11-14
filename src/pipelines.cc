#pragma once

#include "pipelines.h"

#include "fbo.h"

void initPipeline(vk::Device device, const Fbo& fbo, Pipeline& pl) {
  vk::PipelineShaderStageCreateInfo vert_ci{
      .stage = vk::ShaderStageFlagBits::eVertex,
      .module = pl.vert_shader,
      .pName = "main",
  };
  vk::PipelineShaderStageCreateInfo frag_ci{
      .stage = vk::ShaderStageFlagBits::eFragment,
      .module = pl.frag_shader,
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
      .cullMode = pl.cull_mode,
      .frontFace = vk::FrontFace::eClockwise,
      .depthBiasEnable = VK_FALSE,
      .lineWidth = 1.0f,
  };

  vk::PipelineMultisampleStateCreateInfo multisampling{
      .rasterizationSamples = fbo.samples,
      .sampleShadingEnable = VK_FALSE,
  };

  vk::PipelineColorBlendAttachmentState color_blend_att{
      .blendEnable = VK_TRUE,
      .srcColorBlendFactor = vk::BlendFactor::eSrcAlpha,
      .dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha,
      .colorBlendOp = vk::BlendOp::eAdd,
      .srcAlphaBlendFactor = vk::BlendFactor::eSrcAlpha,
      .dstAlphaBlendFactor = vk::BlendFactor::eDstAlpha,
      .alphaBlendOp = vk::BlendOp::eAdd,
      .colorWriteMask =
          vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
          vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA,
  };
  vk::PipelineColorBlendAttachmentState no_blend_att{
      .blendEnable = VK_FALSE,
      .colorWriteMask =
          vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
          vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA,
  };

  vk::PipelineColorBlendStateCreateInfo color_blending{
      .logicOpEnable = VK_FALSE};
  std::vector<vk::PipelineColorBlendAttachmentState> blend_atts(
      fbo.swap ? 1 : fbo.color_fmts.size(),
      pl.enable_blending ? color_blend_att : no_blend_att);
  color_blending.setAttachments(blend_atts);

  vk::PipelineDepthStencilStateCreateInfo depth_ci{
      .depthTestEnable = VK_TRUE,
      .depthWriteEnable = VK_TRUE,
      .depthCompareOp = vk::CompareOp::eLess,
      .stencilTestEnable = VK_FALSE,
  };

  vk::PipelineLayoutCreateInfo pipeline_layout_ci{};
  std::vector<vk::DescriptorSetLayout> set_layouts;
  for (auto* desc_lo : pl.desc_layouts) {
    set_layouts.push_back(*desc_lo->layout);
  }
  pipeline_layout_ci.setSetLayouts(set_layouts);
  pipeline_layout_ci.setPushConstantRanges(pl.push_ranges);
  pl.layout =
      std::move(device.createPipelineLayoutUnique(pipeline_layout_ci).value);

  vk::GraphicsPipelineCreateInfo pipeline_ci{
      .pVertexInputState = &pl.vert_in,
      .pInputAssemblyState = &input_assembly,
      .pViewportState = &viewport_state,
      .pRasterizationState = &rasterizer,
      .pMultisampleState = &multisampling,
      .pDepthStencilState = fbo.depth_fmt ? &depth_ci : nullptr,
      .pColorBlendState = &color_blending,
      .pDynamicState = &dyn_state,
      .layout = *pl.layout,
      .renderPass = *fbo.rp,
      .subpass = 0,
  };
  pipeline_ci.setStages(shader_stages);
  pl.pipeline = std::move(
      device.createGraphicsPipelineUnique(nullptr, pipeline_ci).value);
}
