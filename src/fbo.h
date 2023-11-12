#pragma once

#include "descriptors.h"
#include "images.h"
#include "render-state.h"
#include "vulkan-include.h"

struct Fbo {
  // Inputs
  vk::Extent2D size;
  std::vector<vk::Format> color_fmts;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  bool resolve = false;
  std::optional<vk::Format> depth_fmt;
  vk::Sampler output_sampler;
  int desc_count = 0;
  bool swap = false;
  vk::Format swap_format;
  std::vector<vk::ImageView> swap_views;
  // Outputs
  std::vector<Texture> colors;
  std::vector<Texture> resolves;
  DescLayout output_set;
  Texture depth;
  vk::UniqueRenderPass rp;
  std::vector<vk::ClearValue> clears;
  std::vector<vk::UniqueFramebuffer> fbs;

  void init(const VulkanState& vs);
  void resize(const VulkanState& vs, vk::Extent2D new_size);
  void beginRp(const DrawState& ds) const;

  void initImages(const VulkanState& vs);
  void initDescs(const VulkanState& vs);
  void updateDescs(const VulkanState& vs);
  void initRp(const VulkanState& vs);
  void initFb(const VulkanState& vs);
  void resetImages();
};
