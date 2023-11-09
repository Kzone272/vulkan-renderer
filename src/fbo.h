#pragma once

#include "descriptors.h"
#include "images.h"
#include "vulkan-include.h"

struct Fbo {
  // Inputs
  vk::Extent2D size;
  std::vector<vk::Format> color_fmts;
  vk::SampleCountFlagBits samples = vk::SampleCountFlagBits::e1;
  bool resolve = false;
  std::optional<vk::Format> depth_fmt;
  vk::Sampler output_sampler;
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

  void initImages(ImageFactory& factory);
  void initDescs(int desc_count, vk::Device& device, vk::DescriptorPool& pool);
  void updateDescs(vk::Device& device);
  void initRp(vk::Device& device);
  void initFb(vk::Device& device);
  void resetImages();
};
