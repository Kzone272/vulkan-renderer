#include "fbo.h"

#include "renderer.h"

void Fbo::init(const VulkanState& vs) {
  initImages(vs);
  initDescs(vs);
  initRp(vs);
  initFb(vs);
}

void Fbo::resize(const VulkanState& vs, vk::Extent2D new_size) {
  resetImages();

  size = new_size;
  initImages(vs);
  updateDescs(vs);
  initFb(vs);
}

void Fbo::beginRp(const DrawState& ds) const {
  vk::Viewport viewport{
      .x = 0.0f,
      .y = 0.0f,
      .width = static_cast<float>(size.width),
      .height = static_cast<float>(size.height),
      .minDepth = 0.0f,
      .maxDepth = 1.0f,
  };
  vk::Rect2D scissor{
      .offset = {0, 0},
      .extent = size,
  };
  ds.cmd.setViewport(0, viewport);
  ds.cmd.setScissor(0, scissor);

  vk::RenderPassBeginInfo rp_info{
      .renderPass = *rp,
      .framebuffer = *fbs[swap ? ds.img_ind : 0],
      .renderArea = {
          .offset = {0, 0},
          .extent = size,
      }};
  rp_info.setClearValues(clears);
  ds.cmd.beginRenderPass(rp_info, vk::SubpassContents::eInline);
}

void Fbo::resetImages() {
  colors.clear();
  resolves.clear();
  depth = {};
  fbs.clear();  // Framebuffers reference the ImageViews
}

void Fbo::initImages(const VulkanState& vs) {
  for (auto& format : color_fmts) {
    Texture color{
        .size = size,
        .format = format,
        .samples = samples,
    };
    auto usage = vk::ImageUsageFlagBits::eColorAttachment |
                 (resolve ? vk::ImageUsageFlagBits::eTransientAttachment
                          : vk::ImageUsageFlagBits::eSampled);
    createImage(
        vs, color, vk::ImageTiling::eOptimal, usage,
        vk::MemoryPropertyFlagBits::eDeviceLocal,
        vk::ImageAspectFlagBits::eColor, output_sampler);
    colors.push_back(std::move(color));

    if (resolve && !swap) {
      Texture resolve{
          .size = size,
          .format = format,
          .samples = vk::SampleCountFlagBits::e1,
      };
      createImage(
          vs, resolve, vk::ImageTiling::eOptimal,
          vk::ImageUsageFlagBits::eColorAttachment |
              vk::ImageUsageFlagBits::eSampled,
          vk::MemoryPropertyFlagBits::eDeviceLocal,
          vk::ImageAspectFlagBits::eColor, output_sampler);
      resolves.push_back(std::move(resolve));
    }
  }

  if (depth_fmt) {
    depth = {
        .size = size,
        .format = *depth_fmt,
        .samples = samples,
    };
    createImage(
        vs, depth, vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eDepthStencilAttachment,
        vk::MemoryPropertyFlagBits::eDeviceLocal,
        vk::ImageAspectFlagBits::eDepth, {});
  }
}

void Fbo::initDescs(const VulkanState& vs) {
  if (desc_count == 0) {
    return;
  }

  output_set = {
      .binds =
          {color_fmts.size(),
           Binding{.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };
  output_set.init(vs);
  output_set.alloc(vs, desc_count);
  updateDescs(vs);
}

void Fbo::updateDescs(const VulkanState& vs) {
  if (desc_count == 0) {
    return;
  }

  std::vector<vk::WriteDescriptorSet> writes;

  auto& outputs = resolve ? resolves : colors;
  for (int i = 0; i < outputs.size(); i++) {
    output_set.updateSamplerBind(i, &outputs[i].info, writes);
  }

  vs.device.updateDescriptorSets(writes, nullptr);
}

void Fbo::initRp(const VulkanState& vs) {
  vk::ClearValue color_clear{{0.f, 0.f, 0.f, 1.f}};
  vk::ClearValue depth_clear{{1.f, 0}};

  std::vector<vk::AttachmentDescription> atts;
  std::vector<vk::AttachmentReference> color_refs;
  std::vector<vk::AttachmentReference> resolve_refs;
  for (auto& format : color_fmts) {
    color_refs.push_back({
        .attachment = static_cast<uint32_t>(atts.size()),
        .layout = vk::ImageLayout::eColorAttachmentOptimal,
    });
    atts.push_back({
        .format = format,
        .samples = samples,
        .loadOp = vk::AttachmentLoadOp::eClear,
        .storeOp = vk::AttachmentStoreOp::eStore,
        .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
        .initialLayout = vk::ImageLayout::eUndefined,
        .finalLayout = resolve ? vk::ImageLayout::eColorAttachmentOptimal
                               : vk::ImageLayout::eShaderReadOnlyOptimal,
    });
    clears.push_back(color_clear);
  }

  if (resolve) {
    for (auto& format : color_fmts) {
      if (!swap) {
        resolve_refs.push_back({
            .attachment = static_cast<uint32_t>(atts.size()),
            .layout = vk::ImageLayout::eColorAttachmentOptimal,
        });
        atts.push_back({
            .format = format,
            .samples = vk::SampleCountFlagBits::e1,
            .loadOp = vk::AttachmentLoadOp::eDontCare,
            .storeOp = vk::AttachmentStoreOp::eStore,
            .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
            .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
            .initialLayout = vk::ImageLayout::eUndefined,
            .finalLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
        });
      }
      // Clear not used for resolves.
      clears.push_back({});
    }
  }

  // This won't be used when not depth testing.
  vk::AttachmentReference depth_ref{
      .attachment = static_cast<uint32_t>(atts.size()),
      .layout = vk::ImageLayout::eDepthStencilAttachmentOptimal,
  };
  if (depth_fmt) {
    atts.push_back({
        .format = depth.format,
        .samples = depth.samples,
        .loadOp = vk::AttachmentLoadOp::eClear,
        .storeOp = vk::AttachmentStoreOp::eDontCare,
        .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
        .initialLayout = vk::ImageLayout::eUndefined,
        .finalLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal,
    });
    clears.push_back(depth_clear);
  }

  if (swap) {
    vk::AttachmentReference ref{
        .attachment = static_cast<uint32_t>(atts.size()),
        .layout = vk::ImageLayout::eColorAttachmentOptimal,
    };
    if (resolve) {
      resolve_refs.push_back(ref);
    } else {
      color_refs.push_back(ref);
    }
    atts.push_back({
        .format = swap_format,
        .samples = vk::SampleCountFlagBits::e1,
        .loadOp = vk::AttachmentLoadOp::eDontCare,
        .storeOp = vk::AttachmentStoreOp::eStore,
        .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
        .initialLayout = vk::ImageLayout::eUndefined,
        .finalLayout = vk::ImageLayout::ePresentSrcKHR,
    });
    clears.push_back({});
  }

  vk::SubpassDescription subpass{
      .pipelineBindPoint = vk::PipelineBindPoint::eGraphics};
  subpass.setResolveAttachments(resolve_refs);
  subpass.setColorAttachments(color_refs);
  subpass.setPDepthStencilAttachment(depth_fmt ? &depth_ref : nullptr);

  std::vector<vk::SubpassDependency> deps;
  // This render pass's write should finish before it's read from.
  deps.push_back({
      .srcSubpass = 0,
      .dstSubpass = VK_SUBPASS_EXTERNAL,
      .srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput,
      .dstStageMask = vk::PipelineStageFlagBits::eFragmentShader,
      .srcAccessMask = vk::AccessFlagBits::eColorAttachmentWrite,
      .dstAccessMask = vk::AccessFlagBits::eShaderRead,
  });
  if (depth_fmt) {
    // Previous depth tests should finish before we clear the depth buffer.
    deps.push_back({
        .srcSubpass = VK_SUBPASS_EXTERNAL,
        .dstSubpass = 0,
        .srcStageMask = vk::PipelineStageFlagBits::eEarlyFragmentTests,
        .dstStageMask = vk::PipelineStageFlagBits::eEarlyFragmentTests,
        .srcAccessMask = {},
        .dstAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentWrite,
    });
  }
  if (swap) {
    // Wait for swapchain image to be acquired before writing to it.
    deps.push_back({
        .srcSubpass = VK_SUBPASS_EXTERNAL,
        .dstSubpass = 0,
        .srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput,
        .dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput,
        .srcAccessMask = {},
        .dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite,
    });
  }

  vk::RenderPassCreateInfo rp_ci{};
  rp_ci.setAttachments(atts);
  rp_ci.setSubpasses(subpass);
  rp_ci.setDependencies(deps);
  rp = vs.device.createRenderPassUnique(rp_ci).value;
}

void Fbo::initFb(const VulkanState& vs) {
  vk::FramebufferCreateInfo fb_ci{
      .renderPass = *rp,
      .width = size.width,
      .height = size.height,
      .layers = 1,
  };
  std::vector<vk::ImageView> views;
  for (auto& texture : colors) {
    views.push_back(*texture.image_view);
  }
  for (auto& texture : resolves) {
    views.push_back(*texture.image_view);
  }
  if (depth_fmt) {
    views.push_back(*depth.image_view);
  }
  if (swap) {
    views.push_back({});
    for (auto& view : swap_views) {
      views.back() = view;
      fb_ci.setAttachments(views);
      fbs.push_back(vs.device.createFramebufferUnique(fb_ci).value);
    }
  } else {
    fb_ci.setAttachments(views);
    fbs.push_back(vs.device.createFramebufferUnique(fb_ci).value);
  }
}
