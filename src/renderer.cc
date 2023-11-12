#pragma once

#include "renderer.h"

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>
#include <SDL_vulkan.h>
#include <imgui/backends/imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <span>
#include <vector>

#include "asserts.h"
#include "defines.h"
#include "descriptors.h"
#include "fbo.h"
#include "files.h"
#include "glm-include.h"
#include "images.h"
#include "pipelines.h"
#include "render-objects.h"
#include "strings.h"
#include "vulkan-include.h"

namespace {

constexpr int MAX_FRAMES_IN_FLIGHT = 2;

const std::vector<const char*> validation_layers = {
    "VK_LAYER_KHRONOS_validation"};
const std::vector<const char*> device_extensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
#if __APPLE__
    "VK_KHR_portability_subset",
#endif
};

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT msg_severity,
    VkDebugUtilsMessageTypeFlagsEXT msg_type,
    const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
    void* user_data) {
  printf("dbg_layer: %s\n\n", callback_data->pMessage);
  return VK_FALSE;
}

std::vector<vk::DescriptorBufferInfo*> uboInfos(
    std::vector<DynamicBuf>& dbufs) {
  std::vector<vk::DescriptorBufferInfo*> infos;
  for (auto& dbuf : dbufs) {
    infos.push_back(&dbuf.device.info);
  }
  return infos;
}

std::vector<vk::ImageView> views(
    const std::vector<vk::UniqueImageView>& unique_views) {
  std::vector<vk::ImageView> views;
  for (auto& view : unique_views) {
    views.push_back(*view);
  }
  return views;
}

}  // namespace

void Renderer::init(FrameState* frame_state) {
  frame_state_ = frame_state;
  initVulkan();
  initSdlImage();
  initImgui();
}

void Renderer::drawFrame(FrameState* frame_state) {
  frame_state_ = frame_state;
  drawFrame();
}

void Renderer::cleanup() {
  std::ignore = device_->waitIdle();
  ImGui_ImplVulkan_Shutdown();
}

void Renderer::resizeWindow(uint32_t width, uint32_t height) {
  if (width_ == width && height_ == height) {
    return;
  }
  window_resized_ = true;
  width_ = width;
  height_ = height;
}

MaterialId Renderer::useMaterial(const MaterialInfo& mat_info) {
  loadMaterial(mat_info);
  return loaded_materials_.size() - 1;
}

void Renderer::useMesh(ModelId model_id, const Mesh& mesh, MaterialId mat_id) {
  auto model = loadMesh(mesh);
  model->material = loaded_materials_[mat_id].get();
  loaded_models_[model_id] = std::move(model);
}

void Renderer::setModelMaterial(ModelId model_id, MaterialId mat_id) {
  ASSERT(mat_id < loaded_materials_.size());
  auto model = loaded_models_.find(model_id);
  ASSERT(model != loaded_models_.end());

  model->second->material = loaded_materials_[mat_id].get();
}

// TODO: Return a TextureId instead.
Texture* Renderer::getDrawingTexture() {
  return &drawing_.pass.fbo.colors[0];
}
// TODO: Return a TextureId instead.
Texture* Renderer::getVoronoiTexture() {
  return &voronoi_.pass.fbo.colors[0];
}

void Renderer::initVulkan() {
  createInstance();
  if (enable_validation_layers_) {
    setupDebugMessenger();
  }
  createSurface();
  pickPhysicalDevice();
  createLogicalDevice();
  createCommandPool();
  createCommandBuffers();
  createSwapchain();
  createSamplers();
  createDescriptorPool();
  createImguiDescriptorPool();
  createSyncObjects();
  createShaders();
  // VulkanState should now fully defined.

  createDrawing();
  createVoronoi();
  createScene();
  createPost();
  createSwap();
}

void Renderer::initImgui() {
  ImGui_ImplVulkan_InitInfo init_info{
      .Instance = *instance_,
      .PhysicalDevice = physical_device_,
      .Device = *device_,
      .Queue = gfx_q_,
      .DescriptorPool = *imgui_desc_pool_,
      .MinImageCount = MAX_FRAMES_IN_FLIGHT,
      .ImageCount = MAX_FRAMES_IN_FLIGHT,
      .MSAASamples = (VkSampleCountFlagBits)swap_.pass.fbo.samples,
  };
  ImGui_ImplVulkan_Init(&init_info, *swap_.pass.fbo.rp);

  auto cmd_buf = beginSingleTimeCommands();
  ImGui_ImplVulkan_CreateFontsTexture(cmd_buf);
  endSingleTimeCommands(cmd_buf);
  ImGui_ImplVulkan_DestroyFontUploadObjects();
}

void Renderer::imguiNewFrame() {
  ImGui_ImplVulkan_NewFrame();
}

void Renderer::drawFrame() {
  if (window_resized_) {
    recreateSwapchain();
  }

  ds_.frame = frame_state_->frame_num % MAX_FRAMES_IN_FLIGHT;
  ds_.cmd = *cmd_bufs_[ds_.frame];

  std::ignore = device_->waitForFences(
      *in_flight_fences_[ds_.frame], VK_TRUE, UINT64_MAX);

  vk::Result result;
  std::tie(result, ds_.img_ind) = device_->acquireNextImageKHR(
      *swapchain_, UINT64_MAX, *img_sems_[ds_.frame], {});
  if (result == vk::Result::eErrorOutOfDateKHR) {
    window_resized_ = true;
    return;
  }
  ASSERT(
      result == vk::Result::eSuccess || result == vk::Result::eSuboptimalKHR);
  DASSERT(ds_.img_ind < swapchain_views_.size());

  // Only reset the fence if we're submitting work.
  device_->resetFences(*in_flight_fences_[ds_.frame]);

  ds_.cmd.reset();
  vk::CommandBufferBeginInfo begin_info{};
  std::ignore = ds_.cmd.begin(begin_info);

  if (frame_state_->update_drawing) {
    drawing_.update(ds_, frame_state_->drawing);
  }
  scene_.update(ds_, *frame_state_);
  post_.update(ds_, frame_state_->post);
  recordCommandBuffer();

  vk::SubmitInfo submit_info{};
  vk::PipelineStageFlags wait_stages =
      vk::PipelineStageFlagBits::eColorAttachmentOutput;
  submit_info.setWaitDstStageMask(wait_stages);
  submit_info.setWaitSemaphores(*img_sems_[ds_.frame]);
  submit_info.setCommandBuffers(*cmd_bufs_[ds_.frame]);
  submit_info.setSignalSemaphores(*render_sems_[ds_.frame]);
  std::ignore = gfx_q_.submit(submit_info, *in_flight_fences_[ds_.frame]);

  vk::PresentInfoKHR present_info{};
  present_info.setWaitSemaphores(*render_sems_[ds_.frame]);
  present_info.setSwapchains(*swapchain_);
  present_info.setImageIndices(ds_.img_ind);

  // For some reason Vulkan.hpp asserts on eErrorOutOfDateKHR, so until that's
  // dealt with, I need to use the C interface for present.
  result = (vk::Result)vkQueuePresentKHR(
      present_q_, (VkPresentInfoKHR*)&present_info);
  // result = present_q_.presentKHR(present_info); // This crashes.

  if (result == vk::Result::eErrorOutOfDateKHR ||
      result == vk::Result::eSuboptimalKHR) {
    window_resized_ = true;
  }

  // Reset DrawState in case something tries to use it outside of drawFrame().
  ds_ = {};
}

void Renderer::createInstance() {
  printSupportedExtensions();

  vk::ApplicationInfo app_info{
      .pApplicationName = "Hello Triangle",
      .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
      .pEngineName = "No Engine",
      .engineVersion = VK_MAKE_VERSION(1, 0, 0),
      .apiVersion = VK_API_VERSION_1_0,
  };
  vk::InstanceCreateInfo instance_ci{.pApplicationInfo = &app_info};

  auto ext_names = getRequiredExtensions();
  instance_ci.setPEnabledExtensionNames(ext_names);

  auto validation_layers = getValidationLayers();
  instance_ci.setPEnabledLayerNames(validation_layers);

  vk::DebugUtilsMessengerCreateInfoEXT dbg_messenger_ci{};
  if (enable_validation_layers_) {
    dbg_messenger_ci = makeDbgMessengerCi();
    instance_ci.pNext = &dbg_messenger_ci;
  }

#if __APPLE__
  instance_ci.flags |= vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR;
#endif

  instance_ = vk::createInstanceUnique(instance_ci).value;
  dldi_ = {*instance_, vkGetInstanceProcAddr};
}

std::vector<const char*> Renderer::getRequiredExtensions() {
  uint32_t ext_count = 0;
  ASSERT(SDL_Vulkan_GetInstanceExtensions(nullptr, &ext_count, nullptr));
  std::vector<const char*> ext_names(ext_count);
  ASSERT(
      SDL_Vulkan_GetInstanceExtensions(nullptr, &ext_count, ext_names.data()));

#if __APPLE__
  ext_names.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
  ext_names.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
#endif

  if (enable_validation_layers_) {
    ext_names.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  }

#ifdef DEBUG
  printf("Required instance extensions (%zu):\n", ext_names.size());
  for (auto& name : ext_names) {
    printf("  %s\n", name);
  }
#endif

  return ext_names;
}

void Renderer::printSupportedExtensions() {
  std::vector<vk::ExtensionProperties> sup_exts =
      vk::enumerateInstanceExtensionProperties().value;
  printf("Supported instance extensions (%zd)\n", sup_exts.size());
  for (auto& ext : sup_exts) {
    printf("  %s v%d\n", ext.extensionName.data(), ext.specVersion);
  }
}

std::vector<const char*> Renderer::getValidationLayers() {
  if (!enable_validation_layers_) {
    return {};
  }

  std::vector<vk::LayerProperties> layer_props =
      vk::enumerateInstanceLayerProperties().value;

  printf("Available layers (%zu):\n", layer_props.size());
  for (const auto& layer_prop : layer_props) {
    printf("  %s\n", layer_prop.layerName.data());
  }

  for (const auto& layer : validation_layers) {
    bool found = false;
    for (const auto& layer_prop : layer_props) {
      if (strcmp(layer, layer_prop.layerName) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      printf("Missing required validation layer: %s\n", layer);
      ASSERT(false);
    }
  }

  printf("Required layers (%zu):\n", validation_layers.size());
  for (const auto& layer : validation_layers) {
    printf("  %s\n", layer);
  }

  return validation_layers;
}

vk::DebugUtilsMessengerCreateInfoEXT Renderer::makeDbgMessengerCi() {
  vk::DebugUtilsMessengerCreateInfoEXT ci{
      .messageSeverity =
          // vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose |  // toggle
      vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning |
      vk::DebugUtilsMessageSeverityFlagBitsEXT::eError,
      .messageType = vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral |
                     vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation |
                     vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance,
      .pfnUserCallback = debugCallback,
  };
  return ci;
}

void Renderer::setupDebugMessenger() {
  auto ci = makeDbgMessengerCi();
  dbg_messenger_ =
      instance_->createDebugUtilsMessengerEXTUnique(ci, nullptr, dldi_).value;
}

void Renderer::createSurface() {
  VkSurfaceKHR surface;
  ASSERT(SDL_Vulkan_CreateSurface(window_, *instance_, &surface));
  surface_ = vk::UniqueSurfaceKHR(surface, *instance_);
}

void Renderer::pickPhysicalDevice() {
  std::vector<vk::PhysicalDevice> devices =
      instance_->enumeratePhysicalDevices().value;
  ASSERT(devices.size() > 0);

  for (const auto& device : devices) {
    QueueFamilyIndices indices = findQueueFamilies(device);
    if (!indices.isComplete()) {
      continue;
    }
    if (!checkDeviceExtensionSupport(device)) {
      continue;
    }
    SwapchainSupportDetails swapchain_support = querySwapchainSupport(device);
    if (swapchain_support.formats.empty() ||
        swapchain_support.present_modes.empty()) {
      continue;
    }
    vk::PhysicalDeviceFeatures features = device.getFeatures();
    if (!features.samplerAnisotropy) {
      continue;
    }

    physical_device_ = device;
    q_indices_ = indices;
    swapchain_support_ = swapchain_support;
    break;
  }
  ASSERT(physical_device_);
  vs_.device_props = physical_device_.getProperties();
  vs_.mem_props = physical_device_.getMemoryProperties();
  msaa_samples_ = getMaxSampleCount();
  printf("max msaa samples: %d\n", msaa_samples_);

#ifdef DEBUG
  printf("Supported formats (%zd)\n", swapchain_support_.formats.size());
  for (const auto& format : swapchain_support_.formats) {
    printf("  %d", format.format);
  }
  printf("\n");
#endif
}

vk::SampleCountFlagBits Renderer::getMaxSampleCount() {
  auto count_limit = vs_.device_props.limits.framebufferColorSampleCounts &
                     vs_.device_props.limits.framebufferDepthSampleCounts;

  constexpr std::array<vk::SampleCountFlagBits, 6> possible_counts = {
      vk::SampleCountFlagBits::e64, vk::SampleCountFlagBits::e32,
      vk::SampleCountFlagBits::e16, vk::SampleCountFlagBits::e8,
      vk::SampleCountFlagBits::e4,  vk::SampleCountFlagBits::e2,
  };
  for (const auto count : possible_counts) {
    if (count_limit & count) {
      return count;
    }
  }
  return vk::SampleCountFlagBits::e1;
}

Renderer::QueueFamilyIndices Renderer::findQueueFamilies(
    vk::PhysicalDevice device) {
  auto q_families = device.getQueueFamilyProperties();

  QueueFamilyIndices indices;
  uint32_t i = 0;
  for (const auto& q_family : q_families) {
    if (q_family.queueFlags & vk::QueueFlagBits::eGraphics) {
      indices.gfx_family = i;
    }

    vk::Bool32 present_support =
        device.getSurfaceSupportKHR(i, *surface_).value;
    if (present_support) {
      indices.present_family = i;
    }

    if (indices.isComplete()) {
      break;
    }
    i++;
  }

  return indices;
}

bool Renderer::checkDeviceExtensionSupport(vk::PhysicalDevice device) {
  std::vector<vk::ExtensionProperties> extensions =
      device.enumerateDeviceExtensionProperties().value;

  // Map to just the names.
  std::vector<std::string> available_exts(extensions.size());
  std::transform(
      extensions.begin(), extensions.end(), available_exts.begin(),
      [](vk::ExtensionProperties ext) -> std::string {
        return ext.extensionName;
      });
  // Make a copy so we can sort it.
  std::vector<std::string> required_exts(
      device_extensions.begin(), device_extensions.end());
  std::sort(required_exts.begin(), required_exts.end());
  std::sort(available_exts.begin(), available_exts.end());
  return std::includes(
      available_exts.begin(), available_exts.end(), required_exts.begin(),
      required_exts.end());
}

Renderer::SwapchainSupportDetails Renderer::querySwapchainSupport(
    vk::PhysicalDevice device) {
  SwapchainSupportDetails details;
  details.caps = device.getSurfaceCapabilitiesKHR(*surface_).value;
  details.formats = device.getSurfaceFormatsKHR(*surface_).value;
  details.present_modes = device.getSurfacePresentModesKHR(*surface_).value;

  return details;
}

void Renderer::createLogicalDevice() {
  std::set<uint32_t> unique_q_indices = {
      q_indices_.gfx_family, q_indices_.present_family};
  float q_prio = 1.0f;
  std::vector<vk::DeviceQueueCreateInfo> device_q_cis;
  for (uint32_t q_index : unique_q_indices) {
    device_q_cis.push_back({
        .queueFamilyIndex = q_index,
        .queueCount = 1,
        .pQueuePriorities = &q_prio,
    });
  }

  vk::PhysicalDeviceFeatures device_features{.samplerAnisotropy = VK_TRUE};

  vk::DeviceCreateInfo device_ci{.pEnabledFeatures = &device_features};
  device_ci.setPEnabledExtensionNames(device_extensions);
  device_ci.setQueueCreateInfos(device_q_cis);
  if (enable_validation_layers_) {
    device_ci.setPEnabledLayerNames(validation_layers);
  }
  device_ = physical_device_.createDeviceUnique(device_ci).value;
  vs_.device = *device_;

  gfx_q_ = device_->getQueue(q_indices_.gfx_family, 0);
  ASSERT(gfx_q_);
  present_q_ = device_->getQueue(q_indices_.present_family, 0);
  ASSERT(present_q_);
}

vk::SurfaceFormatKHR Renderer::chooseSwapSurfaceFormat(
    const std::vector<vk::SurfaceFormatKHR>& formats) {
  DASSERT(formats.size());
  for (const auto& format : formats) {
    if (format.format == vk::Format::eB8G8R8A8Srgb &&
        format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
      return format;
    }
  }

  return formats[0];
}

vk::PresentModeKHR Renderer::chooseSwapPresentMode(
    const std::vector<vk::PresentModeKHR>& present_modes) {
  constexpr vk::PresentModeKHR preferred_mode = vk::PresentModeKHR::eMailbox;
  if (std::find(present_modes.begin(), present_modes.end(), preferred_mode) !=
      present_modes.end()) {
    return preferred_mode;
  }

  return vk::PresentModeKHR::eFifo;
}

vk::Extent2D Renderer::chooseSwapExtent(
    const vk::SurfaceCapabilitiesKHR& caps) {
  if (caps.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
    return caps.currentExtent;
  } else {
    vk::Extent2D extent{width_, height_};
    extent.width = std::clamp(
        extent.width, caps.minImageExtent.width, caps.maxImageExtent.width);
    extent.height = std::clamp(
        extent.height, caps.minImageExtent.height, caps.maxImageExtent.height);
    return extent;
  }
}

void Renderer::createSwapchain() {
  auto format = chooseSwapSurfaceFormat(swapchain_support_.formats);
  swapchain_format_ = format.format;
  auto present_mode = chooseSwapPresentMode(swapchain_support_.present_modes);
  swapchain_extent_ = chooseSwapExtent(swapchain_support_.caps);

  uint32_t image_count = swapchain_support_.caps.minImageCount + 1;
  if (swapchain_support_.caps.maxImageCount > 0) {
    image_count = std::min(image_count, swapchain_support_.caps.maxImageCount);
  }

  vk::SwapchainCreateInfoKHR swapchain_ci{
      .surface = *surface_,
      .minImageCount = image_count,
      .imageFormat = swapchain_format_,
      .imageColorSpace = format.colorSpace,
      .imageExtent = swapchain_extent_,
      .imageArrayLayers = 1,
      .imageUsage = vk::ImageUsageFlagBits::eColorAttachment,
      .preTransform = swapchain_support_.caps.currentTransform,
      .compositeAlpha = vk::CompositeAlphaFlagBitsKHR::eOpaque,
      .presentMode = present_mode,
      .clipped = VK_TRUE,
      .oldSwapchain = nullptr,
  };

  std::array<uint32_t, 2> queue_family_indices{
      q_indices_.gfx_family, q_indices_.present_family};
  if (q_indices_.gfx_family != q_indices_.present_family) {
    swapchain_ci.imageSharingMode = vk::SharingMode::eConcurrent;
    swapchain_ci.setQueueFamilyIndices(queue_family_indices);
  } else {
    swapchain_ci.imageSharingMode = vk::SharingMode::eExclusive;
  }

  swapchain_ = device_->createSwapchainKHRUnique(swapchain_ci).value;
  auto swapchain_images = device_->getSwapchainImagesKHR(*swapchain_).value;

  swapchain_views_.resize(swapchain_images.size());
  for (size_t i = 0; i < swapchain_images.size(); i++) {
    swapchain_views_[i] = createImageView(
        vs_, swapchain_images[i], swapchain_format_, 1,
        vk::ImageAspectFlagBits::eColor);
  }

  printf(
      "Created %d swapchain images, format:%d extent:%dx%d\n", image_count,
      swapchain_format_, swapchain_extent_.width, swapchain_extent_.height);
}

void Renderer::createShaders() {
  scene_vert_ = createShaderModule("shaders/shader.vert.spv");
  scene_frag_ = createShaderModule("shaders/shader.frag.spv");
  fullscreen_vert_ = createShaderModule("shaders/fullscreen.vert.spv");
  post_frag_ = createShaderModule("shaders/post.frag.spv");
  circle_frag_ = createShaderModule("shaders/circle.frag.spv");
  sample_frag_ = createShaderModule("shaders/sample.frag.spv");
  voronoi_vert_ = createShaderModule("shaders/voronoi.vert.spv");
  voronoi_frag_ = createShaderModule("shaders/voronoi.frag.spv");
}

vk::UniqueShaderModule Renderer::createShaderModule(std::string filename) {
  auto code = readFile(filename);
  vk::ShaderModuleCreateInfo ci{
      .codeSize = code.size(),
      .pCode = reinterpret_cast<const uint32_t*>(code.data()),
  };
  return device_->createShaderModuleUnique(ci).value;
}

void Renderer::createDrawing() {
  auto& pass = drawing_.pass;
  pass.fbo = {
      .size = {512, 512},
      .color_fmts = {color_fmt_},
      .output_sampler = *linear_sampler_,
      .desc_count = MAX_FRAMES_IN_FLIGHT,
  };

  drawing_.inputs = pass.makeDescLayout();
  *drawing_.inputs = {
      .binds = {{.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  drawing_.draw = drawing_.pass.makePipeline();
  *drawing_.draw = {
      .vert_shader = *fullscreen_vert_,
      .frag_shader = *circle_frag_,
      .desc_layouts = {drawing_.inputs},
  };

  initPass(pass);

  for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
    drawing_.debugs.push_back(createDynamicBuffer(
        sizeof(DebugData), vk::BufferUsageFlagBits::eUniformBuffer));
  }

  // TODO: Generalize so this can be part of initPass()
  std::vector<vk::WriteDescriptorSet> writes;
  drawing_.inputs->updateUboBind(0, uboInfos(drawing_.debugs), writes);
  device_->updateDescriptorSets(writes, nullptr);
}

void Renderer::Drawing::update(const DrawState& ds, const DebugData& debug) {
  updateDynamicBuf(
      ds.cmd, debugs[ds.frame], std::span<const DebugData>(&debug, 1),
      vk::PipelineStageFlagBits::eFragmentShader,
      vk::AccessFlagBits::eUniformRead);
}

void Renderer::initPass(Pass& pass) {
  pass.fbo.init(vs_);

  for (auto& lo : pass.los) {
    lo->init(vs_);
    lo->alloc(vs_, MAX_FRAMES_IN_FLIGHT);
  }

  for (auto& pl : pass.pls) {
    initPipeline(*device_, pass.fbo, *pl);
  }
}

void Renderer::createVoronoi() {
  auto& pass = voronoi_.pass;
  pass.fbo = {
      .size = {512, 512},
      .color_fmts = {color_fmt_},
      .depth_fmt = findDepthFormat(),
      .output_sampler = *linear_sampler_,
  };

  vk::PipelineVertexInputStateCreateInfo vert_in{};
  auto vert_bind = getBindingDesc<Vertex2d>();
  vert_bind.inputRate = vk::VertexInputRate::eInstance;
  vert_in.setVertexBindingDescriptions(vert_bind);
  auto vert_attrs = getAttrDescs<Vertex2d>();
  vert_in.setVertexAttributeDescriptions(vert_attrs);

  voronoi_.draw = pass.makePipeline();
  *voronoi_.draw = {
      .vert_shader = *voronoi_vert_,
      .frag_shader = *voronoi_frag_,
      .vert_in = vert_in,
  };

  // Support up to 100 voronoi cells for now.
  vk::DeviceSize size = 100 * sizeof(Vertex2d);
  for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
    voronoi_.verts.push_back(
        createDynamicBuffer(size, vk::BufferUsageFlagBits::eVertexBuffer));
  }

  initPass(voronoi_.pass);
}

void Renderer::createScene() {
  auto& pass = scene_.pass;
  pass.fbo = {
      .size = swapchain_extent_,
      .color_fmts = {color_fmt_, vk::Format::eR32G32B32A32Sfloat},
      // Disable msaa for now because it makes outline filter worse
      // .samples = msaa_samples_,
      // .resolve = true,
      .depth_fmt = findDepthFormat(),
      .output_sampler = *linear_sampler_,
      .desc_count = MAX_FRAMES_IN_FLIGHT,
  };

  // Bound per frame.
  scene_.global = pass.makeDescLayout();
  *scene_.global = {
      .binds = {{.type = vk::DescriptorType::eUniformBuffer}},
      .stages =
          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
  };
  // Bound per material
  // TODO: Don't allocate any here. These sets are allocated per material.
  scene_.material = pass.makeDescLayout();
  *scene_.material = {
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

  scene_.draw = pass.makePipeline();
  *scene_.draw = {
      .vert_shader = *scene_vert_,
      .frag_shader = *scene_frag_,
      .desc_layouts = {scene_.global, scene_.material},
      .push_ranges = {scene_push},
      .vert_in = vertex_in,
      .cull_mode = vk::CullModeFlagBits::eNone,
  };

  initPass(pass);

  for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
    scene_.globals.push_back(createDynamicBuffer(
        sizeof(GlobalData), vk::BufferUsageFlagBits::eUniformBuffer));
  }

  std::vector<vk::WriteDescriptorSet> writes;
  scene_.global->updateUboBind(0, uboInfos(scene_.globals), writes);
  device_->updateDescriptorSets(writes, nullptr);
}

void Renderer::Scene::update(const DrawState& ds, const FrameState& fs) {
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

void Renderer::createPost() {
  auto& pass = post_.pass;
  pass.fbo = {
      .size = swapchain_extent_,
      .color_fmts = {color_fmt_},
      .output_sampler = *linear_sampler_,
      .desc_count = MAX_FRAMES_IN_FLIGHT,
  };

  post_.inputs = pass.makeDescLayout();
  *post_.inputs = {
      .binds =
          {{.type = vk::DescriptorType::eUniformBuffer},
           {.type = vk::DescriptorType::eUniformBuffer}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  post_.draw = pass.makePipeline();
  *post_.draw = {
      .vert_shader = *fullscreen_vert_,
      .frag_shader = *post_frag_,
      .desc_layouts = {post_.inputs, &scene_.pass.fbo.output_set},
      .vert_in = {},
  };

  initPass(pass);

  for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
    post_.debugs.push_back(createDynamicBuffer(
        sizeof(DebugData), vk::BufferUsageFlagBits::eUniformBuffer));
  }

  std::vector<vk::WriteDescriptorSet> writes;
  post_.inputs->updateUboBind(0, uboInfos(scene_.globals), writes);
  post_.inputs->updateUboBind(1, uboInfos(post_.debugs), writes);
  device_->updateDescriptorSets(writes, nullptr);
}

void Renderer::Post::update(const DrawState& ds, const DebugData& debug) {
  auto stages = vk::PipelineStageFlagBits::eVertexShader |
                vk::PipelineStageFlagBits::eFragmentShader;
  updateDynamicBuf(
      ds.cmd, debugs[ds.frame], std::span<const DebugData>(&debug, 1), stages,
      vk::AccessFlagBits::eUniformRead);
}

void Renderer::createSwap() {
  auto& pass = swap_.pass;

  pass.fbo = {
      .size = swapchain_extent_,
      .swap = true,
      .swap_format = swapchain_format_,
      .swap_views = views(swapchain_views_),
  };

  swap_.sampler = pass.makeDescLayout();
  *swap_.sampler = {
      .binds = {{.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  swap_.draw = pass.makePipeline();
  *swap_.draw = {
      .vert_shader = *fullscreen_vert_,
      .frag_shader = *sample_frag_,
      .desc_layouts = {swap_.sampler},
      .vert_in = {},
  };

  initPass(pass);
}

void Renderer::createCommandPool() {
  vk::CommandPoolCreateInfo ci{
      .flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
      .queueFamilyIndex = q_indices_.gfx_family,
  };
  cmd_pool_ = device_->createCommandPoolUnique(ci).value;
}

vk::Format Renderer::findDepthFormat() {
  return findSupportedFormat(
      {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint,
       vk::Format::eD24UnormS8Uint},
      vk::ImageTiling::eOptimal,
      vk::FormatFeatureFlagBits::eDepthStencilAttachment);
}

bool Renderer::hasStencilComponent(vk::Format format) {
  return format == vk::Format::eD32SfloatS8Uint ||
         format == vk::Format::eD24UnormS8Uint;
}

vk::Format Renderer::findSupportedFormat(
    const std::vector<vk::Format>& formats, vk::ImageTiling tiling,
    vk::FormatFeatureFlags features) {
  ASSERT(
      tiling == vk::ImageTiling::eLinear ||
      tiling == vk::ImageTiling::eOptimal);

  for (const auto& format : formats) {
    vk::FormatProperties props = physical_device_.getFormatProperties(format);

    auto tiling_features = tiling == vk::ImageTiling::eLinear
                               ? props.linearTilingFeatures
                               : props.optimalTilingFeatures;
    if ((tiling_features & features) == features) {
      return format;
    }
  }

  printf("failed to find supported format!\n");
  ASSERT(false);
  return vk::Format::eUndefined;
}

void Renderer::initSdlImage() {
  if (!IMG_Init(IMG_INIT_JPG)) {
    printf("%s", IMG_GetError());
    ASSERT(false);
  }
}

SDL_Surface* Renderer::loadImage(std::string texture_path) {
  SDL_Surface* texture_surface = IMG_Load(texture_path.c_str());
  ASSERT(texture_surface);
  ASSERT(texture_surface->pixels);
  // Vulkan likes images to have alpha channels. The SDL byte order is also
  // defined opposite to vk::Format.
  SDL_PixelFormatEnum desired_fmt = SDL_PIXELFORMAT_ARGB8888;
  if (texture_surface->format->format != desired_fmt) {
    printf(
        "converting image pixel format from %s to %s\n",
        SDL_GetPixelFormatName(texture_surface->format->format),
        SDL_GetPixelFormatName(desired_fmt));
    auto* new_surface =
        SDL_ConvertSurfaceFormat(texture_surface, desired_fmt, 0);
    SDL_FreeSurface(texture_surface);
    texture_surface = new_surface;
  }

  return texture_surface;
}

Texture* Renderer::createTexture(
    void* texture_data, uint32_t width, uint32_t height) {
  auto texture = std::make_unique<Texture>();
  texture->size = {width, height};
  texture->format = vk::Format::eB8G8R8A8Srgb;
  texture->mip_levels = std::floor(std::log2(std::max(width, height))) + 1;

  vk::UniqueBuffer staging_buf;
  vk::UniqueDeviceMemory staging_buf_mem;
  vk::DeviceSize image_size = width * height * 4;
  createBuffer(
      image_size, vk::BufferUsageFlagBits::eTransferSrc,
      vk::MemoryPropertyFlagBits::eHostVisible |
          vk::MemoryPropertyFlagBits::eHostCoherent,
      staging_buf, staging_buf_mem);

  void* mapped_data = device_->mapMemory(*staging_buf_mem, 0, image_size).value;
  memcpy(mapped_data, texture_data, static_cast<size_t>(image_size));
  device_->unmapMemory(*staging_buf_mem);

  createImage(
      vs_, *texture.get(), vk::ImageTiling::eOptimal,
      vk::ImageUsageFlagBits::eTransferSrc |
          vk::ImageUsageFlagBits::eTransferDst |
          vk::ImageUsageFlagBits::eSampled,
      vk::MemoryPropertyFlagBits::eDeviceLocal, vk::ImageAspectFlagBits::eColor,
      *linear_sampler_);

  transitionImageLayout(
      *texture->image, texture->format, texture->mip_levels,
      vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal);
  copyBufferToImage(*staging_buf, *texture->image, width, height);
  generateMipmaps(
      *texture->image, width, height, texture->format, texture->mip_levels);
  // Transitioned to vk::ImageLayout::eShaderReadOnlyOptimal while generating
  // mipmaps.

  auto* ptr = texture.get();
  loaded_textures_.push_back(std::move(texture));
  return ptr;
}

void Renderer::transitionImageLayout(
    vk::Image img, vk::Format format, uint32_t mip_levels,
    vk::ImageLayout old_layout, vk::ImageLayout new_layout) {
  vk::CommandBuffer cmd_buf = beginSingleTimeCommands();

  vk::ImageMemoryBarrier barrier{
      .oldLayout = old_layout,
      .newLayout = new_layout,
      .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .image = img,
      .subresourceRange = {
          .baseMipLevel = 0,
          .levelCount = mip_levels,
          .baseArrayLayer = 0,
          .layerCount = 1,
      }};

  if (new_layout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
    barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
    if (hasStencilComponent(format)) {
      barrier.subresourceRange.aspectMask |= vk::ImageAspectFlagBits::eStencil;
    }
  } else {
    barrier.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eColor;
  }

  vk::PipelineStageFlags src_stage;
  vk::PipelineStageFlags dst_stage;
  if (old_layout == vk::ImageLayout::eUndefined &&
      new_layout == vk::ImageLayout::eTransferDstOptimal) {
    barrier.srcAccessMask = {};
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferWrite;
    src_stage = vk::PipelineStageFlagBits::eTopOfPipe;
    dst_stage = vk::PipelineStageFlagBits::eTransfer;
  } else if (
      old_layout == vk::ImageLayout::eTransferDstOptimal &&
      new_layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
    src_stage = vk::PipelineStageFlagBits::eTransfer;
    dst_stage = vk::PipelineStageFlagBits::eFragmentShader;
  } else if (
      old_layout == vk::ImageLayout::eUndefined &&
      new_layout == vk::ImageLayout::eDepthStencilAttachmentOptimal) {
    barrier.srcAccessMask = {};
    barrier.dstAccessMask = vk::AccessFlagBits::eDepthStencilAttachmentRead |
                            vk::AccessFlagBits::eDepthStencilAttachmentWrite;
    src_stage = vk::PipelineStageFlagBits::eTopOfPipe;
    dst_stage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
  } else {
    printf(
        "Unsupported layout transition! (%d -> %d)\n", old_layout, new_layout);
    ASSERT(false);
  }

  cmd_buf.pipelineBarrier(src_stage, dst_stage, {}, nullptr, nullptr, barrier);

  endSingleTimeCommands(cmd_buf);
}

void Renderer::copyBufferToImage(
    vk::Buffer buf, vk::Image img, uint32_t width, uint32_t height) {
  vk::CommandBuffer cmd_buf = beginSingleTimeCommands();

  vk::BufferImageCopy region{
      .bufferOffset = 0,
      .bufferRowLength = 0,
      .bufferImageHeight = 0,
      .imageSubresource =
          {
              .aspectMask = vk::ImageAspectFlagBits::eColor,
              .mipLevel = 0,
              .baseArrayLayer = 0,
              .layerCount = 1,
          },
      .imageOffset = {0, 0, 0},
      .imageExtent = {width, height, 1},
  };

  cmd_buf.copyBufferToImage(
      buf, img, vk::ImageLayout::eTransferDstOptimal, region);

  endSingleTimeCommands(cmd_buf);
}

void Renderer::generateMipmaps(
    vk::Image img, int32_t width, int32_t height, vk::Format format,
    uint32_t mip_levels) {
  vk::FormatProperties format_props =
      physical_device_.getFormatProperties(format);
  ASSERT(static_cast<bool>(
      format_props.optimalTilingFeatures &
      vk::FormatFeatureFlagBits::eSampledImageFilterLinear));

  vk::CommandBuffer cmd_buf = beginSingleTimeCommands();

  vk::ImageMemoryBarrier barrier{
      .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
      .image = img,
      .subresourceRange =
          {
              .aspectMask = vk::ImageAspectFlagBits::eColor,
              .levelCount = 1,
              .baseArrayLayer = 0,
              .layerCount = 1,
          },
  };

  int32_t mip_width = width;
  int32_t mip_height = height;
  for (uint32_t i = 0; i < mip_levels - 1; i++) {
    // Transition mip i to to be a copy source.
    barrier.subresourceRange.baseMipLevel = i;
    barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal;
    barrier.newLayout = vk::ImageLayout::eTransferSrcOptimal;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
    barrier.dstAccessMask = vk::AccessFlagBits::eTransferRead;
    cmd_buf.pipelineBarrier(
        vk::PipelineStageFlagBits::eTransfer,
        vk::PipelineStageFlagBits::eTransfer, {}, nullptr, nullptr, barrier);

    int32_t src_width = mip_width;
    int32_t src_height = mip_height;
    if (src_width > 1) {
      mip_width /= 2;
    }
    if (src_height > 1) {
      mip_height /= 2;
    }

    // Blit from mip i to mip i+1 at half the size.
    vk::ImageBlit blit{
        .srcSubresource =
            {
                .aspectMask = vk::ImageAspectFlagBits::eColor,
                .mipLevel = i,
                .baseArrayLayer = 0,
                .layerCount = 1,
            },
        .srcOffsets = {{
            vk::Offset3D{0, 0, 0},
            vk::Offset3D{src_width, src_height, 1},
        }},
        .dstSubresource =
            {
                .aspectMask = vk::ImageAspectFlagBits::eColor,
                .mipLevel = i + 1,
                .baseArrayLayer = 0,
                .layerCount = 1,
            },
        .dstOffsets = {{
            vk::Offset3D{0, 0, 0},
            vk::Offset3D{mip_width, mip_height, 1},
        }}};
    cmd_buf.blitImage(
        img, vk::ImageLayout::eTransferSrcOptimal, img,
        vk::ImageLayout::eTransferDstOptimal, blit, vk::Filter::eLinear);

    // Transition mip i to be shader readable.
    barrier.oldLayout = vk::ImageLayout::eTransferSrcOptimal;
    barrier.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
    barrier.srcAccessMask = vk::AccessFlagBits::eTransferRead;
    barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
    cmd_buf.pipelineBarrier(
        vk::PipelineStageFlagBits::eTransfer,
        vk::PipelineStageFlagBits::eFragmentShader, {}, nullptr, nullptr,
        barrier);
  }
  // Transition last mip level to be shader readable.
  barrier.subresourceRange.baseMipLevel = mip_levels - 1;
  barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal;
  barrier.newLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
  barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
  barrier.dstAccessMask = vk::AccessFlagBits::eShaderRead;
  cmd_buf.pipelineBarrier(
      vk::PipelineStageFlagBits::eTransfer,
      vk::PipelineStageFlagBits::eFragmentShader, {}, nullptr, nullptr,
      barrier);

  endSingleTimeCommands(cmd_buf);
}

void Renderer::createSamplers() {
  vk::SamplerCreateInfo ci{
      .magFilter = vk::Filter::eLinear,
      .minFilter = vk::Filter::eLinear,
      .mipmapMode = vk::SamplerMipmapMode::eLinear,
      .addressModeU = vk::SamplerAddressMode::eRepeat,
      .addressModeV = vk::SamplerAddressMode::eRepeat,
      .addressModeW = vk::SamplerAddressMode::eRepeat,
      .mipLodBias = 0.f,
      .anisotropyEnable = VK_TRUE,
      .maxAnisotropy = vs_.device_props.limits.maxSamplerAnisotropy,
      .compareEnable = VK_FALSE,
      .compareOp = vk::CompareOp::eAlways,
      .minLod = 0.f,
      .maxLod = 13.f,  // Somewhat arbitrary, based on log2(4096) + 1
      .borderColor = vk::BorderColor::eIntOpaqueBlack,
      .unnormalizedCoordinates = VK_FALSE,
  };
  linear_sampler_ = device_->createSamplerUnique(ci).value;
  ci.magFilter = vk::Filter::eNearest;
  ci.minFilter = vk::Filter::eNearest;
  ci.anisotropyEnable = VK_FALSE;
  nearest_sampler_ = device_->createSamplerUnique(ci).value;
}

Material* Renderer::loadMaterial(const MaterialInfo& mat_info) {
  auto material = std::make_unique<Material>();
  auto* ptr = material.get();

  if (mat_info.diffuse_texture) {
    material->diffuse = mat_info.diffuse_texture;
  } else if (mat_info.diffuse_path) {
    material->diffuse = loadTexture(*mat_info.diffuse_path);
  } else {
    // Use 1x1 pixel white texture when none is specified.
    material->diffuse = getColorTexture(0xFFFFFFFF);
  }

  stageBuffer(
      sizeof(MaterialData), (void*)&mat_info.data,
      vk::BufferUsageFlagBits::eUniformBuffer, material->ubo.buf,
      material->ubo.mem);
  material->ubo.info = vk::DescriptorBufferInfo{
      .buffer = *material->ubo.buf,
      .offset = 0,
      .range = sizeof(MaterialData),
  };

  material->desc_set = allocDescSet(vs_, *scene_.material->layout);
  std::vector<vk::WriteDescriptorSet> writes;
  updateDescSet(
      material->desc_set, *scene_.material,
      {&material->diffuse->info, &material->ubo.info}, writes);
  device_->updateDescriptorSets(writes, nullptr);

  loaded_materials_.push_back(std::move(material));

  return ptr;
}

Texture* Renderer::loadTexture(std::string path) {
  auto* texture_surface = loadImage(path);
  Texture* texture = createTexture(
      texture_surface->pixels, texture_surface->w, texture_surface->h);
  SDL_FreeSurface(texture_surface);

  return texture;
}

std::unique_ptr<Model> Renderer::loadMesh(const Mesh& mesh) {
  auto model = std::make_unique<Model>();

  model->vertex_count = mesh.vertices.size();
  stageVertices(mesh.vertices, *model);
  if (mesh.indices.size()) {
    model->index_count = mesh.indices.size();
    stageIndices(mesh.indices, *model);
  }

  return model;
}

Texture* Renderer::getColorTexture(uint32_t color) {
  auto it = color_textures_.find(color);
  if (it != color_textures_.end()) {
    return it->second;
  }

  // Create 1x1 color texture.
  auto* texture = createTexture(&color, 1, 1);
  color_textures_.emplace(color, texture);

  return texture;
}

DynamicBuf Renderer::createDynamicBuffer(
    vk::DeviceSize size, vk::BufferUsageFlags usage) {
  DynamicBuf dbuf;
  createBuffer(
      size, vk::BufferUsageFlagBits::eTransferSrc,
      vk::MemoryPropertyFlagBits::eHostVisible |
          vk::MemoryPropertyFlagBits::eHostCoherent,
      dbuf.staging.buf, dbuf.staging.mem);
  dbuf.staging.info.range = size;
  dbuf.staging.info.buffer = *dbuf.staging.buf;

  dbuf.staging.mapped =
      device_->mapMemory(*dbuf.staging.mem, dbuf.staging.info.offset, size)
          .value;

  createBuffer(
      size, usage | vk::BufferUsageFlagBits::eTransferDst,
      vk::MemoryPropertyFlagBits::eDeviceLocal, dbuf.device.buf,
      dbuf.device.mem);
  dbuf.device.info.range = size;
  dbuf.device.info.buffer = *dbuf.device.buf;

  return std::move(dbuf);
}

void Renderer::stageVertices(
    const std::vector<Vertex>& vertices, Model& model) {
  vk::DeviceSize size = sizeof(Vertex) * vertices.size();
  stageBuffer(
      size, (void*)vertices.data(), vk::BufferUsageFlagBits::eVertexBuffer,
      model.vert_buf, model.vert_buf_mem);
}

void Renderer::stageIndices(
    const std::vector<uint32_t>& indices, Model& model) {
  vk::DeviceSize size = sizeof(uint32_t) * indices.size();
  stageBuffer(
      size, (void*)indices.data(), vk::BufferUsageFlagBits::eIndexBuffer,
      model.ind_buf, model.ind_buf_mem);
}

// Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
// from the staging_buf to dst_buf.
// TODO: Reuse createDynamicBuffer() for this.
void Renderer::stageBuffer(
    vk::DeviceSize size, void* data, vk::BufferUsageFlags usage,
    vk::UniqueBuffer& dst_buf, vk::UniqueDeviceMemory& dst_buf_mem) {
  vk::UniqueBuffer staging_buf;
  vk::UniqueDeviceMemory staging_buf_mem;
  createBuffer(
      size, vk::BufferUsageFlagBits::eTransferSrc,
      vk::MemoryPropertyFlagBits::eHostVisible |
          vk::MemoryPropertyFlagBits::eHostCoherent,
      staging_buf, staging_buf_mem);

  void* staging_data = device_->mapMemory(*staging_buf_mem, 0, size).value;
  memcpy(staging_data, data, (size_t)size);
  device_->unmapMemory(*staging_buf_mem);

  createBuffer(
      size, usage | vk::BufferUsageFlagBits::eTransferDst,
      vk::MemoryPropertyFlagBits::eDeviceLocal, dst_buf, dst_buf_mem);

  vk::CommandBuffer cmd_buf = beginSingleTimeCommands();
  vk::BufferCopy copy_region{
      .size = size,
  };
  cmd_buf.copyBuffer(*staging_buf, *dst_buf, copy_region);
  endSingleTimeCommands(cmd_buf);
}

vk::CommandBuffer Renderer::beginSingleTimeCommands() {
  vk::CommandBufferAllocateInfo alloc_info{
      .commandPool = *cmd_pool_,
      .level = vk::CommandBufferLevel::ePrimary,
      .commandBufferCount = 1,
  };

  vk::CommandBuffer cmd_buf =
      device_->allocateCommandBuffers(alloc_info).value[0];

  vk::CommandBufferBeginInfo begin_info{
      .flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit};
  std::ignore = cmd_buf.begin(begin_info);

  return cmd_buf;
}

void Renderer::endSingleTimeCommands(vk::CommandBuffer cmd_buf) {
  std::ignore = cmd_buf.end();

  vk::SubmitInfo submit{};
  submit.setCommandBuffers(cmd_buf);
  std::ignore = gfx_q_.submit(submit);
  std::ignore = gfx_q_.waitIdle();

  device_->freeCommandBuffers(*cmd_pool_, cmd_buf);
}

// TODO: return a Buffer.
void Renderer::createBuffer(
    vk::DeviceSize size, vk::BufferUsageFlags usage,
    vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
    vk::UniqueDeviceMemory& buf_mem) {
  vk::BufferCreateInfo buffer_ci{
      .size = size,
      .usage = usage,
      .sharingMode = vk::SharingMode::eExclusive,
  };
  buf = device_->createBufferUnique(buffer_ci).value;

  vk::MemoryRequirements mem_reqs = device_->getBufferMemoryRequirements(*buf);

  vk::MemoryAllocateInfo alloc_info{
      .allocationSize = mem_reqs.size,
      .memoryTypeIndex =
          findMemoryType(mem_reqs.memoryTypeBits, props, vs_.mem_props),
  };
  buf_mem = device_->allocateMemoryUnique(alloc_info).value;
  std::ignore = device_->bindBufferMemory(*buf, *buf_mem, 0);
}

void Renderer::createDescriptorPool() {
  // Arbitrary. 100 seems fine for now.
  const uint32_t kMaxSamplers = 100;
  const uint32_t kMaxUbos = 100;

  std::vector<vk::DescriptorPoolSize> pool_sizes{
      {.type = vk::DescriptorType::eUniformBuffer, .descriptorCount = kMaxUbos},
      {.type = vk::DescriptorType::eCombinedImageSampler,
       .descriptorCount = kMaxSamplers},
  };

  vk::DescriptorPoolCreateInfo pool_ci{
      .maxSets = kMaxUbos + kMaxSamplers,
  };
  pool_ci.setPoolSizes(pool_sizes);
  desc_pool_ = device_->createDescriptorPoolUnique(pool_ci).value;
  vs_.desc_pool = *desc_pool_;
}

void Renderer::createImguiDescriptorPool() {
  const uint32_t kCount = 100;
  std::vector<vk::DescriptorPoolSize> pool_sizes{
      {.type = vk::DescriptorType::eSampler, .descriptorCount = kCount},
      {.type = vk::DescriptorType::eCombinedImageSampler,
       .descriptorCount = kCount},
      {.type = vk::DescriptorType::eSampledImage, .descriptorCount = kCount},
      {.type = vk::DescriptorType::eStorageImage, .descriptorCount = kCount},
      {.type = vk::DescriptorType::eUniformTexelBuffer,
       .descriptorCount = kCount},
      {.type = vk::DescriptorType::eStorageTexelBuffer,
       .descriptorCount = kCount},
      {.type = vk::DescriptorType::eUniformBuffer, .descriptorCount = kCount},
      {.type = vk::DescriptorType::eStorageBuffer, .descriptorCount = kCount},
      {.type = vk::DescriptorType::eUniformBufferDynamic,
       .descriptorCount = kCount},
      {.type = vk::DescriptorType::eInputAttachment, .descriptorCount = kCount},
  };

  const uint32_t kMaxSets = kCount * pool_sizes.size();
  vk::DescriptorPoolCreateInfo pool_ci{
      // TODO: Apparently FreeDescriptorSet can make allocations slower. Maybe
      // remove it.
      .flags = vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet,
      .maxSets = kMaxSets,
  };
  pool_ci.setPoolSizes(pool_sizes);
  imgui_desc_pool_ = device_->createDescriptorPoolUnique(pool_ci).value;
}

void Renderer::createCommandBuffers() {
  vk::CommandBufferAllocateInfo alloc_info{
      .commandPool = *cmd_pool_,
      .level = vk::CommandBufferLevel::ePrimary,
      .commandBufferCount = MAX_FRAMES_IN_FLIGHT,
  };
  cmd_bufs_ = device_->allocateCommandBuffersUnique(alloc_info).value;
}

void Renderer::beginRp(const Fbo& fbo, int fb_ind) {
  vk::Viewport viewport{
      .x = 0.0f,
      .y = 0.0f,
      .width = static_cast<float>(fbo.size.width),
      .height = static_cast<float>(fbo.size.height),
      .minDepth = 0.0f,
      .maxDepth = 1.0f,
  };
  vk::Rect2D scissor{
      .offset = {0, 0},
      .extent = fbo.size,
  };
  ds_.cmd.setViewport(0, viewport);
  ds_.cmd.setScissor(0, scissor);

  vk::RenderPassBeginInfo rp_info{
      .renderPass = *fbo.rp,
      .framebuffer = *fbo.fbs[fb_ind],
      .renderArea = {
          .offset = {0, 0},
          .extent = fbo.size,
      }};
  rp_info.setClearValues(fbo.clears);
  ds_.cmd.beginRenderPass(rp_info, vk::SubpassContents::eInline);
}

void Renderer::renderDrawing() {
  beginRp(drawing_.pass.fbo, 0);
  ds_.cmd.bindPipeline(
      vk::PipelineBindPoint::eGraphics, *drawing_.draw->pipeline);
  ds_.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *drawing_.draw->layout, 0,
      drawing_.inputs->sets[ds_.frame], nullptr);

  ds_.cmd.draw(3, 1, 0, 0);
  ds_.cmd.endRenderPass();
}

template <class T>
void updateDynamicBuf(
    vk::CommandBuffer cmd, DynamicBuf& dbuf, std::span<T> data,
    vk::PipelineStageFlags dst_stage, vk::AccessFlags dst_access) {
  size_t size = std::min((size_t)dbuf.staging.info.range, data.size_bytes());
  memcpy(dbuf.staging.mapped, data.data(), size);

  vk::BufferCopy copy{
      .srcOffset = dbuf.staging.info.offset,
      .dstOffset = dbuf.device.info.offset,
      .size = size,
  };
  cmd.copyBuffer(*dbuf.staging.buf, *dbuf.device.buf, copy);

  vk::BufferMemoryBarrier barrier{
      .srcAccessMask = vk::AccessFlagBits::eTransferWrite,
      .dstAccessMask = dst_access,
      .buffer = *dbuf.device.buf,
      .offset = dbuf.device.info.offset,
      .size = size,
  };
  cmd.pipelineBarrier(
      vk::PipelineStageFlagBits::eTransfer, dst_stage, {}, nullptr, barrier,
      nullptr);
}

void Renderer::renderVoronoi() {
  std::span<Vertex2d> span = frame_state_->voronoi_cells;
  updateDynamicBuf(
      ds_.cmd, voronoi_.verts[ds_.frame], span,
      vk::PipelineStageFlagBits::eVertexInput,
      vk::AccessFlagBits::eVertexAttributeRead);

  beginRp(voronoi_.pass.fbo, 0);

  ds_.cmd.bindPipeline(
      vk::PipelineBindPoint::eGraphics, *voronoi_.draw->pipeline);
  ds_.cmd.bindVertexBuffers(0, *voronoi_.verts[ds_.frame].device.buf, {0});
  ds_.cmd.draw(6, frame_state_->voronoi_cells.size(), 0, 0);

  ds_.cmd.endRenderPass();
}

void Renderer::renderScene() {
  beginRp(scene_.pass.fbo, 0);

  auto& draw = *scene_.draw;

  ds_.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *draw.pipeline);
  ds_.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *draw.layout, 0,
      scene_.global->sets[ds_.frame], nullptr);

  // TODO: Sort by material, then by model.
  std::sort(
      frame_state_->objects.begin(), frame_state_->objects.end(),
      [](auto& left, auto& right) { return left.model < right.model; });

  ModelId curr_model_id = ModelId::None;
  Material* curr_material = nullptr;
  for (auto& obj : frame_state_->objects) {
    auto it = loaded_models_.find(obj.model);
    ASSERT(it != loaded_models_.end());
    auto* model = it->second.get();

    if (curr_material != model->material) {
      curr_material = model->material;
      ds_.cmd.bindDescriptorSets(
          vk::PipelineBindPoint::eGraphics, *draw.layout, 1,
          model->material->desc_set, nullptr);
    }

    if (curr_model_id != obj.model) {
      curr_model_id = obj.model;
      vk::DeviceSize offsets[] = {0};
      ds_.cmd.bindVertexBuffers(0, *model->vert_buf, offsets);
      if (model->index_count) {
        ds_.cmd.bindIndexBuffer(*model->ind_buf, 0, vk::IndexType::eUint32);
      }
    }

    PushData push_data{obj.transform};
    ds_.cmd.pushConstants<PushData>(
        *draw.layout, vk::ShaderStageFlagBits::eVertex, 0, push_data);

    if (model->index_count) {
      ds_.cmd.drawIndexed(model->index_count, 1, 0, 0, 0);
    } else {
      ds_.cmd.draw(model->vertex_count, 1, 0, 0);
    }
  }

  ds_.cmd.endRenderPass();
}

void Renderer::renderPost() {
  beginRp(post_.pass.fbo, 0);

  ds_.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *post_.draw->pipeline);
  ds_.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *post_.draw->layout, 0,
      post_.inputs->sets[ds_.frame], nullptr);
  ds_.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *post_.draw->layout, 1,
      scene_.pass.fbo.output_set.sets[ds_.frame], nullptr);
  ds_.cmd.draw(3, 1, 0, 0);

  ds_.cmd.endRenderPass();
}

void Renderer::renderSwap() {
  beginRp(swap_.pass.fbo, ds_.img_ind);

  ds_.cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *swap_.draw->pipeline);
  ds_.cmd.bindDescriptorSets(
      vk::PipelineBindPoint::eGraphics, *swap_.draw->layout, 0,
      post_.pass.fbo.output_set.sets[ds_.frame], nullptr);
  ds_.cmd.draw(3, 1, 0, 0);

  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), ds_.cmd);

  ds_.cmd.endRenderPass();
}

void Renderer::recordCommandBuffer() {
  if (frame_state_->update_voronoi) {
    renderVoronoi();
  }
  if (frame_state_->update_drawing) {
    renderDrawing();
  }
  renderScene();
  renderPost();
  renderSwap();

  std::ignore = ds_.cmd.end();
}

void Renderer::createSyncObjects() {
  vk::SemaphoreCreateInfo sem_ci{};
  vk::FenceCreateInfo fence_ci{.flags = vk::FenceCreateFlagBits::eSignaled};

  img_sems_.resize(MAX_FRAMES_IN_FLIGHT);
  render_sems_.resize(MAX_FRAMES_IN_FLIGHT);
  in_flight_fences_.resize(MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
    img_sems_[i] = device_->createSemaphoreUnique(sem_ci).value;
    render_sems_[i] = device_->createSemaphoreUnique(sem_ci).value;
    in_flight_fences_[i] = device_->createFenceUnique(fence_ci).value;
  }
}

void Renderer::recreateSwapchain() {
  std::ignore = device_->waitIdle();

  swapchain_views_.clear();
  swapchain_.reset();

  swapchain_support_ = querySwapchainSupport(physical_device_);
  createSwapchain();

  scene_.pass.fbo.resize(vs_, swapchain_extent_);
  post_.pass.fbo.resize(vs_, swapchain_extent_);

  swap_.pass.fbo.swap_format = swapchain_format_;
  swap_.pass.fbo.swap_views = views(swapchain_views_);
  swap_.pass.fbo.resize(vs_, swapchain_extent_);

  window_resized_ = false;
}
