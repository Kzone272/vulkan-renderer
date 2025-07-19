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
#include <print>
#include <set>
#include <span>
#include <vector>

#include "asserts.h"
#include "command-buffers.h"
#include "defines.h"
#include "descriptors.h"
#include "fbo.h"
#include "files.h"
#include "glm-include.h"
#include "hash.h"
#include "images.h"
#include "pipelines.h"
#include "render-objects.h"
#include "render-state.h"
#include "vulkan-include.h"

namespace {

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
  std::println("dbg_layer: {}", callback_data->pMessage);
  return VK_FALSE;
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

Renderer::~Renderer() {
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
  return mats_.loadMaterial(vs_, mat_info);
}

void Renderer::updateMaterial(MaterialId id, const MaterialData& data) {
  return mats_.updateMaterial(id, data);
}

void Renderer::useMesh(ModelId model_id, const Mesh& mesh) {
  auto model = loadMesh(mesh);
  loaded_models_[model_id] = std::move(model);
}

TextureId Renderer::getDrawingTexture() {
  return mats_.getTextureId(drawing_.pass.fbo.colors[0].get());
}

TextureId Renderer::getVoronoiTexture() {
  return mats_.getTextureId(voronoi_.pass.fbo.colors[0].get());
}

void Renderer::initVulkan() {
  createInstance();
  if (enable_validation_layers_) {
    setupDebugMessenger();
  }
  createSurface();
  pickPhysicalDevice();
  createLogicalDevice();
  createVma();
  createCommandPool();
  createCommandBuffers();
  createSwapchain();
  createSamplers();
  createDescriptorPool();
  createImguiDescriptorPool();
  createSyncObjects();
  createShaders();
  findDepthFormat();
  // VulkanState should now be fully defined.

  mats_.init(vs_);
  sample_query_.init(vs_, scene_samples_);
  drawing_.init(vs_);
  voronoi_.init(vs_);
  scene_.init(vs_, scene_samples_, &mats_);
  edges_.init(
      vs_, scene_.outputSet(), scene_uses_msaa_, sample_query_.outputSet(),
      {&scene_.global_buf.device.info});
  jf_.init(vs_);
  swap_.init(vs_);
  resolve_.init(vs_);
}

void Renderer::initImgui() {
  ImGui_ImplVulkan_InitInfo init_info{
      .Instance = *instance_,
      .PhysicalDevice = physical_device_,
      .Device = *device_,
      .Queue = gfx_q_,
      .DescriptorPool = *imgui_desc_pool_,
      .MinImageCount = vs_.kMaxFramesInFlight,
      .ImageCount = vs_.kMaxFramesInFlight,
      .MSAASamples = (VkSampleCountFlagBits)swap_.pass.fbo.samples,
  };
  ImGui_ImplVulkan_Init(&init_info, *swap_.pass.fbo.rp);

  auto cmd_buf = beginSingleTimeCommands(vs_);
  ImGui_ImplVulkan_CreateFontsTexture(cmd_buf);
  endSingleTimeCommands(vs_, cmd_buf);
  ImGui_ImplVulkan_DestroyFontUploadObjects();
}

void Renderer::imguiNewFrame() {
  ImGui_ImplVulkan_NewFrame();
}

void Renderer::drawFrame() {
  if (window_resized_) {
    recreateSwapchain();
  }

  ds_.frame = frame_state_->frame_num % vs_.kMaxFramesInFlight;
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
  if (frame_state_->update_voronoi) {
    voronoi_.update(ds_, frame_state_->voronoi_cells);
  }
  mats_.update(ds_);
  scene_.update(vs_, ds_, *frame_state_);
  edges_.update(ds_, frame_state_->edges);

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
      .apiVersion = vulkan_version_,
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
  std::println("Required instance extensions ({}):", ext_names.size());
  for (auto& name : ext_names) {
    std::println("  {}", name);
  }
#endif

  return ext_names;
}

void Renderer::printSupportedExtensions() {
  std::vector<vk::ExtensionProperties> sup_exts =
      vk::enumerateInstanceExtensionProperties().value;
  std::println("Supported instance extensions ({})", sup_exts.size());
  for (auto& ext : sup_exts) {
    std::println("  {} v{}", ext.extensionName.data(), ext.specVersion);
  }
}

std::vector<const char*> Renderer::getValidationLayers() {
  if (!enable_validation_layers_) {
    return {};
  }

  std::vector<vk::LayerProperties> layer_props =
      vk::enumerateInstanceLayerProperties().value;

  std::println("Available layers ({}):", layer_props.size());
  for (const auto& layer_prop : layer_props) {
    std::println("  {}", layer_prop.layerName.data());
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
      std::println("Missing required validation layer: {}", layer);
      ASSERT(false);
    }
  }

  std::println("Required layers ({}):", validation_layers.size());
  for (const auto& layer : validation_layers) {
    std::println("  {}", layer);
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
    // Required to query subsample positions.
    if (!features.sampleRateShading) {
      continue;
    }

    physical_device_ = device;
    q_indices_ = indices;
    swapchain_support_ = swapchain_support;
    break;
  }
  ASSERT(physical_device_);
  vs_.physical_device = physical_device_;
  vs_.device_props = physical_device_.getProperties();
  vs_.mem_props = physical_device_.getMemoryProperties();
  max_samples_ = getMaxSampleCount();
  std::println("max msaa samples: {}", (int)max_samples_);

#ifdef DEBUG
  std::println("Supported formats ({})", swapchain_support_.formats.size());
  for (const auto& format : swapchain_support_.formats) {
    std::print("  {}", (int)format.format);
  }
  std::println("");
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

  vk::PhysicalDeviceFeatures device_features{
      .sampleRateShading = VK_TRUE,
      .samplerAnisotropy = VK_TRUE,
  };

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
  vs_.gfx_q = gfx_q_;

  present_q_ = device_->getQueue(q_indices_.present_family, 0);
  ASSERT(present_q_);
}

void Renderer::createVma() {
  vma::AllocatorCreateInfo vma_ci = {
      .physicalDevice = physical_device_,
      .device = vs_.device,
      .instance = *instance_,
      .vulkanApiVersion = vulkan_version_,
  };
  vma_ = vma::createAllocatorUnique(vma_ci).value;
  vs_.vma = *vma_;
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
  vs_.swap_format = format.format;
  auto present_mode = chooseSwapPresentMode(swapchain_support_.present_modes);
  vs_.swap_size = chooseSwapExtent(swapchain_support_.caps);

  uint32_t image_count = swapchain_support_.caps.minImageCount + 1;
  if (swapchain_support_.caps.maxImageCount > 0) {
    image_count = std::min(image_count, swapchain_support_.caps.maxImageCount);
  }

  vk::SwapchainCreateInfoKHR swapchain_ci{
      .surface = *surface_,
      .minImageCount = image_count,
      .imageFormat = vs_.swap_format,
      .imageColorSpace = format.colorSpace,
      .imageExtent = vs_.swap_size,
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
        vs_, swapchain_images[i], vs_.swap_format, 1,
        vk::ImageAspectFlagBits::eColor);
  }
  vs_.swap_views = views(swapchain_views_);

  std::println(
      "Created {} swapchain images, format:{} extent:{}x{}", image_count,
      (int)vs_.swap_format, vs_.swap_size.width, vs_.swap_size.height);
}

void Renderer::createShaders() {
  vs_.shaders.create(vs_);
}

vk::UniqueShaderModule Renderer::createShaderModule(std::string filename) {
  auto code = readFile(filename);
  vk::ShaderModuleCreateInfo ci{
      .codeSize = code.size(),
      .pCode = reinterpret_cast<const uint32_t*>(code.data()),
  };
  return device_->createShaderModuleUnique(ci).value;
}

void Renderer::createCommandPool() {
  vk::CommandPoolCreateInfo ci{
      .flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
      .queueFamilyIndex = q_indices_.gfx_family,
  };
  cmd_pool_ = device_->createCommandPoolUnique(ci).value;
  vs_.cmd_pool = *cmd_pool_;
}

void Renderer::findDepthFormat() {
  vs_.depth_format = findSupportedFormat(
      {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint,
       vk::Format::eD24UnormS8Uint},
      vk::ImageTiling::eOptimal,
      vk::FormatFeatureFlagBits::eDepthStencilAttachment);
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

  std::println("Failed to find supported format!");
  ASSERT(false);
  return vk::Format::eUndefined;
}

void Renderer::initSdlImage() {
  if (!IMG_Init(IMG_INIT_JPG)) {
    std::println("{}", IMG_GetError());
    ASSERT(false);
  }
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
  vs_.linear_sampler = *linear_sampler_;

  auto clamp_ci = ci;
  clamp_ci.addressModeU = vk::SamplerAddressMode::eClampToEdge;
  clamp_ci.addressModeV = vk::SamplerAddressMode::eClampToEdge;
  clamp_ci.addressModeW = vk::SamplerAddressMode::eClampToEdge;
  clamp_sampler_ = device_->createSamplerUnique(clamp_ci).value;
  vs_.clamp_sampler = *clamp_sampler_;

  auto nearest_ci = ci;
  nearest_ci.magFilter = vk::Filter::eNearest;
  nearest_ci.minFilter = vk::Filter::eNearest;
  nearest_ci.mipmapMode = vk::SamplerMipmapMode::eNearest,
  nearest_ci.anisotropyEnable = VK_FALSE;
  nearest_sampler_ = device_->createSamplerUnique(nearest_ci).value;
  vs_.nearest_sampler = *nearest_sampler_;
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

void Renderer::stageVertices(
    const std::vector<Vertex>& vertices, Model& model) {
  vk::DeviceSize size = sizeof(Vertex) * vertices.size();
  stageBuffer(
      size, (void*)vertices.data(), vk::BufferUsageFlagBits::eVertexBuffer,
      model.vert_buf);
}

void Renderer::stageIndices(
    const std::vector<uint32_t>& indices, Model& model) {
  vk::DeviceSize size = sizeof(uint32_t) * indices.size();
  stageBuffer(
      size, (void*)indices.data(), vk::BufferUsageFlagBits::eIndexBuffer,
      model.ind_buf);
}

// Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
// from the staging_buf to dst_buf.
// TODO: Move to buffers.h?
void Renderer::stageBuffer(
    vk::DeviceSize size, void* data, vk::BufferUsageFlags usage,
    Buffer& dst_buf) {
  auto staging_buf = createStagingBuffer(vs_, size);
  dst_buf = createDeviceBuffer(
      vs_, size, usage | vk::BufferUsageFlagBits::eTransferDst);

  vma_->copyMemoryToAllocation(data, *staging_buf.alloc, 0, size);

  vk::CommandBuffer cmd_buf = beginSingleTimeCommands(vs_);

  copyBufferToBuffer(
      cmd_buf, staging_buf, dst_buf, size,
      vk::PipelineStageFlagBits::eTopOfPipe, {});

  endSingleTimeCommands(vs_, cmd_buf);
}

void Renderer::createDescriptorPool() {
  // Arbitrary. 100 seems fine for now.
  const uint32_t kMaxSamplers = 100;
  const uint32_t kMaxUbos = 100;
  const uint32_t kMaxStorageBufs = 100;

  std::vector<vk::DescriptorPoolSize> pool_sizes{
      {.type = vk::DescriptorType::eUniformBuffer, .descriptorCount = kMaxUbos},
      {.type = vk::DescriptorType::eStorageBuffer,
       .descriptorCount = kMaxStorageBufs},
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
      .commandBufferCount = vs_.kMaxFramesInFlight,
  };
  cmd_bufs_ = device_->allocateCommandBuffersUnique(alloc_info).value;
}

void Renderer::recordCommandBuffer() {
  if (frame_state_->frame_num == 0) {
    sample_query_.render(ds_);
  }

  if (frame_state_->update_voronoi) {
    voronoi_.render(ds_);
  }
  if (frame_state_->update_drawing) {
    drawing_.render(ds_);
  }
  scene_.render(ds_, loaded_models_);

  if (frame_state_->stained_glass) {
    edges_.render(ds_, scene_.outputSet()->sets[1]);
    float max_screen = std::max(vs_.swap_size.width, vs_.swap_size.height);
    jf_.render(ds_, max_screen, edges_.outputSet()->sets[0]);
    jf_.initVoronoi(ds_, frame_state_->v_tweak, jf_.lastOutputSet());
    jf_.render(ds_, max_screen, jf_.lastOutputSet());
  } else if (frame_state_->draw_edges) {
    edges_.render(ds_, scene_.outputSet()->sets[1]);
    jf_.render(ds_, frame_state_->edge_w, edges_.outputSet()->sets[0]);
  }

  if (scene_uses_msaa_ && frame_state_->debug_view == DebugView::None) {
    resolve_.render(ds_, scene_.outputSet()->sets[0]);
  }

  {
    swap_.startRender(ds_);

    auto scene_output = scene_uses_msaa_ ? resolve_.outputSet()->sets[0]
                                         : scene_.outputSet()->sets[0];

    if (frame_state_->debug_view == DebugView::None) {
      if (frame_state_->stained_glass) {
        swap_.drawUvSample(ds_, jf_.lastOutputSet(), scene_output);
      } else {
        swap_.drawImage(ds_, scene_output);
      }
    } else if (frame_state_->debug_view == DebugView::Normals) {
      swap_.drawNormals(ds_, scene_.outputSet()->sets[1]);
    } else if (frame_state_->debug_view == DebugView::Depth) {
      swap_.drawDepth(
          ds_, scene_.outputSet()->sets[1], glm::inverse(frame_state_->proj));
    }

    if (frame_state_->draw_edges && !frame_state_->stained_glass) {
      swap_.drawJfSdf(
          ds_, frame_state_->edge_w, frame_state_->edges.i1.i,
          jf_.lastOutputSet());
    }

    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), ds_.cmd);
    ds_.cmd.endRenderPass();
  }

  std::ignore = ds_.cmd.end();
}

void Renderer::createSyncObjects() {
  vk::SemaphoreCreateInfo sem_ci{};
  vk::FenceCreateInfo fence_ci{.flags = vk::FenceCreateFlagBits::eSignaled};

  img_sems_.resize(vs_.kMaxFramesInFlight);
  render_sems_.resize(vs_.kMaxFramesInFlight);
  in_flight_fences_.resize(vs_.kMaxFramesInFlight);
  for (size_t i = 0; i < vs_.kMaxFramesInFlight; i++) {
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

  scene_.resize(vs_);
  edges_.resize(vs_);
  jf_.resize(vs_);
  resolve_.resize(vs_);
  swap_.resize(vs_);

  window_resized_ = false;
}
