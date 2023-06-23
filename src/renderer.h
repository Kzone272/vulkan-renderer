#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>
#include <SDL_vulkan.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>

#include "asserts.h"
#include "defines.h"
#include "frame-state.h"
#include "glm-include.h"
#include "vulkan-include.h"

using std::cerr;
using std::cout;
using std::endl;
using std::printf;

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Time = std::chrono::time_point<Clock>;

namespace {

constexpr int MAX_FRAMES_IN_FLIGHT = 2;

const std::string MODEL_PATH = "assets/models/viking_room.obj";
const std::string TEXTURE_PATH = "assets/textures/viking_room.png";

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

static std::vector<char> readFile(const std::string& filename) {
  std::ifstream file(filename, std::ios::ate | std::ios::binary);
  ASSERT(file.is_open());
  size_t file_size = static_cast<size_t>(file.tellg());
  std::vector<char> buffer(file_size);
  file.seekg(0);
  file.read(buffer.data(), file_size);
  file.close();
  return buffer;
}

}  // namespace

using glm::mat4;
using glm::vec2;
using glm::vec3;

struct Vertex {
  vec3 pos;
  vec3 color;
  vec2 uv;

  bool operator==(const Vertex& other) const {
    return pos == other.pos && color == other.color && uv == other.uv;
  }

  static vk::VertexInputBindingDescription getBindingDesc() {
    return {
        .binding = 0,
        .stride = sizeof(Vertex),
        .inputRate = vk::VertexInputRate::eVertex,
    };
  }

  static std::array<vk::VertexInputAttributeDescription, 3> getAttrDescs() {
    std::array<vk::VertexInputAttributeDescription, 3> attrs{};
    attrs[0] = {
        .location = 0,
        .binding = 0,
        .format = vk::Format::eR32G32B32Sfloat,  // vec3
        .offset = offsetof(Vertex, pos),
    };
    attrs[1] = {
        .location = 1,
        .binding = 0,
        .format = vk::Format::eR32G32B32Sfloat,  // vec3
        .offset = offsetof(Vertex, color),
    };
    attrs[2] = {
        .location = 2,
        .binding = 0,
        .format = vk::Format::eR32G32Sfloat,  // vec2
        .offset = offsetof(Vertex, uv),
    };

    return attrs;
  }
};

namespace std {

template <>
struct hash<Vertex> {
  size_t operator()(Vertex const& vertex) const {
    return ((hash<glm::vec3>()(vertex.pos) ^
             (hash<glm::vec3>()(vertex.color) << 1)) >>
            1) ^
           (hash<glm::vec2>()(vertex.uv) << 1);
  }
};

}  // namespace std

struct Geometry {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
};

class Renderer {
 public:
  Renderer(SDL_Window* window, uint32_t width, uint32_t height) {
    window_ = window;
    width_ = width;
    height_ = height;
  }

  void init(FrameState* frame_state) {
    frame_state_ = frame_state;
    initVulkan();
  }

  void drawFrame(FrameState* frame_state) {
    frame_state_ = frame_state;
    drawFrame();
  }

  void cleanup() {
    std::ignore = device_->waitIdle();
  }

  void resizeWindow(uint32_t width, uint32_t height) {
    if (width_ == width && height_ == height) {
      return;
    }
    window_resized_ = true;
    width_ = width;
    height_ = height;
  }

 private:
  void initVulkan() {
    createInstance();
    if (enable_validation_layers_) {
      setupDebugMessenger();
    }
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapchain();
    createImageViews();
    createRenderPass();
    createDescriptorSetLayout();
    createGraphicsPipeline();
    createCommandPool();
    createColorResources();
    createDepthResources();
    createFrameBuffers();
    createTextureImage();
    createTextureImageView();
    createTextureSampler();
    loadModel();
    createVertexBuffer();
    createIndexBuffer();
    createUniformBuffers();
    createDescriptorPool();
    createDescriptorSets();
    createCommandBuffers();
    createSyncObjects();
  }

  struct AnimationState {
    float clear_val = 0.0f;
    float model_rot = 0.0f;
  };

  void updateUniformBuffer() {
    auto& buf = uniform_bufs_[frame_state_->frame_num % MAX_FRAMES_IN_FLIGHT];
    auto& ubo = frame_state_->ubo;
    memcpy(buf.buf_mapped, &ubo, sizeof(ubo));
  }

  void drawFrame() {
    if (window_resized_) {
      recreateSwapchain();
    }

    int frame = frame_state_->frame_num % MAX_FRAMES_IN_FLIGHT;

    std::ignore =
        device_->waitForFences(*in_flight_fences_[frame], VK_TRUE, UINT64_MAX);

    auto [result, img_ind] = device_->acquireNextImageKHR(
        *swapchain_, UINT64_MAX, *img_sems_[frame], {});
    if (result == vk::Result::eErrorOutOfDateKHR) {
      window_resized_ = true;
      return;
    }
    ASSERT(
        result == vk::Result::eSuccess || result == vk::Result::eSuboptimalKHR);
    ASSERT(img_ind >= 0);
    ASSERT(img_ind < swapchain_images_.size());

    // Only reset the fence if we're submitting work.
    device_->resetFences(*in_flight_fences_[frame]);

    updateUniformBuffer();

    cmd_bufs_[frame]->reset();
    recordCommandBuffer(*cmd_bufs_[frame], img_ind);

    vk::SubmitInfo submit_info{};
    vk::PipelineStageFlags wait_stages =
        vk::PipelineStageFlagBits::eColorAttachmentOutput;
    submit_info.setWaitDstStageMask(wait_stages);
    submit_info.setWaitSemaphores(*img_sems_[frame]);
    submit_info.setCommandBuffers(*cmd_bufs_[frame]);
    submit_info.setSignalSemaphores(*render_sems_[frame]);
    std::ignore = gfx_q_.submit(submit_info, *in_flight_fences_[frame]);

    vk::PresentInfoKHR present_info{};
    present_info.setWaitSemaphores(*render_sems_[frame]);
    present_info.setSwapchains(*swapchain_);
    present_info.setImageIndices(img_ind);

    // For some reason Vulkan.hpp asserts on eErrorOutOfDateKHR, so until that's
    // dealt with, I need to use the C interface for present.
    result = (vk::Result)vkQueuePresentKHR(
        present_q_, (VkPresentInfoKHR*)&present_info);
    // result = present_q_.presentKHR(present_info); // This crashes.
    if (result == vk::Result::eErrorOutOfDateKHR ||
        result == vk::Result::eSuboptimalKHR) {
      window_resized_ = true;
      return;
    }
  }

  void createInstance() {
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

  std::vector<const char*> getRequiredExtensions() {
    uint32_t ext_count = 0;
    ASSERT(SDL_Vulkan_GetInstanceExtensions(nullptr, &ext_count, nullptr));
    std::vector<const char*> ext_names(ext_count);
    ASSERT(SDL_Vulkan_GetInstanceExtensions(
        nullptr, &ext_count, ext_names.data()));

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

  void printSupportedExtensions() {
    std::vector<vk::ExtensionProperties> sup_exts =
        vk::enumerateInstanceExtensionProperties().value;
    printf("Supported instance extensions (%zd)\n", sup_exts.size());
    for (auto& ext : sup_exts) {
      printf("  %s v%d\n", ext.extensionName.data(), ext.specVersion);
    }
  }

  std::vector<const char*> getValidationLayers() {
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

  vk::DebugUtilsMessengerCreateInfoEXT makeDbgMessengerCi() {
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

  void setupDebugMessenger() {
    auto ci = makeDbgMessengerCi();
    dbg_messenger_ =
        instance_->createDebugUtilsMessengerEXTUnique(ci, nullptr, dldi_).value;
  }

  void createSurface() {
    VkSurfaceKHR surface;
    ASSERT(SDL_Vulkan_CreateSurface(window_, *instance_, &surface));
    surface_ = vk::UniqueSurfaceKHR(surface, *instance_);
  }

  void pickPhysicalDevice() {
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
    device_props_ = physical_device_.getProperties();
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

  vk::SampleCountFlagBits getMaxSampleCount() {
    auto count_limit = device_props_.limits.framebufferColorSampleCounts &
                       device_props_.limits.framebufferDepthSampleCounts;

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

  struct QueueFamilyIndices {
    uint32_t gfx_family = -1;
    uint32_t present_family = -1;

    bool isComplete() {
      return gfx_family != -1 && present_family != -1;
    }
  };

  QueueFamilyIndices findQueueFamilies(vk::PhysicalDevice device) {
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

  bool checkDeviceExtensionSupport(vk::PhysicalDevice device) {
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

  struct SwapchainSupportDetails {
    vk::SurfaceCapabilitiesKHR caps;
    std::vector<vk::SurfaceFormatKHR> formats;
    std::vector<vk::PresentModeKHR> present_modes;
  };

  SwapchainSupportDetails querySwapchainSupport(vk::PhysicalDevice device) {
    SwapchainSupportDetails details;
    details.caps = device.getSurfaceCapabilitiesKHR(*surface_).value;
    details.formats = device.getSurfaceFormatsKHR(*surface_).value;
    details.present_modes = device.getSurfacePresentModesKHR(*surface_).value;

    return details;
  }

  void createLogicalDevice() {
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

    gfx_q_ = device_->getQueue(q_indices_.gfx_family, 0);
    ASSERT(gfx_q_);
    present_q_ = device_->getQueue(q_indices_.present_family, 0);
    ASSERT(present_q_);
  }

  vk::SurfaceFormatKHR chooseSwapSurfaceFormat(
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

  vk::PresentModeKHR chooseSwapPresentMode(
      const std::vector<vk::PresentModeKHR>& present_modes) {
    constexpr vk::PresentModeKHR preferred_mode = vk::PresentModeKHR::eMailbox;
    if (std::find(present_modes.begin(), present_modes.end(), preferred_mode) !=
        present_modes.end()) {
      return preferred_mode;
    }

    return vk::PresentModeKHR::eFifo;
  }

  vk::Extent2D chooseSwapExtent(const vk::SurfaceCapabilitiesKHR& caps) {
    if (caps.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
      return caps.currentExtent;
    } else {
      vk::Extent2D extent{width_, height_};
      extent.width = std::clamp(
          extent.width, caps.minImageExtent.width, caps.maxImageExtent.width);
      extent.height = std::clamp(
          extent.height, caps.minImageExtent.height,
          caps.maxImageExtent.height);
      return extent;
    }
  }

  void createSwapchain() {
    auto format = chooseSwapSurfaceFormat(swapchain_support_.formats);
    auto present_mode = chooseSwapPresentMode(swapchain_support_.present_modes);
    auto extent = chooseSwapExtent(swapchain_support_.caps);

    uint32_t image_count = swapchain_support_.caps.minImageCount + 1;
    if (swapchain_support_.caps.maxImageCount > 0) {
      image_count =
          std::min(image_count, swapchain_support_.caps.maxImageCount);
    }

    vk::SwapchainCreateInfoKHR swapchain_ci{
        .surface = *surface_,
        .minImageCount = image_count,
        .imageFormat = format.format,
        .imageColorSpace = format.colorSpace,
        .imageExtent = extent,
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
    swapchain_images_ = device_->getSwapchainImagesKHR(*swapchain_).value;
    swapchain_format_ = format.format;
    swapchain_extent_ = extent;
    printf(
        "Created %d swapchain images, format:%d extent:%dx%d\n", image_count,
        swapchain_format_, swapchain_extent_.width, swapchain_extent_.height);
  }

  void createImageViews() {
    swapchain_views_.resize(swapchain_images_.size());
    for (size_t i = 0; i < swapchain_images_.size(); i++) {
      swapchain_views_[i] = createImageView(
          swapchain_images_[i], swapchain_format_, 1,
          vk::ImageAspectFlagBits::eColor);
    }
  }

  vk::UniqueImageView createImageView(
      vk::Image img, vk::Format format, uint32_t mip_levels,
      vk::ImageAspectFlags aspect_flags) {
    vk::ImageViewCreateInfo ci{
        .image = img,
        .viewType = vk::ImageViewType::e2D,
        .format = format,
        .subresourceRange = {
            .aspectMask = aspect_flags,
            .baseMipLevel = 0,
            .levelCount = mip_levels,
            .baseArrayLayer = 0,
            .layerCount = 1,
        }};

    return device_->createImageViewUnique(ci).value;
  }

  void createRenderPass() {
    vk::AttachmentDescription color_att{
        .format = swapchain_format_,
        .samples = msaa_samples_,
        .loadOp = vk::AttachmentLoadOp::eClear,
        .storeOp = vk::AttachmentStoreOp::eStore,
        .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
        .initialLayout = vk::ImageLayout::eUndefined,
        .finalLayout = vk::ImageLayout::eColorAttachmentOptimal,
    };
    vk::AttachmentReference color_att_ref{
        .attachment = 0,
        .layout = vk::ImageLayout::eColorAttachmentOptimal,
    };

    depth_fmt_ = findDepthFormat();
    vk::AttachmentDescription depth_att{
        .format = depth_fmt_,
        .samples = msaa_samples_,
        .loadOp = vk::AttachmentLoadOp::eClear,
        .storeOp = vk::AttachmentStoreOp::eDontCare,
        .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
        .initialLayout = vk::ImageLayout::eUndefined,
        .finalLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal,
    };
    vk::AttachmentReference depth_att_ref{
        .attachment = 1,
        .layout = vk::ImageLayout::eDepthStencilAttachmentOptimal,
    };

    vk::AttachmentDescription color_resolve_att{
        .format = swapchain_format_,
        .samples = vk::SampleCountFlagBits::e1,
        .loadOp = vk::AttachmentLoadOp::eDontCare,
        .storeOp = vk::AttachmentStoreOp::eStore,
        .stencilLoadOp = vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp = vk::AttachmentStoreOp::eDontCare,
        .initialLayout = vk::ImageLayout::eUndefined,
        .finalLayout = vk::ImageLayout::ePresentSrcKHR,
    };
    vk::AttachmentReference color_resolve_att_ref{
        .attachment = 2,
        .layout = vk::ImageLayout::eColorAttachmentOptimal,
    };

    vk::SubpassDescription subpass{
        .pipelineBindPoint = vk::PipelineBindPoint::eGraphics,
        .colorAttachmentCount = 1,
        .pColorAttachments = &color_att_ref,
        .pResolveAttachments = &color_resolve_att_ref,
        .pDepthStencilAttachment = &depth_att_ref,
    };

    // Writing to the subpass color attachment depends on the swapchain
    // finishing its read of the color attachment.
    // We also need to read and write to the depth buffer.
    vk::SubpassDependency dep{
        .srcSubpass = VK_SUBPASS_EXTERNAL,
        .dstSubpass = 0,
        .srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput |
                        vk::PipelineStageFlagBits::eEarlyFragmentTests,
        .dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput |
                        vk::PipelineStageFlagBits::eEarlyFragmentTests,
        .srcAccessMask = {},
        .dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite |
                         vk::AccessFlagBits::eDepthStencilAttachmentWrite,

    };

    std::array<vk::AttachmentDescription, 3> atts = {
        color_att, depth_att, color_resolve_att};
    vk::RenderPassCreateInfo rp_ci{};
    rp_ci.setAttachments(atts);
    rp_ci.setSubpasses(subpass);
    rp_ci.setDependencies(dep);
    render_pass_ = device_->createRenderPassUnique(rp_ci).value;
  }

  void createDescriptorSetLayout() {
    vk::DescriptorSetLayoutBinding ubo_binding{
        .binding = 0,
        .descriptorType = vk::DescriptorType::eUniformBuffer,
        .descriptorCount = 1,
        .stageFlags = vk::ShaderStageFlagBits::eVertex,
    };

    vk::DescriptorSetLayoutBinding sampler_binding{
        .binding = 1,
        .descriptorType = vk::DescriptorType::eCombinedImageSampler,
        .descriptorCount = 1,
        .stageFlags = vk::ShaderStageFlagBits::eFragment,
    };

    std::array<vk::DescriptorSetLayoutBinding, 2> bindings = {
        ubo_binding, sampler_binding};

    vk::DescriptorSetLayoutCreateInfo layout_ci{};
    layout_ci.setBindings(bindings);
    desc_set_layout_ =
        device_->createDescriptorSetLayoutUnique(layout_ci).value;
  }

  void createGraphicsPipeline() {
    auto vert_shader_code = readFile("shaders/shader.vert.spv");
    auto frag_shader_code = readFile("shaders/shader.frag.spv");

    vk::UniqueShaderModule vert_shader = createShaderModule(vert_shader_code);
    vk::UniqueShaderModule frag_shader = createShaderModule(frag_shader_code);

    vk::PipelineShaderStageCreateInfo vert_shader_stage_ci{
        .stage = vk::ShaderStageFlagBits::eVertex,
        .module = *vert_shader,
        .pName = "main",
    };

    vk::PipelineShaderStageCreateInfo frag_shader_stage_ci{
        .stage = vk::ShaderStageFlagBits::eFragment,
        .module = *frag_shader,
        .pName = "main",
    };
    std::array<vk::PipelineShaderStageCreateInfo, 2> shader_stages = {
        vert_shader_stage_ci, frag_shader_stage_ci};

    std::array<vk::DynamicState, 2> dyn_states = {
        vk::DynamicState::eViewport,
        vk::DynamicState::eScissor,
    };
    vk::PipelineDynamicStateCreateInfo dyn_state{};
    dyn_state.setDynamicStates(dyn_states);

    auto binding = Vertex::getBindingDesc();
    auto attrs = Vertex::getAttrDescs();

    vk::PipelineVertexInputStateCreateInfo vert_in_info{};
    vert_in_info.setVertexBindingDescriptions(binding);
    vert_in_info.setVertexAttributeDescriptions(attrs);

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
        .cullMode = vk::CullModeFlagBits::eBack,
        .frontFace = vk::FrontFace::eClockwise,
        .depthBiasEnable = VK_FALSE,
        .lineWidth = 1.0f,
    };

    vk::PipelineMultisampleStateCreateInfo multisampling{
        .rasterizationSamples = msaa_samples_,
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

    vk::PipelineLayoutCreateInfo pipeline_layout_ci{};
    pipeline_layout_ci.setSetLayouts(*desc_set_layout_);
    pipeline_layout_ =
        device_->createPipelineLayoutUnique(pipeline_layout_ci).value;

    vk::GraphicsPipelineCreateInfo pipeline_ci{
        .pVertexInputState = &vert_in_info,
        .pInputAssemblyState = &input_assembly,
        .pViewportState = &viewport_state,
        .pRasterizationState = &rasterizer,
        .pMultisampleState = &multisampling,
        .pDepthStencilState = &depth_ci,
        .pColorBlendState = &color_blending,
        .pDynamicState = &dyn_state,
        .layout = *pipeline_layout_,
        .renderPass = *render_pass_,
        .subpass = 0,
    };
    pipeline_ci.setStages(shader_stages);
    gfx_pipeline_ =
        device_->createGraphicsPipelineUnique(nullptr, pipeline_ci).value;
  }

  vk::UniqueShaderModule createShaderModule(const std::vector<char>& code) {
    vk::ShaderModuleCreateInfo ci{
        .codeSize = code.size(),
        .pCode = reinterpret_cast<const uint32_t*>(code.data()),
    };
    return device_->createShaderModuleUnique(ci).value;
  }

  void createFrameBuffers() {
    swapchain_fbs_.resize(swapchain_views_.size());
    for (size_t i = 0; i < swapchain_views_.size(); i++) {
      std::array<vk::ImageView, 3> atts = {
          *color_img_view_, *depth_img_view_, *swapchain_views_[i]};

      vk::FramebufferCreateInfo fb_ci{
          .renderPass = *render_pass_,
          .width = swapchain_extent_.width,
          .height = swapchain_extent_.height,
          .layers = 1,
      };
      fb_ci.setAttachments(atts);
      swapchain_fbs_[i] = device_->createFramebufferUnique(fb_ci).value;
    }
  }

  void createCommandPool() {
    vk::CommandPoolCreateInfo ci{
        .flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
        .queueFamilyIndex = q_indices_.gfx_family,
    };
    cmd_pool_ = device_->createCommandPoolUnique(ci).value;
  }

  void createColorResources() {
    vk::Format color_fmt = swapchain_format_;
    createImage(
        swapchain_extent_.width, swapchain_extent_.height, color_fmt, 1,
        msaa_samples_, vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eTransientAttachment |
            vk::ImageUsageFlagBits::eColorAttachment,
        vk::MemoryPropertyFlagBits::eDeviceLocal, color_img_, color_img_mem_);
    color_img_view_ = createImageView(
        *color_img_, color_fmt, 1, vk::ImageAspectFlagBits::eColor);
  }

  void createDepthResources() {
    createImage(
        swapchain_extent_.width, swapchain_extent_.height, depth_fmt_, 1,
        msaa_samples_, vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eDepthStencilAttachment,
        vk::MemoryPropertyFlagBits::eDeviceLocal, depth_img_, depth_img_mem_);
    depth_img_view_ = createImageView(
        *depth_img_, depth_fmt_, 1, vk::ImageAspectFlagBits::eDepth);
    transitionImageLayout(
        *depth_img_, depth_fmt_, 1, vk::ImageLayout::eUndefined,
        vk::ImageLayout::eDepthStencilAttachmentOptimal);
  }

  vk::Format findDepthFormat() {
    return findSupportedFormat(
        {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint,
         vk::Format::eD24UnormS8Uint},
        vk::ImageTiling::eOptimal,
        vk::FormatFeatureFlagBits::eDepthStencilAttachment);
  }

  bool hasStencilComponent(vk::Format format) {
    return format == vk::Format::eD32SfloatS8Uint ||
           format == vk::Format::eD24UnormS8Uint;
  }

  vk::Format findSupportedFormat(
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

  void createTextureImage() {
    if (!IMG_Init(IMG_INIT_JPG)) {
      printf("%s", IMG_GetError());
      ASSERT(false);
    }
    SDL_Surface* texture = IMG_Load(TEXTURE_PATH.c_str());
    ASSERT(texture);
    ASSERT(texture->pixels);
    // Vulkan likes images to have alpha channels. The SDL byte order is also
    // defined opposite to vk::Format.
    SDL_PixelFormatEnum desired_fmt = SDL_PIXELFORMAT_ABGR8888;
    if (texture->format->format != desired_fmt) {
      printf(
          "converting image pixel format from %s to %s\n",
          SDL_GetPixelFormatName(texture->format->format),
          SDL_GetPixelFormatName(desired_fmt));
      auto* new_surface = SDL_ConvertSurfaceFormat(texture, desired_fmt, 0);
      SDL_FreeSurface(texture);
      texture = new_surface;
    }
    uint32_t width = texture->w;
    uint32_t height = texture->h;
    vk::DeviceSize image_size = width * height * 4;
    mip_levels_ = std::floor(std::log2(std::max(width, height))) + 1;

    vk::UniqueBuffer staging_buf;
    vk::UniqueDeviceMemory staging_buf_mem;
    createBuffer(
        image_size, vk::BufferUsageFlagBits::eTransferSrc,
        vk::MemoryPropertyFlagBits::eHostVisible |
            vk::MemoryPropertyFlagBits::eHostCoherent,
        staging_buf, staging_buf_mem);

    void* data = device_->mapMemory(*staging_buf_mem, 0, image_size).value;
    memcpy(data, texture->pixels, static_cast<size_t>(image_size));
    device_->unmapMemory(*staging_buf_mem);
    SDL_FreeSurface(texture);

    texture_fmt_ = vk::Format::eR8G8B8A8Srgb;
    createImage(
        width, height, texture_fmt_, mip_levels_, vk::SampleCountFlagBits::e1,
        vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eTransferSrc |
            vk::ImageUsageFlagBits::eTransferDst |
            vk::ImageUsageFlagBits::eSampled,
        vk::MemoryPropertyFlagBits::eDeviceLocal, texture_img_,
        texture_img_mem_);

    transitionImageLayout(
        *texture_img_, texture_fmt_, mip_levels_, vk::ImageLayout::eUndefined,
        vk::ImageLayout::eTransferDstOptimal);
    copyBufferToImage(*staging_buf, *texture_img_, width, height);
    generateMipmaps(*texture_img_, width, height, texture_fmt_, mip_levels_);
    // Transitioned to vk::ImageLayout::eShaderReadOnlyOptimal while generating
    // mipmaps.
  }

  void createImage(
      uint32_t width, uint32_t height, vk::Format format, uint32_t mip_levels,
      vk::SampleCountFlagBits num_samples, vk::ImageTiling tiling,
      vk::ImageUsageFlags usage, vk::MemoryPropertyFlags props,
      vk::UniqueImage& img, vk::UniqueDeviceMemory& img_mem) {
    vk::ImageCreateInfo img_ci{
        .imageType = vk::ImageType::e2D,
        .format = format,
        .extent =
            {
                .width = width,
                .height = height,
                .depth = 1,
            },
        .mipLevels = mip_levels,
        .arrayLayers = 1,
        .samples = num_samples,
        .tiling = tiling,
        .usage = usage,
        .sharingMode = vk::SharingMode::eExclusive,
        .initialLayout = vk::ImageLayout::eUndefined,
    };
    img = device_->createImageUnique(img_ci).value;

    vk::MemoryRequirements mem_reqs = device_->getImageMemoryRequirements(*img);

    vk::MemoryAllocateInfo alloc_info{
        .allocationSize = mem_reqs.size,
        .memoryTypeIndex = findMemoryType(mem_reqs.memoryTypeBits, props),
    };
    img_mem = device_->allocateMemoryUnique(alloc_info).value;

    std::ignore = device_->bindImageMemory(*img, *img_mem, 0);
  }

  void transitionImageLayout(
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
        barrier.subresourceRange.aspectMask |=
            vk::ImageAspectFlagBits::eStencil;
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
          "Unsupported layout transition! (%d -> %d)\n", old_layout,
          new_layout);
      ASSERT(false);
    }

    cmd_buf.pipelineBarrier(
        src_stage, dst_stage, {}, nullptr, nullptr, barrier);

    endSingleTimeCommands(cmd_buf);
  }

  void copyBufferToImage(
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

  void generateMipmaps(
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

  void createTextureImageView() {
    texture_img_view_ = createImageView(
        *texture_img_, texture_fmt_, mip_levels_,
        vk::ImageAspectFlagBits::eColor);
  }

  void createTextureSampler() {
    vk::SamplerCreateInfo ci{
        .magFilter = vk::Filter::eLinear,
        .minFilter = vk::Filter::eLinear,
        .mipmapMode = vk::SamplerMipmapMode::eLinear,
        .addressModeU = vk::SamplerAddressMode::eRepeat,
        .addressModeV = vk::SamplerAddressMode::eRepeat,
        .addressModeW = vk::SamplerAddressMode::eRepeat,
        .mipLodBias = 0.f,
        .anisotropyEnable = VK_TRUE,
        .maxAnisotropy = device_props_.limits.maxSamplerAnisotropy,
        .compareEnable = VK_FALSE,
        .compareOp = vk::CompareOp::eAlways,
        .minLod = 0.f,
        .maxLod = static_cast<float>(mip_levels_),
        .borderColor = vk::BorderColor::eIntOpaqueBlack,
        .unnormalizedCoordinates = VK_FALSE,
    };
    texture_sampler_ = device_->createSamplerUnique(ci).value;
  }

  void loadModel() {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    ASSERT(tinyobj::LoadObj(
        &attrib, &shapes, &materials, &warn, &err, MODEL_PATH.c_str()));

    std::unordered_map<Vertex, uint32_t> uniq_verts;
    for (const auto& shape : shapes) {
      for (const auto& index : shape.mesh.indices) {
        Vertex vert{};
        vert.pos = {
            attrib.vertices[3 * index.vertex_index],
            attrib.vertices[3 * index.vertex_index + 1],
            attrib.vertices[3 * index.vertex_index + 2],
        };
        vert.uv = {
            attrib.texcoords[2 * index.texcoord_index],
            1.f - attrib.texcoords[2 * index.texcoord_index + 1],  // Flip v
        };
        vert.color = {1.f, 1.f, 1.f};

        auto it = uniq_verts.find(vert);
        if (it == uniq_verts.end()) {
          it = uniq_verts
                   .insert({vert, static_cast<uint32_t>(uniq_verts.size())})
                   .first;
          geom.vertices.push_back(vert);
        }
        geom.indices.push_back(it->second);
      }
    }
    printf(
        "loaded %zd vertices, %zd indices\n", geom.vertices.size(),
        geom.indices.size());
  }

  void createVertexBuffer() {
    vk::DeviceSize size = sizeof(Vertex) * geom.vertices.size();
    stageBuffer(
        size, (void*)geom.vertices.data(),
        vk::BufferUsageFlagBits::eVertexBuffer, vert_buf_, vert_buf_mem_);
  }

  void createIndexBuffer() {
    vk::DeviceSize size = sizeof(uint32_t) * geom.indices.size();
    stageBuffer(
        size, (void*)geom.indices.data(), vk::BufferUsageFlagBits::eIndexBuffer,
        ind_buf_, ind_buf_mem_);
  }

  void createUniformBuffers() {
    vk::DeviceSize buf_size = sizeof(UniformBufferObject);
    uniform_bufs_.resize(MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
      createBuffer(
          buf_size, vk::BufferUsageFlagBits::eUniformBuffer,
          vk::MemoryPropertyFlagBits::eHostVisible |
              vk::MemoryPropertyFlagBits::eHostCoherent,
          uniform_bufs_[i].buf, uniform_bufs_[i].buf_mem);
      uniform_bufs_[i].buf_mapped =
          device_->mapMemory(*uniform_bufs_[i].buf_mem, 0, buf_size).value;
    }
  }

  // Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
  // from the staging_buf to dst_buf.
  void stageBuffer(
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
    copyBuffer(*staging_buf, *dst_buf, size);
  }

  void copyBuffer(vk::Buffer src_buf, vk::Buffer dst_buf, vk::DeviceSize size) {
    vk::CommandBuffer cmd_buf = beginSingleTimeCommands();

    vk::BufferCopy copy_region{
        .size = size,
    };
    cmd_buf.copyBuffer(src_buf, dst_buf, copy_region);

    endSingleTimeCommands(cmd_buf);
  }

  vk::CommandBuffer beginSingleTimeCommands() {
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

  void endSingleTimeCommands(vk::CommandBuffer cmd_buf) {
    std::ignore = cmd_buf.end();

    vk::SubmitInfo submit{};
    submit.setCommandBuffers(cmd_buf);
    std::ignore = gfx_q_.submit(submit);
    std::ignore = gfx_q_.waitIdle();

    device_->freeCommandBuffers(*cmd_pool_, cmd_buf);
  }

  void createBuffer(
      vk::DeviceSize size, vk::BufferUsageFlags usage,
      vk::MemoryPropertyFlags props, vk::UniqueBuffer& buf,
      vk::UniqueDeviceMemory& buf_mem) {
    vk::BufferCreateInfo buffer_ci{
        .size = size,
        .usage = usage,
        .sharingMode = vk::SharingMode::eExclusive,
    };
    buf = device_->createBufferUnique(buffer_ci).value;

    vk::MemoryRequirements mem_reqs =
        device_->getBufferMemoryRequirements(*buf);

    vk::MemoryAllocateInfo alloc_info{
        .allocationSize = mem_reqs.size,
        .memoryTypeIndex = findMemoryType(mem_reqs.memoryTypeBits, props),
    };
    buf_mem = device_->allocateMemoryUnique(alloc_info).value;
    std::ignore = device_->bindBufferMemory(*buf, *buf_mem, 0);
  }

  uint32_t findMemoryType(uint32_t type_filter, vk::MemoryPropertyFlags props) {
    vk::PhysicalDeviceMemoryProperties mem_props =
        physical_device_.getMemoryProperties();

    for (uint32_t i = 0; i < mem_props.memoryTypeCount; i++) {
      if (type_filter & (1 << i) &&
          (mem_props.memoryTypes[i].propertyFlags & props) == props) {
        return i;
      }
    }
    ASSERT(false);
    return 0;
  }

  void createDescriptorPool() {
    std::array<vk::DescriptorPoolSize, 2> pool_sizes{{
        {
            .type = vk::DescriptorType::eUniformBuffer,
            .descriptorCount = MAX_FRAMES_IN_FLIGHT,
        },
        {
            .type = vk::DescriptorType::eCombinedImageSampler,
            .descriptorCount = MAX_FRAMES_IN_FLIGHT,
        },
    }};

    vk::DescriptorPoolCreateInfo pool_ci{
        .maxSets = MAX_FRAMES_IN_FLIGHT,
    };
    pool_ci.setPoolSizes(pool_sizes);
    desc_pool_ = device_->createDescriptorPoolUnique(pool_ci).value;
  }

  void createDescriptorSets() {
    std::vector<vk::DescriptorSetLayout> layouts(
        MAX_FRAMES_IN_FLIGHT, *desc_set_layout_);
    vk::DescriptorSetAllocateInfo alloc_info{
        .descriptorPool = *desc_pool_,
    };
    alloc_info.setSetLayouts(layouts);

    std::vector<vk::DescriptorSet> desc_sets =
        device_->allocateDescriptorSets(alloc_info).value;
    ASSERT(desc_sets.size() == uniform_bufs_.size());

    for (int i = 0; i < uniform_bufs_.size(); i++) {
      auto& buf_state = uniform_bufs_[i];
      buf_state.desc_set = std::move(desc_sets[i]);

      std::array<vk::WriteDescriptorSet, 2> desc_writes{};

      vk::DescriptorBufferInfo buffer_info{
          .buffer = *buf_state.buf,
          .offset = 0,
          .range = sizeof(UniformBufferObject),
      };
      desc_writes[0] = {
          .dstSet = buf_state.desc_set,
          .dstBinding = 0,
          .dstArrayElement = 0,
          .descriptorType = vk::DescriptorType::eUniformBuffer,
      };
      desc_writes[0].setBufferInfo(buffer_info);

      vk::DescriptorImageInfo image_info{
          .sampler = *texture_sampler_,
          .imageView = *texture_img_view_,
          .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
      };
      desc_writes[1] = {
          .dstSet = buf_state.desc_set,
          .dstBinding = 1,
          .dstArrayElement = 0,
          .descriptorType = vk::DescriptorType::eCombinedImageSampler,
      };
      desc_writes[1].setImageInfo(image_info);

      device_->updateDescriptorSets(desc_writes, nullptr);
    }
  }

  void createCommandBuffers() {
    vk::CommandBufferAllocateInfo alloc_info{
        .commandPool = *cmd_pool_,
        .level = vk::CommandBufferLevel::ePrimary,
        .commandBufferCount = MAX_FRAMES_IN_FLIGHT,
    };
    cmd_bufs_ = device_->allocateCommandBuffersUnique(alloc_info).value;
  }

  void recordCommandBuffer(vk::CommandBuffer cmd_buf, uint32_t img_ind) {
    vk::CommandBufferBeginInfo begin_info{};
    std::ignore = cmd_buf.begin(begin_info);

    vk::RenderPassBeginInfo rp_info{
        .renderPass = *render_pass_,
        .framebuffer = *swapchain_fbs_[img_ind],
        .renderArea = {
            .offset = {0, 0},
            .extent = swapchain_extent_,
        }};
    // float val = anim_.clear_val;
    float val = 0.f;
    std::array<vk::ClearValue, 2> clear_values{};
    clear_values[0].color = {val, val, val, 1.0f};
    clear_values[1].depthStencil = {1.f, 0};
    rp_info.setClearValues(clear_values);

    cmd_buf.beginRenderPass(rp_info, vk::SubpassContents::eInline);
    cmd_buf.bindPipeline(vk::PipelineBindPoint::eGraphics, *gfx_pipeline_);

    vk::Viewport viewport{
        .x = 0.0f,
        .y = 0.0f,
        .width = static_cast<float>(swapchain_extent_.width),
        .height = static_cast<float>(swapchain_extent_.height),
        .minDepth = 0.0f,
        .maxDepth = 1.0f,
    };
    cmd_buf.setViewport(0, viewport);

    vk::Rect2D scissor{
        .offset = {0, 0},
        .extent = swapchain_extent_,
    };
    cmd_buf.setScissor(0, scissor);

    auto& buf_state =
        uniform_bufs_[frame_state_->frame_num % MAX_FRAMES_IN_FLIGHT];
    cmd_buf.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, *pipeline_layout_, 0,
        buf_state.desc_set, nullptr);

    vk::DeviceSize offsets[] = {0};
    cmd_buf.bindVertexBuffers(0, *vert_buf_, offsets);
    cmd_buf.bindIndexBuffer(*ind_buf_, 0, vk::IndexType::eUint32);

    cmd_buf.drawIndexed(geom.indices.size(), 1, 0, 0, 0);
    cmd_buf.endRenderPass();
    std::ignore = cmd_buf.end();
  }

  void createSyncObjects() {
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

  void recreateSwapchain() {
    std::ignore = device_->waitIdle();
    cleanupSwapchain();

    swapchain_support_ = querySwapchainSupport(physical_device_);
    createSwapchain();
    createImageViews();
    createColorResources();
    createDepthResources();
    createFrameBuffers();
    window_resized_ = false;
  }

  void cleanupSwapchain() {
    depth_img_view_.reset();
    depth_img_.reset();
    depth_img_mem_.reset();

    color_img_view_.reset();
    color_img_.reset();
    color_img_mem_.reset();

    swapchain_fbs_.clear();
    swapchain_views_.clear();
    swapchain_.reset();
  }

  // The window should only be used in createSurface().
  SDL_Window* window_ = nullptr;
  bool window_resized_ = false;
  uint32_t width_ = 100;
  uint32_t height_ = 100;

  vk::UniqueInstance instance_;
  vk::DispatchLoaderDynamic dldi_;
  vk::UniqueSurfaceKHR surface_;
  vk::UniqueHandle<vk::DebugUtilsMessengerEXT, vk::DispatchLoaderDynamic>
      dbg_messenger_;
  vk::PhysicalDevice physical_device_;
  vk::PhysicalDeviceProperties device_props_;
  // Indices of queue families for the selected |physical_device_|
  QueueFamilyIndices q_indices_;
  vk::UniqueDevice device_;
  vk::Queue gfx_q_;
  vk::Queue present_q_;
  SwapchainSupportDetails swapchain_support_;
  vk::UniqueSwapchainKHR swapchain_;
  std::vector<vk::Image> swapchain_images_;
  vk::Format swapchain_format_;
  vk::Extent2D swapchain_extent_;
  std::vector<vk::UniqueImageView> swapchain_views_;
  vk::UniqueRenderPass render_pass_;
  vk::UniqueDescriptorSetLayout desc_set_layout_;
  vk::UniqueDescriptorPool desc_pool_;
  vk::UniquePipelineLayout pipeline_layout_;
  vk::UniquePipeline gfx_pipeline_;
  std::vector<vk::UniqueFramebuffer> swapchain_fbs_;
  vk::UniqueCommandPool cmd_pool_;
  std::vector<vk::UniqueCommandBuffer> cmd_bufs_;
  std::vector<vk::UniqueSemaphore> img_sems_;
  std::vector<vk::UniqueSemaphore> render_sems_;
  std::vector<vk::UniqueFence> in_flight_fences_;
  vk::UniqueBuffer vert_buf_;
  vk::UniqueDeviceMemory vert_buf_mem_;
  vk::UniqueBuffer ind_buf_;
  vk::UniqueDeviceMemory ind_buf_mem_;
  vk::Format texture_fmt_;
  uint32_t mip_levels_;
  vk::UniqueImage texture_img_;
  vk::UniqueDeviceMemory texture_img_mem_;
  vk::UniqueImageView texture_img_view_;
  vk::UniqueSampler texture_sampler_;
  vk::UniqueImage color_img_;
  vk::UniqueDeviceMemory color_img_mem_;
  vk::UniqueImageView color_img_view_;
  vk::Format depth_fmt_;
  vk::UniqueImage depth_img_;
  vk::UniqueDeviceMemory depth_img_mem_;
  vk::UniqueImageView depth_img_view_;
  vk::SampleCountFlagBits msaa_samples_ = vk::SampleCountFlagBits::e1;

  Geometry geom;

  FrameState* frame_state_ = nullptr;

  struct UniformBufferState {
    vk::UniqueBuffer buf;
    vk::UniqueDeviceMemory buf_mem;
    void* buf_mapped;
    vk::DescriptorSet desc_set;
  };
  std::vector<UniformBufferState> uniform_bufs_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
