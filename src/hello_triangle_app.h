#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_image.h>
#include <SDL_vulkan.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>

#include "asserts.h"
#define GLM_FORCE_RADIANS
#define GLM_FORCE_LEFT_HANDED
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>
#include <iostream>
#include <set>
#include <vector>

#include "defines.h"

using std::cerr;
using std::cout;
using std::endl;
using std::printf;

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Time = std::chrono::time_point<Clock>;

namespace {

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;
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

struct UniformBufferObject {
  alignas(16) mat4 model;
  alignas(16) mat4 view;
  alignas(16) mat4 proj;
};

class HelloTriangleApp {
 public:
  void run() {
    initWindow();
    initVulkan();
    mainLoop();
    cleanup();
  }

  // Apparently this can be called on another thread by the OS. That could
  // potentiall cause problems in the future.
  static int SdlEventWatcher(void* data, SDL_Event* event) {
    if (event->type == SDL_WINDOWEVENT) {
      if (event->window.event == SDL_WINDOWEVENT_RESIZED) {
        reinterpret_cast<HelloTriangleApp*>(data)->WindowResized();
      } else if (event->window.event == SDL_WINDOWEVENT_MOVED) {
        reinterpret_cast<HelloTriangleApp*>(data)->WindowMoved();
      }
    }
    return 0;
  }

  void WindowResized() {
    window_resized_ = true;
    // This might not be correct, but we'll check the windows size again in
    // recreateSwapchain().
    empty_window_ = false;
    update();
  }

  void WindowMoved() {
    update();
  }

 private:
  void initWindow() {
    ASSERT(SDL_Init(SDL_INIT_VIDEO) == 0);

    uint32_t window_flags = SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE;
    window_ = SDL_CreateWindow(
        "Vulkan Tutorial", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, window_flags);
    if (!window_) {
      std::string error{SDL_GetError()};
      cerr << error << endl;
      ASSERT(false);
    }
    SDL_AddEventWatch(SdlEventWatcher, this);
  }

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

  void mainLoop() {
    while (true) {
      processEvents();
      if (quit_) {
        break;
      }
      update();
    }

    vkDeviceWaitIdle(device_);
  }

  void processEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_WINDOWEVENT) {
        if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
          quit_ = true;
          break;
        } else if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
          window_resized_ = true;
        } else if (event.window.event == SDL_WINDOWEVENT_MINIMIZED) {
          window_minimized_ = true;
        } else if (event.window.event == SDL_WINDOWEVENT_RESTORED) {
          window_minimized_ = false;
        }
      }
    }
  }

  void update() {
    timeTick();
    animate();
    if (!window_minimized_ && !empty_window_) {
      drawFrame();
      frame_num_++;
    }
  }

  using float_ms = std::chrono::duration<float, std::ratio<1, 1000>>;

  void timeTick() {
    Time now = Clock::now();
    if (frame_num_ == 0) {
      last_frame_time_ = now;
    }

    auto time_delta = now - last_frame_time_;
    time_delta_ms_ = float_ms(time_delta).count();
    last_frame_time_ = now;

    checkFps(now);
  }

  void checkFps(Time now) {
    if (frame_num_ == 0) {
      next_fps_time_ = now + 1s;
      return;
    }

    if (now > next_fps_time_) {
      int fps = frame_num_ - last_fps_frame_;
      printf("fps: %d\n", fps);

      next_fps_time_ = now + 1s;
      last_fps_frame_ = frame_num_;
    }
  }

  struct AnimationState {
    float clear_val = 0.0f;
    float model_rot = 0.0f;
  };

  void animate() {
    anim_.clear_val = updateClearValue();
    anim_.model_rot = updateModelRotation();
  }

  float updateClearValue() {
    constexpr float seq_dur_ms = 5000;
    static float seq = 0;
    seq += time_delta_ms_ / seq_dur_ms;
    while (seq > 1) {
      seq -= 1;
    }
    // return seq < 0.5f ? seq : (1.0f - seq); // linear bounce
    return cos(seq * 2.0f * M_PI) / 2.0f + 0.5f;
  }

  float updateModelRotation() {
    constexpr float seq_dur_ms = 4000;
    constexpr float total_rot = 360;
    static float seq = 0;
    seq += time_delta_ms_ / seq_dur_ms;
    while (seq > 1) {
      seq -= 1;
    }
    return seq * total_rot;
  }

  void updateUniformBuffer() {
    auto& buf = uniform_bufs_[frame_num_ % MAX_FRAMES_IN_FLIGHT];

    UniformBufferObject ubo;
    ubo.model =
        // spin model
        glm::rotate(mat4(1), glm::radians(anim_.model_rot), vec3(0, 1, 0)) *
        // reorient model
        glm::rotate(mat4(1), glm::radians(-90.f), vec3(1, 0, 0));
    ubo.view = glm::lookAt(vec3(0, 1.25, -2.5), vec3(0), vec3(0, 1, 0));
    ubo.proj = glm::perspective(
        glm::radians(45.0f),
        (float)swapchain_extent_.width / (float)swapchain_extent_.height, 0.1f,
        100.0f);
    // Invert y-axis because Vulkan is opposite GL.
    ubo.proj[1][1] *= -1;

    memcpy(buf.buf_mapped, &ubo, sizeof(ubo));
  }

  void drawFrame() {
    int frame = frame_num_ % MAX_FRAMES_IN_FLIGHT;

    vkWaitForFences(device_, 1, &in_flight_fences_[frame], VK_TRUE, UINT64_MAX);

    uint32_t img_ind = -1;
    VkResult result = vkAcquireNextImageKHR(
        device_, swapchain_, UINT64_MAX, img_sems_[frame], VK_NULL_HANDLE,
        &img_ind);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
      recreateSwapchain();
      return;
    }
    ASSERT(result == VK_SUCCESS || result == VK_SUBOPTIMAL_KHR);
    ASSERT(img_ind >= 0);
    ASSERT(img_ind < swapchain_images_.size());

    // Only reset the fence if we're submitting work.
    VKASSERT(vkResetFences(device_, 1, &in_flight_fences_[frame]));

    updateUniformBuffer();

    VKASSERT(vkResetCommandBuffer(cmd_bufs_[frame], 0));
    recordCommandBuffer(cmd_bufs_[frame], img_ind);

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    VkPipelineStageFlags wait_stages[] = {
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
    submit_info.waitSemaphoreCount = 1;
    submit_info.pWaitSemaphores = &img_sems_[frame];
    submit_info.pWaitDstStageMask = wait_stages;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &cmd_bufs_[frame];
    submit_info.signalSemaphoreCount = 1;
    submit_info.pSignalSemaphores = &render_sems_[frame];
    VKASSERT(vkQueueSubmit(gfx_q_, 1, &submit_info, in_flight_fences_[frame]));

    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = &render_sems_[frame];
    present_info.swapchainCount = 1;
    present_info.pSwapchains = (VkSwapchainKHR*)&swapchain_;
    present_info.pImageIndices = &img_ind;

    result = vkQueuePresentKHR(present_q_, &present_info);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR ||
        window_resized_) {
      window_resized_ = false;
      recreateSwapchain();
    } else {
      VKASSERT(result);
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

    instance_ = vk::createInstance(instance_ci).value;
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

#define LOAD_VK_FN(fn) (PFN_##fn) vkGetInstanceProcAddr(instance_, #fn);

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
    vk::DispatchLoaderDynamic dldi(instance_, vkGetInstanceProcAddr);
    dbg_messenger_ =
        instance_.createDebugUtilsMessengerEXT(ci, nullptr, dldi).value;
  }

  void createSurface() {
    VkSurfaceKHR surface;
    ASSERT(SDL_Vulkan_CreateSurface(window_, instance_, &surface));
    surface_ = surface;
  }

  void pickPhysicalDevice() {
    std::vector<vk::PhysicalDevice> devices =
        instance_.enumeratePhysicalDevices().value;
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
          device.getSurfaceSupportKHR(i, surface_).value;
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
    details.caps = device.getSurfaceCapabilitiesKHR(surface_).value;
    details.formats = device.getSurfaceFormatsKHR(surface_).value;
    details.present_modes = device.getSurfacePresentModesKHR(surface_).value;

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
    device_ = physical_device_.createDevice(device_ci).value;

    gfx_q_ = device_.getQueue(q_indices_.gfx_family, 0);
    ASSERT(gfx_q_);
    present_q_ = device_.getQueue(q_indices_.present_family, 0);
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
      int width = 0;
      int height = 0;
      SDL_GL_GetDrawableSize(window_, &width, &height);
      vk::Extent2D extent{
          static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
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
        .surface = surface_,
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
        .oldSwapchain = VK_NULL_HANDLE,
    };

    std::array<uint32_t, 2> queue_family_indices{
        q_indices_.gfx_family, q_indices_.present_family};
    if (q_indices_.gfx_family != q_indices_.present_family) {
      swapchain_ci.imageSharingMode = vk::SharingMode::eConcurrent;
      swapchain_ci.setQueueFamilyIndices(queue_family_indices);
    } else {
      swapchain_ci.imageSharingMode = vk::SharingMode::eExclusive;
    }

    swapchain_ = device_.createSwapchainKHR(swapchain_ci).value;
    swapchain_images_ = device_.getSwapchainImagesKHR(swapchain_).value;
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

  vk::ImageView createImageView(
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

    return device_.createImageView(ci).value;
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
    render_pass_ = device_.createRenderPass(rp_ci).value;
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
    desc_set_layout_ = device_.createDescriptorSetLayout(layout_ci).value;
  }

  void createGraphicsPipeline() {
    auto vert_shader_code = readFile("shaders/shader.vert.spv");
    auto frag_shader_code = readFile("shaders/shader.frag.spv");

    vk::ShaderModule vert_shader = createShaderModule(vert_shader_code);
    vk::ShaderModule frag_shader = createShaderModule(frag_shader_code);

    vk::PipelineShaderStageCreateInfo vert_shader_stage_ci{
        .stage = vk::ShaderStageFlagBits::eVertex,
        .module = vert_shader,
        .pName = "main",
    };

    vk::PipelineShaderStageCreateInfo frag_shader_stage_ci{
        .stage = vk::ShaderStageFlagBits::eFragment,
        .module = frag_shader,
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
    pipeline_layout_ci.setSetLayouts(desc_set_layout_);
    pipeline_layout_ = device_.createPipelineLayout(pipeline_layout_ci).value;

    vk::GraphicsPipelineCreateInfo pipeline_ci{
        .pVertexInputState = &vert_in_info,
        .pInputAssemblyState = &input_assembly,
        .pViewportState = &viewport_state,
        .pRasterizationState = &rasterizer,
        .pMultisampleState = &multisampling,
        .pDepthStencilState = &depth_ci,
        .pColorBlendState = &color_blending,
        .pDynamicState = &dyn_state,
        .layout = pipeline_layout_,
        .renderPass = render_pass_,
        .subpass = 0,
    };
    pipeline_ci.setStages(shader_stages);
    gfx_pipeline_ =
        device_.createGraphicsPipeline(VK_NULL_HANDLE, pipeline_ci).value;

    // Cleanup
    device_.destroyShaderModule(vert_shader);
    device_.destroyShaderModule(frag_shader);
  }

  vk::ShaderModule createShaderModule(const std::vector<char>& code) {
    vk::ShaderModuleCreateInfo ci{
        .codeSize = code.size(),
        .pCode = reinterpret_cast<const uint32_t*>(code.data()),
    };
    return device_.createShaderModule(ci).value;
  }

  void createFrameBuffers() {
    swapchain_fbs_.resize(swapchain_views_.size());
    for (size_t i = 0; i < swapchain_views_.size(); i++) {
      std::array<vk::ImageView, 3> atts = {
          color_img_view_, depth_img_view_, swapchain_views_[i]};

      vk::FramebufferCreateInfo fb_ci{
          .renderPass = render_pass_,
          .width = swapchain_extent_.width,
          .height = swapchain_extent_.height,
          .layers = 1,
      };
      fb_ci.setAttachments(atts);
      swapchain_fbs_[i] = device_.createFramebuffer(fb_ci).value;
    }
  }

  void createCommandPool() {
    vk::CommandPoolCreateInfo ci{
        .flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
        .queueFamilyIndex = q_indices_.gfx_family,
    };
    cmd_pool_ = device_.createCommandPool(ci).value;
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
        color_img_, color_fmt, 1, vk::ImageAspectFlagBits::eColor);
  }

  void createDepthResources() {
    createImage(
        swapchain_extent_.width, swapchain_extent_.height, depth_fmt_, 1,
        msaa_samples_, vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eDepthStencilAttachment,
        vk::MemoryPropertyFlagBits::eDeviceLocal, depth_img_, depth_img_mem_);
    depth_img_view_ = createImageView(
        depth_img_, depth_fmt_, 1, vk::ImageAspectFlagBits::eDepth);
    transitionImageLayout(
        depth_img_, depth_fmt_, 1, vk::ImageLayout::eUndefined,
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
    // defined opposite to VkFormat.
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

    vk::Buffer staging_buf;
    vk::DeviceMemory staging_buf_mem;
    createBuffer(
        image_size, vk::BufferUsageFlagBits::eTransferSrc,
        vk::MemoryPropertyFlagBits::eHostVisible |
            vk::MemoryPropertyFlagBits::eHostCoherent,
        staging_buf, staging_buf_mem);

    void* data = device_.mapMemory(staging_buf_mem, 0, image_size).value;
    memcpy(data, texture->pixels, static_cast<size_t>(image_size));
    device_.unmapMemory(staging_buf_mem);
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
        texture_img_, texture_fmt_, mip_levels_, vk::ImageLayout::eUndefined,
        vk::ImageLayout::eTransferDstOptimal);
    copyBufferToImage(staging_buf, texture_img_, width, height);
    generateMipmaps(texture_img_, width, height, texture_fmt_, mip_levels_);
    // Transitioned to vk::ImageLayout::eShaderReadOnlyOptimal while generating
    // mipmaps.

    device_.destroyBuffer(staging_buf);
    device_.freeMemory(staging_buf_mem);
  }

  void createImage(
      uint32_t width, uint32_t height, vk::Format format, uint32_t mip_levels,
      vk::SampleCountFlagBits num_samples, vk::ImageTiling tiling,
      vk::ImageUsageFlags usage, vk::MemoryPropertyFlags props, vk::Image& img,
      vk::DeviceMemory& img_mem) {
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
    img = device_.createImage(img_ci).value;

    vk::MemoryRequirements mem_reqs = device_.getImageMemoryRequirements(img);

    vk::MemoryAllocateInfo alloc_info{
        .allocationSize = mem_reqs.size,
        .memoryTypeIndex = findMemoryType(mem_reqs.memoryTypeBits, props),
    };
    img_mem = device_.allocateMemory(alloc_info).value;

    std::ignore = device_.bindImageMemory(img, img_mem, 0);
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
    ASSERT(
        format_props.optimalTilingFeatures &
        vk::FormatFeatureFlagBits::eSampledImageFilterLinear);

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
        texture_img_, texture_fmt_, mip_levels_,
        vk::ImageAspectFlagBits::eColor);
  }

  void createTextureSampler() {
    VkSamplerCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    ci.magFilter = VK_FILTER_LINEAR;
    ci.minFilter = VK_FILTER_LINEAR;
    ci.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    ci.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    ci.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    ci.anisotropyEnable = VK_TRUE;
    ci.maxAnisotropy = device_props_.limits.maxSamplerAnisotropy;
    ci.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    ci.unnormalizedCoordinates = VK_FALSE;
    ci.compareEnable = VK_FALSE;
    ci.compareOp = VK_COMPARE_OP_ALWAYS;
    ci.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    ci.mipLodBias = 0.f;
    ci.minLod = 0.f;
    ci.maxLod = mip_levels_;
    VKASSERT(vkCreateSampler(device_, &ci, nullptr, &texture_sampler_));
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
    VkDeviceSize size = sizeof(Vertex) * geom.vertices.size();
    stageBuffer(
        size, (void*)geom.vertices.data(),
        vk::BufferUsageFlagBits::eVertexBuffer, vert_buf_, vert_buf_mem_);
  }

  void createIndexBuffer() {
    VkDeviceSize size = sizeof(uint32_t) * geom.indices.size();
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
          device_.mapMemory(uniform_bufs_[i].buf_mem, 0, buf_size).value;
    }
  }

  // Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
  // from the staging_buf to dst_buf.
  void stageBuffer(
      vk::DeviceSize size, void* data, vk::BufferUsageFlags usage,
      vk::Buffer& dst_buf, vk::DeviceMemory& dst_buf_mem) {
    vk::Buffer staging_buf;
    vk::DeviceMemory staging_buf_mem;
    createBuffer(
        size, vk::BufferUsageFlagBits::eTransferSrc,
        vk::MemoryPropertyFlagBits::eHostVisible |
            vk::MemoryPropertyFlagBits::eHostCoherent,
        staging_buf, staging_buf_mem);

    void* staging_data = device_.mapMemory(staging_buf_mem, 0, size).value;
    memcpy(staging_data, data, (size_t)size);
    device_.unmapMemory(staging_buf_mem);

    createBuffer(
        size, usage | vk::BufferUsageFlagBits::eTransferDst,
        vk::MemoryPropertyFlagBits::eDeviceLocal, dst_buf, dst_buf_mem);
    copyBuffer(staging_buf, dst_buf, size);

    device_.destroyBuffer(staging_buf);
    device_.freeMemory(staging_buf_mem);
  }

  void copyBuffer(vk::Buffer src_buf, vk::Buffer dst_buf, vk::DeviceSize size) {
    vk::CommandBuffer cmd_buf = beginSingleTimeCommands();

    vk::BufferCopy copy_region{
        .size = size,
    };
    cmd_buf.copyBuffer(src_buf, dst_buf, copy_region);

    endSingleTimeCommands(cmd_buf);
  }

  VkCommandBuffer beginSingleTimeCommands() {
    VkCommandBufferAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandPool = cmd_pool_;
    alloc_info.commandBufferCount = 1;

    VkCommandBuffer cmd_buf;
    VKASSERT(vkAllocateCommandBuffers(device_, &alloc_info, &cmd_buf));

    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VKASSERT(vkBeginCommandBuffer(cmd_buf, &begin_info));

    return cmd_buf;
  }

  void endSingleTimeCommands(VkCommandBuffer cmd_buf) {
    VKASSERT(vkEndCommandBuffer(cmd_buf));

    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd_buf;
    VKASSERT(vkQueueSubmit(gfx_q_, 1, &submit, VK_NULL_HANDLE));
    VKASSERT(vkQueueWaitIdle(gfx_q_));

    vkFreeCommandBuffers(device_, cmd_pool_, 1, &cmd_buf);
  }

  void createBuffer(
      vk::DeviceSize size, vk::BufferUsageFlags usage,
      vk::MemoryPropertyFlags props, vk::Buffer& buf,
      vk::DeviceMemory& buf_mem) {
    vk::BufferCreateInfo buffer_ci{
        .size = size,
        .usage = usage,
        .sharingMode = vk::SharingMode::eExclusive,
    };
    buf = device_.createBuffer(buffer_ci).value;

    vk::MemoryRequirements mem_reqs = device_.getBufferMemoryRequirements(buf);

    vk::MemoryAllocateInfo alloc_info{
        .allocationSize = mem_reqs.size,
        .memoryTypeIndex = findMemoryType(mem_reqs.memoryTypeBits, props),
    };
    buf_mem = device_.allocateMemory(alloc_info).value;
    std::ignore = device_.bindBufferMemory(buf, buf_mem, 0);
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
  }

  void createDescriptorPool() {
    std::array<VkDescriptorPoolSize, 2> pool_sizes{};
    pool_sizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    pool_sizes[0].descriptorCount = MAX_FRAMES_IN_FLIGHT;
    pool_sizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    pool_sizes[1].descriptorCount = MAX_FRAMES_IN_FLIGHT;

    VkDescriptorPoolCreateInfo pool_ci{};
    pool_ci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_ci.poolSizeCount = pool_sizes.size();
    pool_ci.pPoolSizes = pool_sizes.data();
    pool_ci.maxSets = MAX_FRAMES_IN_FLIGHT;
    VKASSERT(vkCreateDescriptorPool(device_, &pool_ci, nullptr, &desc_pool_));
  }

  void createDescriptorSets() {
    std::vector<VkDescriptorSetLayout> layouts(
        MAX_FRAMES_IN_FLIGHT, desc_set_layout_);
    VkDescriptorSetAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc_info.descriptorPool = desc_pool_;
    alloc_info.descriptorSetCount = MAX_FRAMES_IN_FLIGHT;
    alloc_info.pSetLayouts = layouts.data();

    std::vector<VkDescriptorSet> desc_sets(MAX_FRAMES_IN_FLIGHT);
    VKASSERT(vkAllocateDescriptorSets(device_, &alloc_info, desc_sets.data()));

    ASSERT(desc_sets.size() == uniform_bufs_.size());
    for (int i = 0; i < uniform_bufs_.size(); i++) {
      auto& buf_state = uniform_bufs_[i];
      buf_state.desc_set = desc_sets[i];

      std::array<VkWriteDescriptorSet, 2> desc_writes{};

      VkDescriptorBufferInfo buffer_info{};
      buffer_info.buffer = buf_state.buf;
      buffer_info.offset = 0;
      buffer_info.range = sizeof(UniformBufferObject);

      desc_writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      desc_writes[0].dstSet = buf_state.desc_set;
      desc_writes[0].dstBinding = 0;
      desc_writes[0].dstArrayElement = 0;
      desc_writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
      desc_writes[0].descriptorCount = 1;
      desc_writes[0].pBufferInfo = &buffer_info;

      VkDescriptorImageInfo image_info{};
      image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      image_info.imageView = texture_img_view_;
      image_info.sampler = texture_sampler_;

      desc_writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      desc_writes[1].dstSet = buf_state.desc_set;
      desc_writes[1].dstBinding = 1;
      desc_writes[1].dstArrayElement = 0;
      desc_writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
      desc_writes[1].descriptorCount = 1;
      desc_writes[1].pImageInfo = &image_info;

      vkUpdateDescriptorSets(
          device_, desc_writes.size(), desc_writes.data(), 0, nullptr);
    }
  }

  void createCommandBuffers() {
    cmd_bufs_.resize(MAX_FRAMES_IN_FLIGHT);

    VkCommandBufferAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.commandPool = cmd_pool_;
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandBufferCount = cmd_bufs_.size();
    VKASSERT(vkAllocateCommandBuffers(device_, &alloc_info, cmd_bufs_.data()));
  }

  void recordCommandBuffer(VkCommandBuffer cmd_buf, uint32_t img_ind) {
    VkCommandBufferBeginInfo begin_info{};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    VKASSERT(vkBeginCommandBuffer(cmd_buf, &begin_info));

    VkRenderPassBeginInfo rp_info{};
    rp_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp_info.renderPass = render_pass_;
    rp_info.framebuffer = swapchain_fbs_[img_ind];
    rp_info.renderArea.offset = {0, 0};
    rp_info.renderArea.extent = swapchain_extent_;
    // float val = anim_.clear_val;
    float val = 0.f;
    std::array<VkClearValue, 2> clear_values{};
    clear_values[0].color = {{val, val, val, 1.0f}};
    clear_values[1].depthStencil = {1.f, 0};
    rp_info.clearValueCount = clear_values.size();
    rp_info.pClearValues = clear_values.data();
    vkCmdBeginRenderPass(cmd_buf, &rp_info, VK_SUBPASS_CONTENTS_INLINE);

    vkCmdBindPipeline(cmd_buf, VK_PIPELINE_BIND_POINT_GRAPHICS, gfx_pipeline_);

    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = (float)swapchain_extent_.width;
    viewport.height = (float)swapchain_extent_.height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd_buf, 0, 1, &viewport);

    VkRect2D scissor{};
    scissor.offset = {0, 0};
    scissor.extent = swapchain_extent_;
    vkCmdSetScissor(cmd_buf, 0, 1, &scissor);

    auto buf_state = uniform_bufs_[frame_num_ % MAX_FRAMES_IN_FLIGHT];
    vkCmdBindDescriptorSets(
        cmd_buf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_, 0, 1,
        (VkDescriptorSet*)&buf_state.desc_set, 0, nullptr);

    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(cmd_buf, 0, 1, (VkBuffer*)&vert_buf_, offsets);
    vkCmdBindIndexBuffer(cmd_buf, ind_buf_, 0, VK_INDEX_TYPE_UINT32);

    vkCmdDrawIndexed(cmd_buf, geom.indices.size(), 1, 0, 0, 0);
    vkCmdEndRenderPass(cmd_buf);
    VKASSERT(vkEndCommandBuffer(cmd_buf));
  }

  void createSyncObjects() {
    VkSemaphoreCreateInfo sem_ci{};
    sem_ci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fence_ci{};
    fence_ci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence_ci.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    img_sems_.resize(MAX_FRAMES_IN_FLIGHT);
    render_sems_.resize(MAX_FRAMES_IN_FLIGHT);
    in_flight_fences_.resize(MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
      VKASSERT(vkCreateSemaphore(device_, &sem_ci, nullptr, &img_sems_[i]));
      VKASSERT(vkCreateSemaphore(device_, &sem_ci, nullptr, &render_sems_[i]));
      VKASSERT(
          vkCreateFence(device_, &fence_ci, nullptr, &in_flight_fences_[i]));
    }
  }

  void recreateSwapchain() {
    int width = 0;
    int height = 0;
    SDL_GL_GetDrawableSize(window_, &width, &height);
    if (width == 0 || height == 0) {
      empty_window_ = true;
      return;
    }

    if (swapchain_extent_.width == width &&
        swapchain_extent_.height == height) {
      return;
    }

    vkDeviceWaitIdle(device_);

    cleanupSwapchain();

    swapchain_support_ = querySwapchainSupport(physical_device_);
    createSwapchain();
    createImageViews();
    createColorResources();
    createDepthResources();
    createFrameBuffers();
  }

  void cleanupSwapchain() {
    vkDestroyImageView(device_, depth_img_view_, nullptr);
    vkDestroyImage(device_, depth_img_, nullptr);
    vkFreeMemory(device_, depth_img_mem_, nullptr);

    vkDestroyImageView(device_, color_img_view_, nullptr);
    vkDestroyImage(device_, color_img_, nullptr);
    vkFreeMemory(device_, color_img_mem_, nullptr);

    for (auto fb : swapchain_fbs_) {
      vkDestroyFramebuffer(device_, fb, nullptr);
    }
    for (auto image_view : swapchain_views_) {
      vkDestroyImageView(device_, image_view, nullptr);
    }
    vkDestroySwapchainKHR(device_, swapchain_, nullptr);
  }

  void cleanup() {
    cleanupSwapchain();

    vkDestroyBuffer(device_, vert_buf_, nullptr);
    vkFreeMemory(device_, vert_buf_mem_, nullptr);
    vkDestroyBuffer(device_, ind_buf_, nullptr);
    vkFreeMemory(device_, ind_buf_mem_, nullptr);
    vkDestroySampler(device_, texture_sampler_, nullptr);
    vkDestroyImageView(device_, texture_img_view_, nullptr);
    vkDestroyImage(device_, texture_img_, nullptr);
    vkFreeMemory(device_, texture_img_mem_, nullptr);

    for (auto& buf_state : uniform_bufs_) {
      vkDestroyBuffer(device_, buf_state.buf, nullptr);
      vkFreeMemory(device_, buf_state.buf_mem, nullptr);
      // paranoia
      buf_state.buf_mapped = nullptr;
      buf_state.desc_set = VK_NULL_HANDLE;  // cleaned up by the pool
    }

    vkDestroyDescriptorPool(device_, desc_pool_, nullptr);
    vkDestroyDescriptorSetLayout(device_, desc_set_layout_, nullptr);
    vkDestroyPipeline(device_, gfx_pipeline_, nullptr);
    vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
    vkDestroyRenderPass(device_, render_pass_, nullptr);
    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
      vkDestroySemaphore(device_, img_sems_[i], nullptr);
      vkDestroySemaphore(device_, render_sems_[i], nullptr);
      vkDestroyFence(device_, in_flight_fences_[i], nullptr);
    }
    vkDestroyCommandPool(device_, cmd_pool_, nullptr);
    vkDestroyDevice(device_, nullptr);
    if (enable_validation_layers_) {
      auto destroy_dbg_fn = LOAD_VK_FN(vkDestroyDebugUtilsMessengerEXT);
      ASSERT(destroy_dbg_fn);
      destroy_dbg_fn(instance_, dbg_messenger_, nullptr);
    }
    vkDestroySurfaceKHR(instance_, surface_, nullptr);
    vkDestroyInstance(instance_, nullptr);

    SDL_DestroyWindow(window_);
    SDL_Quit();
  }

  SDL_Window* window_ = nullptr;

  bool quit_ = false;
  bool window_resized_ = false;
  bool window_minimized_ = false;
  bool empty_window_ = false;

  uint64_t frame_num_ = 0;
  Time last_frame_time_;
  // Time in ms since the last draw, as a float
  float time_delta_ms_ = 0.0f;

  // Used to calculate FPS
  uint64_t last_fps_frame_ = 0;
  Time next_fps_time_;

  AnimationState anim_;

  vk::Instance instance_;
  vk::DebugUtilsMessengerEXT dbg_messenger_;
  vk::SurfaceKHR surface_;
  vk::PhysicalDevice physical_device_;
  vk::PhysicalDeviceProperties device_props_;
  // Indices of queue families for the selected |physical_device_|
  QueueFamilyIndices q_indices_;
  vk::Device device_;
  vk::Queue gfx_q_;
  vk::Queue present_q_;
  SwapchainSupportDetails swapchain_support_;
  vk::SwapchainKHR swapchain_;
  std::vector<vk::Image> swapchain_images_;
  vk::Format swapchain_format_;
  vk::Extent2D swapchain_extent_;
  std::vector<vk::ImageView> swapchain_views_;
  vk::RenderPass render_pass_;
  vk::DescriptorSetLayout desc_set_layout_;
  VkDescriptorPool desc_pool_;
  vk::PipelineLayout pipeline_layout_;
  vk::Pipeline gfx_pipeline_;
  std::vector<vk::Framebuffer> swapchain_fbs_;
  vk::CommandPool cmd_pool_;
  std::vector<VkCommandBuffer> cmd_bufs_;
  std::vector<VkSemaphore> img_sems_;
  std::vector<VkSemaphore> render_sems_;
  std::vector<VkFence> in_flight_fences_;
  vk::Buffer vert_buf_;
  vk::DeviceMemory vert_buf_mem_;
  vk::Buffer ind_buf_;
  vk::DeviceMemory ind_buf_mem_;
  vk::Format texture_fmt_;
  uint32_t mip_levels_;
  vk::Image texture_img_;
  vk::DeviceMemory texture_img_mem_;
  vk::ImageView texture_img_view_;
  VkSampler texture_sampler_;
  vk::Image color_img_;
  vk::DeviceMemory color_img_mem_;
  vk::ImageView color_img_view_;
  vk::Format depth_fmt_;
  vk::Image depth_img_;
  vk::DeviceMemory depth_img_mem_;
  vk::ImageView depth_img_view_;
  vk::SampleCountFlagBits msaa_samples_ = vk::SampleCountFlagBits::e1;

  Geometry geom;

  struct UniformBufferState {
    vk::Buffer buf;
    vk::DeviceMemory buf_mem;
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
