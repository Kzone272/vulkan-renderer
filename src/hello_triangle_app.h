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

  static VkVertexInputBindingDescription getBindingDesc() {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(Vertex);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    return binding;
  }

  static std::array<VkVertexInputAttributeDescription, 3> getAttrDescs() {
    std::array<VkVertexInputAttributeDescription, 3> attrs{};
    attrs[0].binding = 0;
    attrs[0].location = 0;
    attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;  // vec3
    attrs[0].offset = offsetof(Vertex, pos);

    attrs[1].binding = 0;
    attrs[1].location = 1;
    attrs[1].format = VK_FORMAT_R32G32B32_SFLOAT;  // vec3
    attrs[1].offset = offsetof(Vertex, color);

    attrs[2].binding = 0;
    attrs[2].location = 2;
    attrs[2].format = VK_FORMAT_R32G32_SFLOAT;  // vec2
    attrs[2].offset = offsetof(Vertex, uv);

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
    int gfx_family = -1;
    int present_family = -1;

    bool isComplete() {
      return gfx_family != -1 && present_family != -1;
    }
  };

  QueueFamilyIndices findQueueFamilies(vk::PhysicalDevice device) {
    auto q_families = device.getQueueFamilyProperties();

    QueueFamilyIndices indices;
    int i = 0;
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
    std::set<int> unique_q_indices = {
        q_indices_.gfx_family, q_indices_.present_family};
    float q_prio = 1.0f;
    std::vector<vk::DeviceQueueCreateInfo> device_q_cis;
    for (int q_index : unique_q_indices) {
      device_q_cis.push_back({
          .queueFamilyIndex = static_cast<uint32_t>(q_index),
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
        static_cast<uint32_t>(q_indices_.gfx_family),
        static_cast<uint32_t>(q_indices_.present_family)};
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
          swapchain_images_[i], (VkFormat)swapchain_format_, 1,
          VK_IMAGE_ASPECT_COLOR_BIT);
    }
  }

  VkImageView createImageView(
      VkImage img, VkFormat format, uint32_t mip_levels,
      VkImageAspectFlags aspect_flags) {
    VkImageViewCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    ci.image = img;
    ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    ci.format = format;
    ci.subresourceRange.aspectMask = aspect_flags;
    ci.subresourceRange.baseMipLevel = 0;
    ci.subresourceRange.levelCount = mip_levels;
    ci.subresourceRange.baseArrayLayer = 0;
    ci.subresourceRange.layerCount = 1;

    VkImageView img_view;
    VKASSERT(vkCreateImageView(device_, &ci, nullptr, &img_view));

    return img_view;
  }

  void createRenderPass() {
    VkAttachmentDescription color_att{};
    color_att.format = (VkFormat)swapchain_format_;
    color_att.samples = (VkSampleCountFlagBits)msaa_samples_;
    color_att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color_att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color_att.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference color_att_ref{};
    color_att_ref.attachment = 0;
    color_att_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    depth_fmt_ = findDepthFormat();
    VkAttachmentDescription depth_att{};
    depth_att.format = depth_fmt_;
    depth_att.samples = (VkSampleCountFlagBits)msaa_samples_;
    depth_att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth_att.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth_att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth_att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth_att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth_att.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depth_att_ref{};
    depth_att_ref.attachment = 1;
    depth_att_ref.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentDescription color_resolve_att{};
    color_resolve_att.format = (VkFormat)swapchain_format_;
    color_resolve_att.samples = VK_SAMPLE_COUNT_1_BIT;
    color_resolve_att.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_resolve_att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_resolve_att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_resolve_att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color_resolve_att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color_resolve_att.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference color_resolve_att_ref{};
    color_resolve_att_ref.attachment = 2;
    color_resolve_att_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &color_att_ref;
    subpass.pDepthStencilAttachment = &depth_att_ref;
    subpass.pResolveAttachments = &color_resolve_att_ref;

    // Writing to the subpass color attachment depends on the swapchain
    // finishing its read of the color attachment.
    // We also need to read and write to the depth buffer.
    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                       VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = 0;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                       VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    std::array<VkAttachmentDescription, 3> atts = {
        color_att, depth_att, color_resolve_att};
    VkRenderPassCreateInfo rp_ci{};
    rp_ci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rp_ci.attachmentCount = atts.size();
    rp_ci.pAttachments = atts.data();
    rp_ci.subpassCount = 1;
    rp_ci.pSubpasses = &subpass;
    rp_ci.dependencyCount = 1;
    rp_ci.pDependencies = &dep;
    VKASSERT(vkCreateRenderPass(device_, &rp_ci, nullptr, &render_pass_));
  }

  void createDescriptorSetLayout() {
    VkDescriptorSetLayoutBinding ubo_binding{};
    ubo_binding.binding = 0;
    ubo_binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    ubo_binding.descriptorCount = 1;
    ubo_binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    VkDescriptorSetLayoutBinding sampler_binding{};
    sampler_binding.binding = 1;
    sampler_binding.descriptorCount = 1;
    sampler_binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sampler_binding.pImmutableSamplers = nullptr;
    sampler_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    std::array<VkDescriptorSetLayoutBinding, 2> bindings = {
        ubo_binding, sampler_binding};

    VkDescriptorSetLayoutCreateInfo layout_ci{};
    layout_ci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layout_ci.bindingCount = bindings.size();
    layout_ci.pBindings = bindings.data();
    VKASSERT(vkCreateDescriptorSetLayout(
        device_, &layout_ci, nullptr, &desc_set_layout_));
  }

  void createGraphicsPipeline() {
    auto vert_shader_code = readFile("shaders/shader.vert.spv");
    auto frag_shader_code = readFile("shaders/shader.frag.spv");

    VkShaderModule vert_shader = createShaderModule(vert_shader_code);
    VkShaderModule frag_shader = createShaderModule(frag_shader_code);

    VkPipelineShaderStageCreateInfo vert_shader_stage_ci{};
    vert_shader_stage_ci.sType =
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    vert_shader_stage_ci.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vert_shader_stage_ci.module = vert_shader;
    vert_shader_stage_ci.pName = "main";

    VkPipelineShaderStageCreateInfo frag_shader_stage_ci{};
    frag_shader_stage_ci.sType =
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    frag_shader_stage_ci.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    frag_shader_stage_ci.module = frag_shader;
    frag_shader_stage_ci.pName = "main";

    std::vector<VkPipelineShaderStageCreateInfo> shader_stages = {
        vert_shader_stage_ci, frag_shader_stage_ci};

    std::vector<VkDynamicState> dyn_states = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR,
    };
    VkPipelineDynamicStateCreateInfo dyn_state{};
    dyn_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dyn_state.dynamicStateCount = dyn_states.size();
    dyn_state.pDynamicStates = dyn_states.data();

    auto binding = Vertex::getBindingDesc();
    auto attrs = Vertex::getAttrDescs();

    VkPipelineVertexInputStateCreateInfo vert_in_info{};
    vert_in_info.sType =
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vert_in_info.vertexBindingDescriptionCount = 1;
    vert_in_info.pVertexBindingDescriptions = &binding;
    vert_in_info.vertexAttributeDescriptionCount = attrs.size();
    vert_in_info.pVertexAttributeDescriptions = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
    input_assembly.sType =
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    input_assembly.primitiveRestartEnable = VK_FALSE;

    VkPipelineViewportStateCreateInfo viewport_state{};
    viewport_state.sType =
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewport_state.viewportCount = 1;
    viewport_state.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterizer{};
    rasterizer.sType =
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rasterizer.depthClampEnable = VK_FALSE;
    rasterizer.rasterizerDiscardEnable = VK_FALSE;
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
    rasterizer.depthBiasEnable = VK_FALSE;

    VkPipelineMultisampleStateCreateInfo multisampling{};
    multisampling.sType =
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisampling.sampleShadingEnable = VK_FALSE;
    multisampling.rasterizationSamples = (VkSampleCountFlagBits)msaa_samples_;

    VkPipelineColorBlendAttachmentState color_blend_att{};
    color_blend_att.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    color_blend_att.blendEnable = VK_TRUE;
    color_blend_att.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    color_blend_att.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    color_blend_att.colorBlendOp = VK_BLEND_OP_ADD;
    color_blend_att.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    color_blend_att.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;

    VkPipelineColorBlendStateCreateInfo color_blending{};
    color_blending.sType =
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    color_blending.logicOpEnable = VK_FALSE;
    color_blending.attachmentCount = 1;
    color_blending.pAttachments = &color_blend_att;

    VkPipelineDepthStencilStateCreateInfo depth_ci{};
    depth_ci.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depth_ci.depthTestEnable = VK_TRUE;
    depth_ci.depthWriteEnable = VK_TRUE;
    depth_ci.depthCompareOp = VK_COMPARE_OP_LESS;
    depth_ci.stencilTestEnable = VK_FALSE;

    VkPipelineLayoutCreateInfo pipeline_layout_ci{};
    pipeline_layout_ci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_ci.setLayoutCount = 1;
    pipeline_layout_ci.pSetLayouts = &desc_set_layout_;

    VKASSERT(vkCreatePipelineLayout(
        device_, &pipeline_layout_ci, nullptr, &pipeline_layout_));

    VkGraphicsPipelineCreateInfo pipeline_ci{};
    pipeline_ci.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    pipeline_ci.stageCount = shader_stages.size();
    pipeline_ci.pStages = shader_stages.data();
    pipeline_ci.pVertexInputState = &vert_in_info;
    pipeline_ci.pInputAssemblyState = &input_assembly;
    pipeline_ci.pViewportState = &viewport_state;
    pipeline_ci.pRasterizationState = &rasterizer;
    pipeline_ci.pMultisampleState = &multisampling;
    pipeline_ci.pDepthStencilState = &depth_ci;
    pipeline_ci.pColorBlendState = &color_blending;
    pipeline_ci.pDynamicState = &dyn_state;
    pipeline_ci.layout = pipeline_layout_;
    pipeline_ci.renderPass = render_pass_;
    pipeline_ci.subpass = 0;
    VKASSERT(vkCreateGraphicsPipelines(
        device_, VK_NULL_HANDLE, 1, &pipeline_ci, nullptr, &gfx_pipeline_));

    // Cleanup
    vkDestroyShaderModule(device_, vert_shader, nullptr);
    vkDestroyShaderModule(device_, frag_shader, nullptr);
  }

  VkShaderModule createShaderModule(const std::vector<char>& code) {
    VkShaderModuleCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    ci.codeSize = code.size();
    ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule shader_module;
    VKASSERT(vkCreateShaderModule(device_, &ci, nullptr, &shader_module));
    return shader_module;
  }

  void createFrameBuffers() {
    swapchain_fbs_.resize(swapchain_views_.size());
    for (size_t i = 0; i < swapchain_views_.size(); i++) {
      std::array<VkImageView, 3> atts = {
          color_img_view_, depth_img_view_, swapchain_views_[i]};

      VkFramebufferCreateInfo fb_ci{};
      fb_ci.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
      fb_ci.renderPass = render_pass_;
      fb_ci.attachmentCount = atts.size();
      fb_ci.pAttachments = atts.data();
      fb_ci.width = swapchain_extent_.width;
      fb_ci.height = swapchain_extent_.height;
      fb_ci.layers = 1;
      VKASSERT(
          vkCreateFramebuffer(device_, &fb_ci, nullptr, &swapchain_fbs_[i]));
    }
  }

  void createCommandPool() {
    VkCommandPoolCreateInfo pool_ci{};
    pool_ci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    pool_ci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_ci.queueFamilyIndex = q_indices_.gfx_family;
    VKASSERT(vkCreateCommandPool(device_, &pool_ci, nullptr, &cmd_pool_));
  }

  void createColorResources() {
    VkFormat color_fmt = (VkFormat)swapchain_format_;
    createImage(
        swapchain_extent_.width, swapchain_extent_.height, color_fmt, 1,
        (VkSampleCountFlagBits)msaa_samples_, VK_IMAGE_TILING_OPTIMAL,
        VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT |
            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, color_img_, color_img_mem_);
    color_img_view_ =
        createImageView(color_img_, color_fmt, 1, VK_IMAGE_ASPECT_COLOR_BIT);
  }

  void createDepthResources() {
    createImage(
        swapchain_extent_.width, swapchain_extent_.height, depth_fmt_, 1,
        (VkSampleCountFlagBits)msaa_samples_, VK_IMAGE_TILING_OPTIMAL,
        VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, depth_img_, depth_img_mem_);
    depth_img_view_ =
        createImageView(depth_img_, depth_fmt_, 1, VK_IMAGE_ASPECT_DEPTH_BIT);
    transitionImageLayout(
        depth_img_, depth_fmt_, 1, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
  }

  VkFormat findDepthFormat() {
    return findSupportedFormat(
        {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT,
         VK_FORMAT_D24_UNORM_S8_UINT},
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
  }

  bool hasStencilComponent(VkFormat format) {
    return format == VK_FORMAT_D32_SFLOAT_S8_UINT ||
           format == VK_FORMAT_D24_UNORM_S8_UINT;
  }

  VkFormat findSupportedFormat(
      const std::vector<VkFormat>& formats, VkImageTiling tiling,
      VkFormatFeatureFlags features) {
    ASSERT(
        tiling == VK_IMAGE_TILING_LINEAR || tiling == VK_IMAGE_TILING_OPTIMAL);

    for (const auto& format : formats) {
      VkFormatProperties props;
      vkGetPhysicalDeviceFormatProperties(physical_device_, format, &props);

      auto tiling_features = tiling == VK_IMAGE_TILING_LINEAR
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
    VkDeviceSize image_size = width * height * 4;
    mip_levels_ = std::floor(std::log2(std::max(width, height))) + 1;

    VkBuffer staging_buf;
    VkDeviceMemory staging_buf_mem;
    createBuffer(
        image_size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        staging_buf, staging_buf_mem);

    void* data;
    vkMapMemory(device_, staging_buf_mem, 0, image_size, 0, &data);
    memcpy(data, texture->pixels, static_cast<size_t>(image_size));
    vkUnmapMemory(device_, staging_buf_mem);
    SDL_FreeSurface(texture);

    texture_fmt_ = VK_FORMAT_R8G8B8A8_SRGB;
    createImage(
        width, height, texture_fmt_, mip_levels_, VK_SAMPLE_COUNT_1_BIT,
        VK_IMAGE_TILING_OPTIMAL,
        VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
            VK_IMAGE_USAGE_SAMPLED_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, texture_img_, texture_img_mem_);

    transitionImageLayout(
        texture_img_, texture_fmt_, mip_levels_, VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    copyBufferToImage(staging_buf, texture_img_, width, height);
    generateMipmaps(texture_img_, width, height, texture_fmt_, mip_levels_);
    // Transitioned to VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL while generating
    // mipmaps.

    vkDestroyBuffer(device_, staging_buf, nullptr);
    vkFreeMemory(device_, staging_buf_mem, nullptr);
  }

  void createImage(
      uint32_t width, uint32_t height, VkFormat format, uint32_t mip_levels,
      VkSampleCountFlagBits num_samples, VkImageTiling tiling,
      VkImageUsageFlags usage, VkMemoryPropertyFlags props, VkImage& img,
      VkDeviceMemory& img_mem) {
    VkImageCreateInfo img_ci{};
    img_ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    img_ci.imageType = VK_IMAGE_TYPE_2D;
    img_ci.extent.width = width;
    img_ci.extent.height = height;
    img_ci.extent.depth = 1;
    img_ci.mipLevels = mip_levels;
    img_ci.arrayLayers = 1;
    img_ci.format = format;
    img_ci.tiling = tiling;
    img_ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    img_ci.usage = usage;
    img_ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    img_ci.samples = num_samples;
    VKASSERT(vkCreateImage(device_, &img_ci, nullptr, &img));

    VkMemoryRequirements mem_reqs;
    vkGetImageMemoryRequirements(device_, img, &mem_reqs);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_reqs.size;
    alloc_info.memoryTypeIndex = findMemoryType(mem_reqs.memoryTypeBits, props);
    VKASSERT(vkAllocateMemory(device_, &alloc_info, nullptr, &img_mem));

    VKASSERT(vkBindImageMemory(device_, img, img_mem, 0));
  }

  void transitionImageLayout(
      VkImage img, VkFormat format, uint32_t mip_levels,
      VkImageLayout old_layout, VkImageLayout new_layout) {
    VkCommandBuffer cmd_buf = beginSingleTimeCommands();

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = old_layout;
    barrier.newLayout = new_layout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = img;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = mip_levels;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;

    if (new_layout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
      barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
      if (hasStencilComponent(format)) {
        barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
      }
    } else {
      barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    }

    VkPipelineStageFlags src_stage;
    VkPipelineStageFlags dst_stage;
    if (old_layout == VK_IMAGE_LAYOUT_UNDEFINED &&
        new_layout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
      barrier.srcAccessMask = 0;
      barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      src_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
      dst_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    } else if (
        old_layout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL &&
        new_layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
      src_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
      dst_stage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    } else if (
        old_layout == VK_IMAGE_LAYOUT_UNDEFINED &&
        new_layout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
      barrier.srcAccessMask = 0;
      barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                              VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
      src_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
      dst_stage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    } else {
      printf(
          "Unsupported layout transition! (%d -> %d)\n", old_layout,
          new_layout);
      ASSERT(false);
    }

    vkCmdPipelineBarrier(
        cmd_buf, src_stage, dst_stage, 0, 0, nullptr, 0, nullptr, 1, &barrier);

    endSingleTimeCommands(cmd_buf);
  }

  void copyBufferToImage(
      VkBuffer buf, VkImage img, uint32_t width, uint32_t height) {
    VkCommandBuffer cmd_buf = beginSingleTimeCommands();

    VkBufferImageCopy region{};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = {0, 0, 0};
    region.imageExtent = {width, height, 1};

    vkCmdCopyBufferToImage(
        cmd_buf, buf, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    endSingleTimeCommands(cmd_buf);
  }

  void generateMipmaps(
      VkImage img, int32_t width, int32_t height, VkFormat format,
      uint32_t mip_levels) {
    VkFormatProperties format_props;
    vkGetPhysicalDeviceFormatProperties(
        physical_device_, format, &format_props);
    ASSERT(
        format_props.optimalTilingFeatures &
        VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT);

    VkCommandBuffer cmd_buf = beginSingleTimeCommands();

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.image = img;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.layerCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.levelCount = 1;

    int32_t mip_width = width;
    int32_t mip_height = height;
    for (uint32_t i = 0; i < mip_levels - 1; i++) {
      // Transition mip i to to be a copy source.
      barrier.subresourceRange.baseMipLevel = i;
      barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
      barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
      barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      vkCmdPipelineBarrier(
          cmd_buf, VK_PIPELINE_STAGE_TRANSFER_BIT,
          VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1,
          &barrier);

      int32_t src_width = mip_width;
      int32_t src_height = mip_height;
      if (src_width > 1) {
        mip_width /= 2;
      }
      if (src_height > 1) {
        mip_height /= 2;
      }

      // Blit from mip i to mip i+1 at half the size.
      VkImageBlit blit{};
      blit.srcOffsets[0] = {0, 0, 0};
      blit.srcOffsets[1] = {src_width, src_height, 1};
      blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      blit.srcSubresource.mipLevel = i;
      blit.srcSubresource.layerCount = 1;
      blit.srcSubresource.baseArrayLayer = 0;
      blit.dstOffsets[0] = {0, 0, 0};
      blit.dstOffsets[1] = {mip_width, mip_height, 1};
      blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      blit.dstSubresource.mipLevel = i + 1;
      blit.dstSubresource.layerCount = 1;
      blit.dstSubresource.baseArrayLayer = 0;
      vkCmdBlitImage(
          cmd_buf, img, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, img,
          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

      // Transition mip i to be shader readable.
      barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
      barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
      barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
      vkCmdPipelineBarrier(
          cmd_buf, VK_PIPELINE_STAGE_TRANSFER_BIT,
          VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1,
          &barrier);
    }
    // Transition last mip level to be shader readable.
    barrier.subresourceRange.baseMipLevel = mip_levels - 1;
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(
        cmd_buf, VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1,
        &barrier);

    endSingleTimeCommands(cmd_buf);
  }

  void createTextureImageView() {
    texture_img_view_ = createImageView(
        texture_img_, texture_fmt_, mip_levels_, VK_IMAGE_ASPECT_COLOR_BIT);
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
        size, (void*)geom.vertices.data(), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        vert_buf_, vert_buf_mem_);
  }

  void createIndexBuffer() {
    VkDeviceSize size = sizeof(uint32_t) * geom.indices.size();
    stageBuffer(
        size, (void*)geom.indices.data(), VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
        ind_buf_, ind_buf_mem_);
  }

  void createUniformBuffers() {
    VkDeviceSize buf_size = sizeof(UniformBufferObject);
    uniform_bufs_.resize(MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
      createBuffer(
          buf_size, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
          uniform_bufs_[i].buf, uniform_bufs_[i].buf_mem);
      VKASSERT(vkMapMemory(
          device_, uniform_bufs_[i].buf_mem, 0, buf_size, 0,
          &uniform_bufs_[i].buf_mapped));
    }
  }

  // Copy data to a CPU staging buffer, create a GPU buffer, and submit a copy
  // from the staging_buf to dst_buf.
  void stageBuffer(
      VkDeviceSize size, void* data, VkBufferUsageFlags usage,
      VkBuffer& dst_buf, VkDeviceMemory& dst_buf_mem) {
    VkBuffer staging_buf;
    VkDeviceMemory staging_buf_mem;
    createBuffer(
        size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        staging_buf, staging_buf_mem);

    void* staging_data;
    VKASSERT(vkMapMemory(device_, staging_buf_mem, 0, size, 0, &staging_data));
    memcpy(staging_data, data, (size_t)size);
    vkUnmapMemory(device_, staging_buf_mem);

    createBuffer(
        size, usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, dst_buf, dst_buf_mem);
    copyBuffer(staging_buf, dst_buf, size);

    vkDestroyBuffer(device_, staging_buf, nullptr);
    vkFreeMemory(device_, staging_buf_mem, nullptr);
  }

  void copyBuffer(VkBuffer src_buf, VkBuffer dst_buf, VkDeviceSize size) {
    VkCommandBuffer cmd_buf = beginSingleTimeCommands();

    VkBufferCopy copy_region{};
    copy_region.size = size;
    vkCmdCopyBuffer(cmd_buf, src_buf, dst_buf, 1, &copy_region);

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
      VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags props,
      VkBuffer& buf, VkDeviceMemory& buf_mem) {
    VkBufferCreateInfo buffer_ci{};
    buffer_ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_ci.size = size;
    buffer_ci.usage = usage;
    buffer_ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VKASSERT(vkCreateBuffer(device_, &buffer_ci, nullptr, &buf));

    VkMemoryRequirements mem_reqs;
    vkGetBufferMemoryRequirements(device_, buf, &mem_reqs);

    VkMemoryAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.allocationSize = mem_reqs.size;
    alloc_info.memoryTypeIndex = findMemoryType(mem_reqs.memoryTypeBits, props);
    VKASSERT(vkAllocateMemory(device_, &alloc_info, nullptr, &buf_mem));
    VKASSERT(vkBindBufferMemory(device_, buf, buf_mem, 0));
  }

  uint32_t findMemoryType(uint32_t type_filter, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties mem_props;
    vkGetPhysicalDeviceMemoryProperties(physical_device_, &mem_props);

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
        &buf_state.desc_set, 0, nullptr);

    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(cmd_buf, 0, 1, &vert_buf_, offsets);
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
  VkRenderPass render_pass_;
  VkDescriptorSetLayout desc_set_layout_;
  VkDescriptorPool desc_pool_;
  VkPipelineLayout pipeline_layout_;
  VkPipeline gfx_pipeline_;
  std::vector<VkFramebuffer> swapchain_fbs_;
  VkCommandPool cmd_pool_;
  std::vector<VkCommandBuffer> cmd_bufs_;
  std::vector<VkSemaphore> img_sems_;
  std::vector<VkSemaphore> render_sems_;
  std::vector<VkFence> in_flight_fences_;
  VkBuffer vert_buf_;
  VkDeviceMemory vert_buf_mem_;
  VkBuffer ind_buf_;
  VkDeviceMemory ind_buf_mem_;
  VkFormat texture_fmt_;
  uint32_t mip_levels_;
  VkImage texture_img_;
  VkDeviceMemory texture_img_mem_;
  VkImageView texture_img_view_;
  VkSampler texture_sampler_;
  VkImage color_img_;
  VkDeviceMemory color_img_mem_;
  VkImageView color_img_view_;
  VkFormat depth_fmt_;
  VkImage depth_img_;
  VkDeviceMemory depth_img_mem_;
  VkImageView depth_img_view_;
  vk::SampleCountFlagBits msaa_samples_ = vk::SampleCountFlagBits::e1;

  Geometry geom;

  struct UniformBufferState {
    VkBuffer buf;
    VkDeviceMemory buf_mem;
    void* buf_mapped;
    VkDescriptorSet desc_set;
  };
  std::vector<UniformBufferState> uniform_bufs_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
