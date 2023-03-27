#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>

#include "asserts.h"
#include "defines.h"

using std::cerr;
using std::cout;
using std::endl;
using std::printf;

namespace {

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;

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
  printf("validation layer: %s\n", callback_data->pMessage);
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

class HelloTriangleApp {
 public:
  void run() {
    initWindow();
    initVulkan();
    mainLoop();
    cleanup();
  }

 private:
  void initWindow() {
    ASSERT(SDL_Init(SDL_INIT_VIDEO) == 0);

    SDL_WindowFlags window_flags = SDL_WINDOW_VULKAN;
    window_ = SDL_CreateWindow(
        "Vulkan Tutorial", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, window_flags);
    if (!window_) {
      std::string error{SDL_GetError()};
      cerr << error << endl;
      ASSERT(false);
    }
  }

  void initVulkan() {
    createInstance();
    if (enable_validation_layers_) {
      setupDebugMessenger();
    }
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapChain();
    createImageViews();
    createRenderPass();
    createGraphicsPipeline();
    createFrameBuffers();
    createCommandPool();
    createCommandBuffer();
    createSyncObjects();
  }

  void mainLoop() {
    SDL_Event event;
    while (true) {
      SDL_PollEvent(&event);
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE) {
        break;
      }
      drawFrame();
    }

    vkDeviceWaitIdle(device_);
  }

  void drawFrame() {
    vkWaitForFences(device_, 1, &in_flight_fence_, VK_TRUE, UINT64_MAX);
    VKASSERT(vkResetFences(device_, 1, &in_flight_fence_));

    uint32_t img_ind = -1;
    vkAcquireNextImageKHR(
        device_, swapchain_, UINT64_MAX, img_sem_, VK_NULL_HANDLE, &img_ind);
    ASSERT(img_ind >= 0);
    ASSERT(img_ind < swapchain_images_.size());

    VKASSERT(vkResetCommandBuffer(cmd_buffer_, 0));
    recordCommandBuffer(cmd_buffer_, img_ind);

    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    VkPipelineStageFlags wait_stages[] = {
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
    submit_info.waitSemaphoreCount = 1;
    submit_info.pWaitSemaphores = &img_sem_;
    submit_info.pWaitDstStageMask = wait_stages;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &cmd_buffer_;
    submit_info.signalSemaphoreCount = 1;
    submit_info.pSignalSemaphores = &render_sem_;
    VKASSERT(vkQueueSubmit(gfx_q_, 1, &submit_info, in_flight_fence_));

    VkPresentInfoKHR present_info{};
    present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present_info.waitSemaphoreCount = 1;
    present_info.pWaitSemaphores = &render_sem_;
    present_info.swapchainCount = 1;
    present_info.pSwapchains = &swapchain_;
    present_info.pImageIndices = &img_ind;
    vkQueuePresentKHR(present_q_, &present_info);
  }

  void createInstance() {
    printSupportedExtensions();

    VkApplicationInfo app_info{};
    app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app_info.pApplicationName = "Hello Triangle";
    app_info.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    app_info.pEngineName = "No Engine";
    app_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    app_info.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo instance_ci{};
    instance_ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_ci.pApplicationInfo = &app_info;

    auto ext_names = getRequiredExtensions();
    instance_ci.enabledExtensionCount = ext_names.size();
    instance_ci.ppEnabledExtensionNames = ext_names.data();

    auto validation_layers = getValidationLayers();
    if (validation_layers.size()) {
      instance_ci.enabledLayerCount = validation_layers.size();
      instance_ci.ppEnabledLayerNames = validation_layers.data();
    } else {
      instance_ci.enabledLayerCount = 0;
    }

    VkDebugUtilsMessengerCreateInfoEXT dbg_messenger_ci{};
    if (enable_validation_layers_) {
      makeDbgMessengerCi(dbg_messenger_ci);
      instance_ci.pNext = &dbg_messenger_ci;
    }

#if __APPLE__
    instance_ci.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

    VKASSERT(vkCreateInstance(&instance_ci, nullptr, &instance_));
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
    uint32_t sup_ext_count = 0;
    VKASSERT(vkEnumerateInstanceExtensionProperties(
        nullptr, &sup_ext_count, nullptr));
    std::vector<VkExtensionProperties> sup_exts(sup_ext_count);
    VKASSERT(vkEnumerateInstanceExtensionProperties(
        nullptr, &sup_ext_count, sup_exts.data()));
    printf("Supported instance extensions (%d)\n", sup_ext_count);
    for (auto& ext : sup_exts) {
      printf("  %s v%d\n", ext.extensionName, ext.specVersion);
    }
  }

  std::vector<const char*> getValidationLayers() {
    if (!enable_validation_layers_) {
      return {};
    }

    uint32_t layer_count = 0;
    VKASSERT(vkEnumerateInstanceLayerProperties(&layer_count, nullptr));
    std::vector<VkLayerProperties> layer_props(layer_count);
    VKASSERT(
        vkEnumerateInstanceLayerProperties(&layer_count, layer_props.data()));

    printf("Available layers (%u):\n", layer_count);
    for (const auto& layer_prop : layer_props) {
      printf("  %s\n", layer_prop.layerName);
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

  void makeDbgMessengerCi(VkDebugUtilsMessengerCreateInfoEXT& ci) {
    ci.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    ci.messageSeverity =
        // VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |  // toggle comment
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                     VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                     VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback = debugCallback;
    ci.pUserData = nullptr;
  }

  void setupDebugMessenger() {
    VkDebugUtilsMessengerCreateInfoEXT ci{};
    makeDbgMessengerCi(ci);
    auto create_fn = LOAD_VK_FN(vkCreateDebugUtilsMessengerEXT);
    ASSERT(create_fn);
    VKASSERT(create_fn(instance_, &ci, nullptr, &dbg_messenger_));
  }

  void createSurface() {
    ASSERT(SDL_Vulkan_CreateSurface(window_, instance_, &surface_));
  }

  void pickPhysicalDevice() {
    uint32_t device_count = 0;
    vkEnumeratePhysicalDevices(instance_, &device_count, nullptr);
    ASSERT(device_count > 0);
    std::vector<VkPhysicalDevice> devices(device_count);
    vkEnumeratePhysicalDevices(instance_, &device_count, devices.data());

    for (const auto& device : devices) {
      if (isDeviceSuitable(device)) {
        physical_device_ = device;
        break;
      }
    }
    ASSERT(physical_device_);
  }

  struct QueueFamilyIndices {
    int gfx_family = -1;
    int present_family = -1;

    bool isComplete() {
      return gfx_family != -1 && present_family != -1;
    }
  };

  bool isDeviceSuitable(VkPhysicalDevice device) {
    QueueFamilyIndices indices = findQueueFamilies(device);
    if (!indices.isComplete()) {
      return false;
    }
    if (!checkDeviceExtensionSupport(device)) {
      return false;
    }
    SwapchainSupportDetails swapchain_support = querySwapChainSupport(device);
    if (swapchain_support.formats.empty() ||
        swapchain_support.present_modes.empty()) {
      return false;
    }

    // Cache these because we'll use this device.
    q_indices_ = indices;
    swapchain_support_ = swapchain_support;

    return true;
  }

  QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) {
    uint32_t q_family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &q_family_count, nullptr);
    std::vector<VkQueueFamilyProperties> q_families(q_family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(
        device, &q_family_count, q_families.data());

    QueueFamilyIndices indices;
    int i = 0;
    for (const auto& q_family : q_families) {
      if (q_family.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
        indices.gfx_family = i;
      }

      VkBool32 present_support = false;
      vkGetPhysicalDeviceSurfaceSupportKHR(
          device, i, surface_, &present_support);
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

  bool checkDeviceExtensionSupport(VkPhysicalDevice device) {
    uint32_t ext_count = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &ext_count, nullptr);
    std::vector<VkExtensionProperties> extensions(ext_count);
    vkEnumerateDeviceExtensionProperties(
        device, nullptr, &ext_count, extensions.data());

    // Map to just the names.
    std::vector<std::string> available_exts(ext_count);
    std::transform(
        extensions.begin(), extensions.end(), available_exts.begin(),
        [](VkExtensionProperties ext) {
          return std::string{ext.extensionName};
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
    VkSurfaceCapabilitiesKHR caps;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> present_modes;
  };

  SwapchainSupportDetails querySwapChainSupport(VkPhysicalDevice device) {
    SwapchainSupportDetails details;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface_, &details.caps);
    uint32_t format_count = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(
        device, surface_, &format_count, nullptr);
    if (format_count) {
      details.formats.resize(format_count);
      vkGetPhysicalDeviceSurfaceFormatsKHR(
          device, surface_, &format_count, details.formats.data());
    }
    uint32_t present_mode_count = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(
        device, surface_, &present_mode_count, nullptr);
    if (present_mode_count) {
      details.present_modes.resize(present_mode_count);
      vkGetPhysicalDeviceSurfacePresentModesKHR(
          device, surface_, &present_mode_count, details.present_modes.data());
    }

#ifdef DEBUG
    printf("Supported formats (%d)\n", format_count);
    for (const auto& format : details.formats) {
      printf("  %d", format.format);
    }
    printf("\n");
#endif

    return details;
  }

  void createLogicalDevice() {
    std::vector<VkDeviceQueueCreateInfo> device_q_cis;
    std::set<int> unique_q_indices = {
        q_indices_.gfx_family, q_indices_.present_family};
    float q_prio = 1.0f;
    for (int q_index : unique_q_indices) {
      VkDeviceQueueCreateInfo device_q_ci{};
      device_q_ci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
      device_q_ci.queueFamilyIndex = q_index;
      device_q_ci.queueCount = 1;
      device_q_ci.pQueuePriorities = &q_prio;
      device_q_cis.push_back(device_q_ci);
    }

    VkPhysicalDeviceFeatures device_features{};

    VkDeviceCreateInfo device_ci{};
    device_ci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    device_ci.queueCreateInfoCount = device_q_cis.size();
    device_ci.pQueueCreateInfos = device_q_cis.data();
    device_ci.pEnabledFeatures = &device_features;
    device_ci.enabledExtensionCount = device_extensions.size();
    device_ci.ppEnabledExtensionNames = device_extensions.data();
    if (enable_validation_layers_) {
      device_ci.enabledLayerCount = validation_layers.size();
      device_ci.ppEnabledLayerNames = validation_layers.data();
    } else {
      device_ci.enabledLayerCount = 0;
    }

    VKASSERT(vkCreateDevice(physical_device_, &device_ci, nullptr, &device_));
    vkGetDeviceQueue(device_, q_indices_.gfx_family, 0, &gfx_q_);
    ASSERT(gfx_q_);
    vkGetDeviceQueue(device_, q_indices_.present_family, 0, &present_q_);
    ASSERT(present_q_);
  }

  VkSurfaceFormatKHR chooseSwapSurfaceFormat(
      const std::vector<VkSurfaceFormatKHR>& formats) {
    DASSERT(formats.size());
    for (const auto& format : formats) {
      if (format.format == VK_FORMAT_B8G8R8A8_SRGB &&
          format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
        return format;
      }
    }

    return formats[0];
  }

  VkPresentModeKHR chooseSwapPresentMode(
      const std::vector<VkPresentModeKHR>& present_modes) {
    if (std::find(
            present_modes.begin(), present_modes.end(),
            VK_PRESENT_MODE_MAILBOX_KHR) != present_modes.end()) {
      return VK_PRESENT_MODE_MAILBOX_KHR;
    }

    return VK_PRESENT_MODE_FIFO_KHR;
  }

  VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& caps) {
    if (caps.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
      return caps.currentExtent;
    } else {
      int width = 0;
      int height = 0;
      SDL_GL_GetDrawableSize(window_, &width, &height);
      VkExtent2D extent = {
          static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
      extent.width = std::clamp(
          extent.width, caps.minImageExtent.width, caps.maxImageExtent.width);
      extent.height = std::clamp(
          extent.height, caps.minImageExtent.height,
          caps.maxImageExtent.height);
      return extent;
    }
  }

  void createSwapChain() {
    auto format = chooseSwapSurfaceFormat(swapchain_support_.formats);
    auto present_mode = chooseSwapPresentMode(swapchain_support_.present_modes);
    auto extent = chooseSwapExtent(swapchain_support_.caps);

    uint32_t image_count = swapchain_support_.caps.minImageCount + 1;
    if (swapchain_support_.caps.maxImageCount > 0) {
      image_count =
          std::min(image_count, swapchain_support_.caps.maxImageCount);
    }

    VkSwapchainCreateInfoKHR swapchain_ci{};
    swapchain_ci.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    swapchain_ci.surface = surface_;
    swapchain_ci.minImageCount = image_count;
    swapchain_ci.imageFormat = format.format;
    swapchain_ci.imageColorSpace = format.colorSpace;
    swapchain_ci.imageExtent = extent;
    swapchain_ci.imageArrayLayers = 1;
    swapchain_ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    swapchain_ci.preTransform = swapchain_support_.caps.currentTransform;
    swapchain_ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapchain_ci.presentMode = present_mode;
    swapchain_ci.clipped = VK_TRUE;
    swapchain_ci.oldSwapchain = VK_NULL_HANDLE;

    uint32_t queue_family_indices[] = {
        static_cast<uint32_t>(q_indices_.gfx_family),
        static_cast<uint32_t>(q_indices_.present_family)};
    if (q_indices_.gfx_family != q_indices_.present_family) {
      swapchain_ci.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
      swapchain_ci.queueFamilyIndexCount = 2;
      swapchain_ci.pQueueFamilyIndices = queue_family_indices;
    } else {
      swapchain_ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
      swapchain_ci.queueFamilyIndexCount = 0;
      swapchain_ci.pQueueFamilyIndices = nullptr;
    }

    VKASSERT(
        vkCreateSwapchainKHR(device_, &swapchain_ci, nullptr, &swapchain_));

    vkGetSwapchainImagesKHR(device_, swapchain_, &image_count, nullptr);
    swapchain_images_.resize(image_count);
    vkGetSwapchainImagesKHR(
        device_, swapchain_, &image_count, swapchain_images_.data());

    swapchain_format_ = format.format;
    swapchain_extent_ = extent;
    printf(
        "Created %d swapchain images, format:%d extent:%dx%d\n", image_count,
        swapchain_format_, swapchain_extent_.width, swapchain_extent_.height);
  }

  void createImageViews() {
    swapchain_views_.resize(swapchain_images_.size());
    for (size_t i = 0; i < swapchain_images_.size(); i++) {
      VkImageViewCreateInfo ci{};
      ci.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      ci.image = swapchain_images_[i];
      ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
      ci.format = swapchain_format_;
      ci.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
      ci.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
      ci.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
      ci.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
      ci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      ci.subresourceRange.baseMipLevel = 0;
      ci.subresourceRange.levelCount = 1;
      ci.subresourceRange.baseArrayLayer = 0;
      ci.subresourceRange.layerCount = 1;
      VKASSERT(vkCreateImageView(device_, &ci, nullptr, &swapchain_views_[i]));
    }
  }

  void createRenderPass() {
    VkAttachmentDescription color_att{};
    color_att.format = swapchain_format_;
    color_att.samples = VK_SAMPLE_COUNT_1_BIT;
    color_att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color_att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color_att.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference color_att_ref{};
    color_att_ref.attachment = 0;
    color_att_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &color_att_ref;

    // Writing to the subpass color attachment depends on the swapchain
    // finishing its read of the color attachment.
    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dep.srcAccessMask = 0;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo rp_ci{};
    rp_ci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rp_ci.attachmentCount = 1;
    rp_ci.pAttachments = &color_att;
    rp_ci.subpassCount = 1;
    rp_ci.pSubpasses = &subpass;
    rp_ci.dependencyCount = 1;
    rp_ci.pDependencies = &dep;
    VKASSERT(vkCreateRenderPass(device_, &rp_ci, nullptr, &render_pass_));
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

    VkPipelineVertexInputStateCreateInfo vert_in_info{};
    vert_in_info.sType =
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vert_in_info.vertexBindingDescriptionCount = 0;
    vert_in_info.pVertexBindingDescriptions = nullptr;
    vert_in_info.vertexAttributeDescriptionCount = 0;
    vert_in_info.pVertexAttributeDescriptions = nullptr;

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
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

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

    VkPipelineLayoutCreateInfo pipeline_layout_ci{};
    pipeline_layout_ci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_ci.setLayoutCount = 0;
    pipeline_layout_ci.pushConstantRangeCount = 0;

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
    pipeline_ci.pDepthStencilState = nullptr;
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
      std::vector<VkImageView> atts = {swapchain_views_[i]};

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

  void createCommandBuffer() {
    VkCommandBufferAllocateInfo alloc_info{};
    alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    alloc_info.commandPool = cmd_pool_;
    alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc_info.commandBufferCount = 1;
    VKASSERT(vkAllocateCommandBuffers(device_, &alloc_info, &cmd_buffer_));
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
    VkClearValue clear_col = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
    rp_info.clearValueCount = 1;
    rp_info.pClearValues = &clear_col;
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

    vkCmdDraw(cmd_buf, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd_buf);
    VKASSERT(vkEndCommandBuffer(cmd_buf));
  }

  void createSyncObjects() {
    VkSemaphoreCreateInfo sem_ci{};
    sem_ci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    VKASSERT(vkCreateSemaphore(device_, &sem_ci, nullptr, &img_sem_));
    VKASSERT(vkCreateSemaphore(device_, &sem_ci, nullptr, &render_sem_));

    VkFenceCreateInfo fence_ci{};
    fence_ci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence_ci.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    VKASSERT(vkCreateFence(device_, &fence_ci, nullptr, &in_flight_fence_));
  }

  void cleanup() {
    vkDestroySemaphore(device_, img_sem_, nullptr);
    vkDestroySemaphore(device_, render_sem_, nullptr);
    vkDestroyFence(device_, in_flight_fence_, nullptr);
    vkDestroyCommandPool(device_, cmd_pool_, nullptr);
    for (auto fb : swapchain_fbs_) {
      vkDestroyFramebuffer(device_, fb, nullptr);
    }
    vkDestroyPipeline(device_, gfx_pipeline_, nullptr);
    vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
    vkDestroyRenderPass(device_, render_pass_, nullptr);
    for (auto image_view : swapchain_views_) {
      vkDestroyImageView(device_, image_view, nullptr);
    }
    vkDestroySwapchainKHR(device_, swapchain_, nullptr);
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

  VkInstance instance_;
  VkDebugUtilsMessengerEXT dbg_messenger_;
  VkSurfaceKHR surface_;
  VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
  // Indices of queue families for the selected |physical_device_|
  QueueFamilyIndices q_indices_;
  VkDevice device_;
  VkQueue gfx_q_ = VK_NULL_HANDLE;
  VkQueue present_q_ = VK_NULL_HANDLE;
  SwapchainSupportDetails swapchain_support_;
  VkSwapchainKHR swapchain_;
  std::vector<VkImage> swapchain_images_;
  VkFormat swapchain_format_;
  VkExtent2D swapchain_extent_;
  std::vector<VkImageView> swapchain_views_;
  VkRenderPass render_pass_;
  VkPipelineLayout pipeline_layout_;
  VkPipeline gfx_pipeline_;
  std::vector<VkFramebuffer> swapchain_fbs_;
  VkCommandPool cmd_pool_;
  VkCommandBuffer cmd_buffer_;
  VkSemaphore img_sem_;
  VkSemaphore render_sem_;
  VkFence in_flight_fence_;

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
