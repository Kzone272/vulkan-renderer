#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

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

static VKAPI_ATTR VkBool32 VKAPI_CALL
debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT msg_severity,
              VkDebugUtilsMessageTypeFlagsEXT msg_type,
              const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
              void* user_data) {
  printf("validation layer: %s\n", callback_data->pMessage);
  return VK_FALSE;
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
    window_ =
        SDL_CreateWindow("Vulkan Tutorial", SDL_WINDOWPOS_CENTERED,
                         SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, window_flags);
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
  }

  void mainLoop() {
    SDL_Event event;
    while (true) {
      SDL_PollEvent(&event);
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE) {
        break;
      }
    }
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
    ASSERT(SDL_Vulkan_GetInstanceExtensions(nullptr, &ext_count,
                                            ext_names.data()));

#if __APPLE__
    ext_names.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
#endif

    if (enable_validation_layers_) {
      ext_names.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    printf("Required extensions (%zu):\n", ext_names.size());
    for (auto& name : ext_names) {
      printf("  %s\n", name);
    }

    return ext_names;
  }

  void printSupportedExtensions() {
    uint32_t sup_ext_count = 0;
    VKASSERT(vkEnumerateInstanceExtensionProperties(nullptr, &sup_ext_count,
                                                    nullptr));
    std::vector<VkExtensionProperties> sup_exts(sup_ext_count);
    VKASSERT(vkEnumerateInstanceExtensionProperties(nullptr, &sup_ext_count,
                                                    sup_exts.data()));
    printf("Vulkan supports %d extensions\n", sup_ext_count);
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

    q_indices_ = indices;
    return true;
  }

  QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) {
    uint32_t q_family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &q_family_count, nullptr);
    std::vector<VkQueueFamilyProperties> q_families(q_family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &q_family_count,
                                             q_families.data());

    QueueFamilyIndices indices;
    int i = 0;
    for (const auto& q_family : q_families) {
      if (q_family.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
        indices.gfx_family = i;
      }

      VkBool32 present_support = false;
      vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface_,
                                           &present_support);
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

  void createLogicalDevice() {
    std::vector<VkDeviceQueueCreateInfo> device_q_cis;
    std::set<int> unique_q_indices = {q_indices_.gfx_family,
                                      q_indices_.present_family};
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
    device_ci.enabledExtensionCount = 0;
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

  void cleanup() {
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

#ifdef DEBUG
  const bool enable_validation_layers_ = true;
#else
  const bool enable_validation_layers_ = false;
#endif  // DEBUG
};
