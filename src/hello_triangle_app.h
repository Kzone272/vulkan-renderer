#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <iostream>
#include <vector>

#include "asserts.h"
#include "defines.h"

using std::cerr;
using std::cout;
using std::endl;
using std::printf;

constexpr int WIDTH = 800;
constexpr int HEIGHT = 600;

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

    VkInstanceCreateInfo create_info{};
    create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    create_info.pApplicationInfo = &app_info;

    auto ext_names = getRequiredExtensions();
    create_info.enabledExtensionCount = ext_names.size();
    create_info.ppEnabledExtensionNames = ext_names.data();

    auto validation_layers = getValidationLayers();
    create_info.enabledLayerCount = validation_layers.size();
    create_info.ppEnabledLayerNames = validation_layers.data();

#if __APPLE__
    create_info.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

    VKASSERT(vkCreateInstance(&create_info, nullptr, &instance_));
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

    printf("Required extensions (%lu):\n", ext_names.size());
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
    std::vector<const char*> required_layers;
#ifdef DEBUG
    required_layers.push_back("VK_LAYER_KHRONOS_validation");
#endif

    if (required_layers.size()) {
      uint32_t layer_count = 0;
      VKASSERT(vkEnumerateInstanceLayerProperties(&layer_count, nullptr));
      std::vector<VkLayerProperties> layer_props(layer_count);
      VKASSERT(
          vkEnumerateInstanceLayerProperties(&layer_count, layer_props.data()));

      printf("Available layers:\n");
      for (const auto& layer_prop : layer_props) {
        printf("  %s\n", layer_prop.layerName);
      }

      for (const auto& layer : required_layers) {
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
    }

    return required_layers;
  }

  void cleanup() {
    vkDestroyInstance(instance_, nullptr);

    SDL_DestroyWindow(window_);
    SDL_Quit();
  }

  SDL_Window* window_ = nullptr;

  VkInstance instance_;
};
