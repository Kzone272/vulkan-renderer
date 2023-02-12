#pragma once

#include <SDL.h>
#undef main  // SDL needs this on Windows
#include <SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <vector>

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

    uint32_t ext_count = 0;
    ASSERT(SDL_Vulkan_GetInstanceExtensions(nullptr, &ext_count, nullptr));
    std::vector<const char*> ext_names(ext_count);
    ASSERT(SDL_Vulkan_GetInstanceExtensions(nullptr, &ext_count,
                                            ext_names.data()));

    printf("SDL requires %d extensions:\n", ext_count);
    for (auto& name : ext_names) {
      printf("  %s\n", name);
    }

    create_info.enabledExtensionCount = ext_count;
    create_info.ppEnabledExtensionNames = ext_names.data();
    create_info.enabledLayerCount = 0;

    VKASSERT(vkCreateInstance(&create_info, nullptr, &instance_));
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

  void cleanup() {
    vkDestroyInstance(instance_, nullptr);

    SDL_DestroyWindow(window_);
    SDL_Quit();
  }

  SDL_Window* window_ = nullptr;

  VkInstance instance_;
};
