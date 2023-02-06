#pragma once

#include <vulkan/vulkan.h>

class HelloTriangleApp {
 public:
  void run() {
    initVulkan();
    mainLoop();
    cleanup();
  }

 private:
  void initVulkan() {}

  void mainLoop() {}

  void cleanup() {}
};
