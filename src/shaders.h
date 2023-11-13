#pragma once

#include <map>
#include <set>

#include "vulkan-include.h"

const std::set<std::string> kShaders{
    // clang-format off
    "scene.vert.spv",
    "scene.frag.spv",
    "fullscreen.vert.spv",
    "post.frag.spv",
    "circle.frag.spv",
    "sample.frag.spv",
    "voronoi.vert.spv",
    "voronoi.frag.spv",
    // clang-format on
};

struct VulkanState;

class Shaders {
 public:
  Shaders() = default;
  // Move only.
  Shaders(const Shaders& shaders) = delete;
  Shaders& operator=(const Shaders& shaders) = delete;

  void create(const VulkanState& vs);
  vk::ShaderModule get(std::string filename) const;
  void clear();

 private:
  vk::UniqueShaderModule createShaderModule(
      const VulkanState& vs, std::string filename);

  std::string shader_dir_ = "shaders";
  std::map<std::string, vk::UniqueShaderModule> shaders_;
};
