#include "shaders.h"

#include <filesystem>

#include "files.h"
#include "render-state.h"

void Shaders::create(const VulkanState& vs) {
  for (auto& shader : kShaders) {
    shaders_.emplace(shader, createShaderModule(vs, shader));
  }
}

vk::ShaderModule Shaders::get(std::string filename) {
  ASSERT(kShaders.contains(filename));
  return *shaders_.find(filename)->second;
}

void Shaders::clear() {
  shaders_.clear();
}

vk::UniqueShaderModule Shaders::createShaderModule(
    const VulkanState& vs, std::string filename) {
  std::filesystem::path fullpath = shader_dir_;
  fullpath /= filename;
  auto code = readFile(fullpath.string());

  vk::ShaderModuleCreateInfo ci{
      .codeSize = code.size(),
      .pCode = reinterpret_cast<const uint32_t*>(code.data()),
  };
  return vs.device.createShaderModuleUnique(ci).value;
}
