#pragma once

#include <memory>
#include <vector>

#include "buffers.h"
#include "images.h"
#include "render-objects.h"
#include "vulkan-include.h"

struct VulkanState;

class Materials {
 public:
  void init(const VulkanState& vs);
  MaterialId loadMaterial(const VulkanState& vs, const MaterialInfo& mat_info);

  MaterialData& getMaterialData(MaterialId id);
  vk::DescriptorSet getDescSet(MaterialId id);

 private:
  Texture* createTexture(
      vk::CommandBuffer cmd_buf, const VulkanState& vs, void* texture_data,
      uint32_t width, uint32_t height);
  TextureId loadTexture(std::string path);
  TextureId getColorTexture(uint32_t color);

  DescLayout mat_layout_;
  DynamicBuf mat_buf_;

  std::vector<std::unique_ptr<Texture>> loaded_textures_;
  std::map<uint32_t, Texture*> color_textures_;
  std::vector<Texture*> refd_textures_;
  std::map<uint32_t, vk::DescriptorSet> texture_descs_;

  std::vector<MaterialData> material_datas_;
  std::vector<vk::DescriptorSet> material_descs_;
};
