#pragma once

#include <memory>
#include <vector>

#include "buffers.h"
#include "descriptors.h"
#include "images.h"
#include "render-objects.h"
#include "vulkan-include.h"

struct VulkanState;
struct DrawState;

class Materials {
 public:
  void init(const VulkanState& vs);
  void update(const DrawState& ds);

  DescLayout* layout() {
    return &mat_layout_;
  }
  vk::DescriptorBufferInfo* bufferInfo() {
    return &mat_buf_.device.info;
  }

  MaterialId loadMaterial(const VulkanState& vs, const MaterialInfo& mat_info);
  vk::DescriptorSet getDesc(MaterialId id);
  TextureId getTextureId(Texture* texture);

 private:
  TextureId loadTexture(const VulkanState& vs, std::string_view path);
  TextureId getColorTexture(const VulkanState& vs, uint32_t color);
  Texture* createTexture(
      const VulkanState& vs, void* texture_data, uint32_t width,
      uint32_t height);

  DescLayout mat_layout_;
  DynamicBuf mat_buf_;

  std::vector<std::unique_ptr<Texture>> loaded_textures_;
  std::map<uint32_t, Texture*> color_textures_;
  std::vector<Texture*> refd_textures_;
  std::map<uint32_t, vk::DescriptorSet> texture_descs_;

  std::vector<MaterialData> material_datas_;
  std::vector<vk::DescriptorSet> material_descs_;
};
