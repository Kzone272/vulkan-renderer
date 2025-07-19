#include "materials.h"

#undef main  // SDL needs this on Windows
#include <SDL_image.h>

#include "command-buffers.h"
#include "hash.h"
#include "render-state.h"

void Materials::init(const VulkanState& vs) {
  mat_layout_ = {
      .binds = {{.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };
  mat_layout_.init(vs);

  mat_buf_ = createDynamicBuffer(
      vs, kMaxMaterials * sizeof(MaterialData),
      vk::BufferUsageFlagBits::eStorageBuffer);
}

void Materials::update(const DrawState& ds) {
  updateDynamicBuf(
      ds, mat_buf_, std::span(material_datas_),
      vk::PipelineStageFlagBits::eFragmentShader,
      vk::AccessFlagBits::eShaderRead);
}

MaterialId Materials::loadMaterial(
    const VulkanState& vs, const MaterialInfo& mat_info) {
  TextureId diffuse;
  if (mat_info.diffuse_texture != kTextureIdNone) {
    diffuse = mat_info.diffuse_texture;
  } else if (mat_info.diffuse_path) {
    diffuse = loadTexture(vs, *mat_info.diffuse_path);
  } else {
    // Use 1x1 pixel white texture when none is specified.
    diffuse = getColorTexture(vs, 0xFFFFFFFF);
  }

  TextureId textureIds[] = {diffuse};
  uint32_t textureHash = hashBytes(std::span(textureIds));

  auto desc_it = texture_descs_.find(textureHash);
  vk::DescriptorSet mat_desc;
  if (desc_it != texture_descs_.end()) {
    mat_desc = desc_it->second;
  } else {
    mat_desc = allocDescSet(vs, *mat_layout_.layout);
    std::vector<vk::WriteDescriptorSet> writes;
    updateDescSet(
        mat_desc, mat_layout_, {&(refd_textures_[diffuse]->info)}, writes);
    vs.device.updateDescriptorSets(writes, nullptr);

    texture_descs_.emplace(textureHash, mat_desc);
  }

  MaterialId id = material_datas_.size();
  material_descs_.emplace_back(mat_desc);
  material_datas_.emplace_back(mat_info.data);

  return id;
}

vk::DescriptorSet Materials::getDesc(MaterialId id) {
  DASSERT(id >= 0 && id < material_descs_.size());
  return material_descs_[id];
}

TextureId Materials::getTextureId(Texture* texture) {
  if (texture->id == kTextureIdNone) {
    texture->id = (TextureId)refd_textures_.size();
    refd_textures_.push_back(texture);
  }
  return texture->id;
}

TextureId Materials::loadTexture(const VulkanState& vs, std::string_view path) {
  auto* texture_surface = loadImage(path);
  Texture* texture = createTexture(
      vs, texture_surface->pixels, texture_surface->w, texture_surface->h);
  SDL_FreeSurface(texture_surface);

  return getTextureId(texture);
}

TextureId Materials::getColorTexture(const VulkanState& vs, uint32_t color) {
  auto it = color_textures_.find(color);
  if (it != color_textures_.end()) {
    return getTextureId(it->second);
  }

  // Create 1x1 color texture.
  auto* texture = createTexture(vs, &color, 1, 1);
  color_textures_.emplace(color, texture);

  return getTextureId(texture);
}

Texture* Materials::createTexture(
    const VulkanState& vs, void* texture_data, uint32_t width,
    uint32_t height) {
  auto texture = std::make_unique<Texture>();
  texture->size = {width, height};
  texture->format = vk::Format::eB8G8R8A8Srgb;
  texture->mip_levels = std::floor(std::log2(std::max(width, height))) + 1;

  vk::DeviceSize image_size = width * height * 4;
  auto staging = createStagingBuffer(vs, image_size);

  vs.vma.copyMemoryToAllocation(texture_data, *staging.alloc, 0, image_size);

  createImage(
      vs, *texture.get(), vk::ImageTiling::eOptimal,
      vk::ImageUsageFlagBits::eTransferSrc |
          vk::ImageUsageFlagBits::eTransferDst |
          vk::ImageUsageFlagBits::eSampled,
      vk::MemoryPropertyFlagBits::eDeviceLocal, vk::ImageAspectFlagBits::eColor,
      vs.linear_sampler);

  auto cmd_buf = beginSingleTimeCommands(vs);

  transitionImageLayout(
      cmd_buf, *texture->image, texture->format, texture->mip_levels,
      vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal);
  copyBufferToImage(cmd_buf, *staging.buf, *texture->image, width, height);

  vk::FormatProperties format_props =
      vs.physical_device.getFormatProperties(texture->format);
  bool generate_mips =
      (bool)(format_props.optimalTilingFeatures &
             vk::FormatFeatureFlagBits::eSampledImageFilterLinear);
  if (generate_mips) {
    generateMipmaps(
        cmd_buf, *texture->image, width, height, texture->mip_levels);
    // Transitioned to vk::ImageLayout::eShaderReadOnlyOptimal while generating
    // mipmaps.
  } else {
    transitionImageLayout(
        cmd_buf, *texture->image, texture->format, texture->mip_levels,
        vk::ImageLayout::eTransferDstOptimal,
        vk::ImageLayout::eShaderReadOnlyOptimal);
  }

  endSingleTimeCommands(vs, cmd_buf);

  auto* ptr = texture.get();
  loaded_textures_.push_back(std::move(texture));
  return ptr;
}
