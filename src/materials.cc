#include "materials.h"

#include "descriptors.h"
#include "hash.h"

void Materials::init(const VulkanState& vs) {
  DescLayout mat_layout_{
      .binds = {{.type = vk::DescriptorType::eCombinedImageSampler}},
      .stages = vk::ShaderStageFlagBits::eFragment,
  };

  mat_buf_ = createDynamicBuffer(
      vs, kMaxMaterials * sizeof(MaterialData),
      vk::BufferUsageFlagBits::eStorageBuffer);
}

MaterialId Materials::loadMaterial(
    const VulkanState& vs, const MaterialInfo& mat_info) {
  TextureId diffuse;
  if (mat_info.diffuse_texture != kTextureIdNone) {
    diffuse = mat_info.diffuse_texture;
  } else if (mat_info.diffuse_path) {
    diffuse = loadTexture(*mat_info.diffuse_path);
  } else {
    // Use 1x1 pixel white texture when none is specified.
    diffuse = getColorTexture(0xFFFFFFFF);
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

Texture* Materials::createTexture(
    vk::CommandBuffer cmd_buf, const VulkanState& vs, void* texture_data,
    uint32_t width, uint32_t height) {
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

  auto* ptr = texture.get();
  loaded_textures_.push_back(std::move(texture));
  return ptr;
}
