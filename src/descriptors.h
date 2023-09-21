#pragma once

#include <memory>
#include <variant>
#include <vector>

#include "assert.h"
#include "render-objects.h"
#include "strings.h"
#include "vulkan-include.h"

struct Binding {
  vk::DescriptorType type;
  uint32_t count = 1;
};

struct DescLayout {
  std::vector<Binding> binds;
  vk::ShaderStageFlags stages;
  vk::UniqueDescriptorSetLayout layout;
};

void createDescLayout(vk::Device& device, DescLayout& layout) {
  std::vector<vk::DescriptorSetLayoutBinding> binds;

  uint32_t i = 0;
  for (auto& bind : layout.binds) {
    binds.push_back({
        .binding = i++,
        .descriptorType = bind.type,
        .descriptorCount = bind.count,
        .stageFlags = layout.stages,
    });
  }

  vk::DescriptorSetLayoutCreateInfo layout_ci{};
  layout_ci.setBindings(binds);
  layout.layout = device.createDescriptorSetLayoutUnique(layout_ci).value;
}

struct CombinedSamplerUpdate {
  vk::ImageView view;
  vk::Sampler sampler;
};
struct UboUpdate {
  vk::Buffer buffer;
  vk::DeviceSize offset = 0;
  vk::DeviceSize size;
};

void updateDescSet(
    const vk::Device& device, const vk::DescriptorSet& desc,
    const DescLayout& layout,
    std::vector<std::variant<CombinedSamplerUpdate, UboUpdate>>&& updates) {
  assert(layout.binds.size() == updates.size());

  std::vector<vk::WriteDescriptorSet> writes;
  std::vector<std::unique_ptr<vk::DescriptorImageInfo>> image_infos;
  std::vector<std::unique_ptr<vk::DescriptorBufferInfo>> buffer_infos;

  for (uint32_t i = 0; i < updates.size(); i++) {
    auto& update = updates[i];
    auto& bind = layout.binds[i];

    vk::WriteDescriptorSet write{
        .dstSet = desc,
        .dstBinding = i,
        .dstArrayElement = 0,
        .descriptorType = bind.type,
    };

    switch (bind.type) {
      case vk::DescriptorType::eCombinedImageSampler: {
        assert(std::holds_alternative<CombinedSamplerUpdate>(update));
        auto& sampler_update = std::get<CombinedSamplerUpdate>(update);
        std::unique_ptr<vk::DescriptorImageInfo> info(
            new vk::DescriptorImageInfo{
                .sampler = sampler_update.sampler,
                .imageView = sampler_update.view,
                .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
            });
        write.setImageInfo(*info.get());
        image_infos.push_back(std::move(info));
        break;
      }
      case vk::DescriptorType::eUniformBuffer: {
        assert(std::holds_alternative<UboUpdate>(update));
        auto& ubo_update = std::get<UboUpdate>(update);
        std::unique_ptr<vk::DescriptorBufferInfo> info(
            new vk::DescriptorBufferInfo{
                .buffer = ubo_update.buffer,
                .offset = ubo_update.offset,
                .range = ubo_update.size,
            });
        write.setBufferInfo(*info.get());
        buffer_infos.push_back(std::move(info));
        break;
      }
      default:
        log("Unsupported descriptor type! {}", (int)bind.type);
        assert(false);
    }

    writes.push_back(write);
  }

  device.updateDescriptorSets(writes, nullptr);
}
