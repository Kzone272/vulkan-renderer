#pragma once

#include <map>

#include "glm-include.h"
#include "vulkan-include.h"
#include "world.h"

struct UniformBufferData {
  alignas(16) glm::mat4 proj_view;
};

struct PushData {
  mat4 model;
};

struct Texture {
  vk::UniqueImage image;
  vk::UniqueDeviceMemory image_mem;
  vk::UniqueImageView image_view;
  vk::Format format;
  uint32_t mip_levels = 1;
  vk::DescriptorSet desc_set;
};

struct Model {
  vk::UniqueBuffer vert_buf;
  vk::UniqueDeviceMemory vert_buf_mem;
  vk::UniqueBuffer ind_buf;
  vk::UniqueDeviceMemory ind_buf_mem;
  uint32_t index_count;
  std::unique_ptr<Texture> texture;
};

struct Vertex {
  vec3 pos;
  vec3 color;
  vec2 uv;

  bool operator==(const Vertex& other) const {
    return pos == other.pos && color == other.color && uv == other.uv;
  }

  static vk::VertexInputBindingDescription getBindingDesc() {
    return {
        .binding = 0,
        .stride = sizeof(Vertex),
        .inputRate = vk::VertexInputRate::eVertex,
    };
  }

  static std::array<vk::VertexInputAttributeDescription, 3> getAttrDescs() {
    std::array<vk::VertexInputAttributeDescription, 3> attrs{};
    attrs[0] = {
        .location = 0,
        .binding = 0,
        .format = vk::Format::eR32G32B32Sfloat,  // vec3
        .offset = offsetof(Vertex, pos),
    };
    attrs[1] = {
        .location = 1,
        .binding = 0,
        .format = vk::Format::eR32G32B32Sfloat,  // vec3
        .offset = offsetof(Vertex, color),
    };
    attrs[2] = {
        .location = 2,
        .binding = 0,
        .format = vk::Format::eR32G32Sfloat,  // vec2
        .offset = offsetof(Vertex, uv),
    };

    return attrs;
  }
};

namespace std {

template <>
struct hash<Vertex> {
  size_t operator()(Vertex const& vertex) const {
    return ((hash<vec3>()(vertex.pos) ^ (hash<vec3>()(vertex.color) << 1)) >>
            1) ^
           (hash<vec2>()(vertex.uv) << 1);
  }
};

}  // namespace std
