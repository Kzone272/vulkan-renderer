#pragma once

#include <map>
#include <optional>

#include "glm-include.h"
#include "object.h"
#include "vulkan-include.h"

struct Light {
  enum class Type {
    None,
    Directional,
    Point,
  };
  Type type = Type::None;
  alignas(16) vec3 vec{0, 0, 0};  // Direction, or position.
  alignas(16) vec3 color{1, 1, 1};
  float falloff = 0;
};

struct UniformBufferData {
  alignas(16) mat4 proj_view;
  alignas(16) Light lights[8];
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
};

struct Material {
  Texture* diffuse;
  vk::UniqueBuffer ubo_buf;
  vk::UniqueDeviceMemory ubo_buf_mem;
  vk::DescriptorSet desc_set;
};

struct Model {
  vk::UniqueBuffer vert_buf;
  vk::UniqueDeviceMemory vert_buf_mem;
  vk::UniqueBuffer ind_buf;
  vk::UniqueDeviceMemory ind_buf_mem;
  uint32_t index_count;
  Material* material;
};

struct Vertex {
  vec3 pos = {0, 0, 0};
  vec3 normal = {0, 1, 0};
  vec3 color = {1, 1, 1};
  vec2 uv = {0, 0};

  bool operator==(const Vertex& other) const {
    return pos == other.pos && normal == other.normal && color == other.color &&
           uv == other.uv;
  }

  static vk::VertexInputBindingDescription getBindingDesc() {
    return {
        .binding = 0,
        .stride = sizeof(Vertex),
        .inputRate = vk::VertexInputRate::eVertex,
    };
  }

  static std::vector<vk::VertexInputAttributeDescription> getAttrDescs() {
    std::vector<vk::VertexInputAttributeDescription> attrs = {
        {
            .location = 0,
            .binding = 0,
            .format = vk::Format::eR32G32B32Sfloat,  // vec3
            .offset = offsetof(Vertex, pos),
        },
        {
            .location = 1,
            .binding = 0,
            .format = vk::Format::eR32G32B32Sfloat,  // vec3
            .offset = offsetof(Vertex, normal),
        },
        {
            .location = 2,
            .binding = 0,
            .format = vk::Format::eR32G32B32Sfloat,  // vec3
            .offset = offsetof(Vertex, color),
        },
        {
            .location = 3,
            .binding = 0,
            .format = vk::Format::eR32G32Sfloat,  // vec2
            .offset = offsetof(Vertex, uv),
        },
    };

    return attrs;
  }
};

struct Mesh {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
};

namespace std {

// TODO: Hash this more generally.
template <>
struct hash<Vertex> {
  size_t operator()(Vertex const& vertex) const {
    return ((hash<vec3>()(vertex.pos) ^ (hash<vec3>()(vertex.normal) << 1)) >>
            1) ^
           (hash<vec2>()(vertex.uv) << 1);
  }
};

}  // namespace std
