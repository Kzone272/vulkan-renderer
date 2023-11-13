#pragma once

#include "buffers.h"
#include "render-objects.h"
#include "vulkan-include.h"

struct Texture;

struct Material {
  Texture* diffuse;
  Buffer ubo;
  vk::DescriptorSet desc_set;
};

struct Model {
  vk::UniqueBuffer vert_buf;
  vk::UniqueDeviceMemory vert_buf_mem;
  vk::UniqueBuffer ind_buf;
  vk::UniqueDeviceMemory ind_buf_mem;
  uint32_t index_count;
  uint32_t vertex_count;
  Material* material;
};

template <class T>
static vk::VertexInputBindingDescription getBindingDesc() {
  return {
      .binding = 0,
      .stride = sizeof(T),
      .inputRate = vk::VertexInputRate::eVertex,
  };
}

template <class T>
static std::vector<vk::VertexInputAttributeDescription> getAttrDescs();

template <>
static std::vector<vk::VertexInputAttributeDescription> getAttrDescs<Vertex>() {
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

template <>
static std::vector<vk::VertexInputAttributeDescription>
getAttrDescs<Vertex2d>() {
  std::vector<vk::VertexInputAttributeDescription> attrs = {
      {
          .location = 0,
          .binding = 0,
          .format = vk::Format::eR32G32Sfloat,  // vec2
          .offset = offsetof(Vertex2d, pos),
      },
      {
          .location = 1,
          .binding = 0,
          .format = vk::Format::eR32G32B32Sfloat,  // vec3
          .offset = offsetof(Vertex2d, color),
      },
  };
  return attrs;
}
