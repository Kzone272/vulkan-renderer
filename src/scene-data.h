#pragma once

#include "buffers.h"
#include "render-objects.h"
#include "vulkan-include.h"

struct Texture;

struct Material {
  Texture* diffuse = nullptr;
  Buffer ubo;
  vk::DescriptorSet desc_set;
};

struct Model {
  Buffer vert_buf;
  Buffer ind_buf;
  uint32_t index_count = 0;
  uint32_t vertex_count = 0;
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
