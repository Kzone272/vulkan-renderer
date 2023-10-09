#pragma once

#include <map>
#include <optional>

#include "glm-include.h"

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

struct GlobalData {
  alignas(16) mat4 view;
  alignas(16) mat4 proj;
  alignas(16) Light lights[8];
};

struct PushData {
  mat4 model;
};

union BoolsInt {
  struct {
    bool b1 = false;
    bool b2 = false;
    bool b3 = false;
    bool b4 = false;
  } b;
  uint32_t i = 0;
};

// General data than can be tied to ImGui controls.
struct DebugData {
  float f1 = 1.5;
  float f2 = 0.5;
  float f3 = 0.01;
  float f4 = 0.5;
  BoolsInt i1 = {};
  BoolsInt i2 = {};
  BoolsInt i3 = {};
  BoolsInt i4 = {};
};

enum class ModelId {
  None,
  Viking,
  Pony,
  Cube,
  Bone,
  Control,
  Tetra,
  Floor,
};

struct ModelInfo {
  std::string obj_path;
  std::string texture_path;
  mat4 model_transform{1};
};

struct Texture;
struct MaterialInfo {
  // TODO: Make this a TextureId
  Texture* diffuse_texture;
  std::optional<std::string> diffuse_path;
  struct UniformBufferObject {
    vec3 color = {1, 1, 1};
  } ubo;
};
typedef uint32_t MaterialId;

struct RenderObject {
  ModelId model;
  mat4 transform;
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
