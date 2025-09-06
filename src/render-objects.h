#pragma once

#include <optional>

#include "glm-include.h"

struct Light {
  enum class Type : uint32_t {
    None,
    Directional,
    Point,
  };
  vec4 vec = vec4(0);    // Direction, or position.
  vec4 color = vec4(1);  // Falloff in the w component.
  Type type = Type::None;
  // Pad size to nearest 16 bytes, because the next light's vec4 will need
  // 16-byte alignment anyway.
  float pad1 = 0;
  float pad2 = 0;
  float pad3 = 0;
};

struct GlobalData {
  alignas(16) mat4 view;
  alignas(16) mat4 proj;
  alignas(16) mat4 inv_proj;
  uint32_t width;
  uint32_t height;
  float near;
  float far;
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
  float f1 = 0;
  float f2 = 0;
  float f3 = 0;
  float f4 = 0;
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
  BallControl,
  BoxControl,
  Tetra,
  Floor,
  Sphere,
};

static constexpr size_t kMaxMaterials = 1024;  // arbitrary big number

struct MaterialData {
  enum class Type : uint32_t {
    Phong,
    Gooch,
    Toon,
  };
  vec3 color1 = {1, 1, 1};  // This takes up the size of a vec4
  vec3 color2 = {0, 0, 0};  // This takes up the size of a vec4
  vec4 data3 = {0, 0, 0, 0};
  uint32_t pad3 = 0;
  uint32_t pad4 = 0;
  uint32_t pad5 = 0;
  Type type = Type::Phong;
};

typedef uint32_t TextureId;
inline const uint32_t kTextureIdNone = -1;

enum class ScenePipeline : uint32_t {
  Basic,
  Gradient,
};

struct MaterialInfo {
  TextureId diffuse_texture = kTextureIdNone;
  std::optional<std::string> diffuse_path;
  MaterialData data;
  ScenePipeline pipeline = ScenePipeline::Basic;
};
typedef uint32_t MaterialId;
inline const uint32_t kMaterialIdNone = -1;

struct ModelInfo {
  std::string obj_path;
  std::string texture_path;
  mat4 model_transform{1};
  MaterialId material = kMaterialIdNone;
};

struct DrawData {
  ModelId model = ModelId::None;
  MaterialId material = kMaterialIdNone;
  uint32_t objInd = (uint32_t)-1;
};

static constexpr size_t kMaxObjects = 64 * 1024;  // arbitrary big number

struct ObjectData {
  uint32_t index = -1;
  uint32_t matIndex = kMaterialIdNone;
  uint32_t pad2 = 0;
  uint32_t pad3 = 0;
};

struct Vertex2d {
  vec2 pos = {0, 0};
  vec3 color = {0, 0, 0};
};

enum class DebugView {
  None,
  Normals,
  Depth,
};

struct Draw2d {
  enum class Type : uint32_t {
    Line,
    Circle,
  };
  vec2 pos1{0};
  vec2 pos2{0};
  vec4 color{1};
  vec4 values{0};
  Type type = Type::Line;
};

struct FrameState {
  uint64_t frame_num = 0;
  bool drawsUpdated = true;
  std::vector<DrawData> draws;
  std::vector<mat4> transforms;
  std::vector<Light> lights;
  mat4 model;
  mat4 view;
  mat4 proj;
  mat4 viewProj;
  uint32_t width;
  uint32_t height;
  float near;
  float far;
  DebugData drawing{
      .f1 = 0.5,
  };
  DebugView debug_view = DebugView::None;
  bool draw_edges = true;
  bool stained_glass = false;
  float edge_w = 1.25;
  float v_tweak = 1.12;
  DebugData edges{
      .f3 = 1,
      .f4 = 15,
  };
  bool update_drawing = false;
  std::vector<Vertex2d> voronoi_cells;
  bool update_voronoi = false;
  std::vector<Draw2d> draws2d;
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
