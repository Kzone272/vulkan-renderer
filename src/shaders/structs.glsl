// Common structs used across shaders.

struct Light {
  vec4 vec;
  vec4 color;
  uint type;
  float pad1;
  float pad2;
  float pad3;
};
const uint kDirectionalLightType = 1;
const uint kPointLightType = 2;

struct GlobalData {
  mat4 view;
  mat4 proj;
  mat4 inv_proj;
  uint width;
  uint height;
  float near;
  float far;
  Light lights[8];
};

struct DebugData {
  float f1, f2, f3, f4;
  int i1, i2, i3, i4;
};

struct MaterialData {
  vec3 color1;
  float pad1;
  vec3 color2;
  float pad2;
  uint type;
  uint pad3;
  uint pad4;
  uint pad5;
};
const uint kPhongMaterial = 0;
const uint kGoochMaterial = 1;
const uint kToonMaterial = 2;

struct ObjectData {
  uint index;
  uint matIndex;
  uint pad2;
  uint pad3;
};
