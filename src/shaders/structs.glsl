struct Light {
  uint type;
  vec3 vec;
  vec3 color;
  float falloff;
};
const uint kDirectionalLightType = 1;
const uint kPointLightType = 2;

struct GlobalData {
  mat4 view;
  mat4 proj;
  mat4 inv_proj;
  Light lights[8];
};

struct DebugData {
  float f1, f2, f3, f4;
  int i1, i2, i3, i4;
};

struct Material {
  vec3 color;
};
