#version 450

layout(location = 0) in vec2 fragUv;

// TODO: Move this into shared glsl code.
layout(set = 0, binding = 0) uniform DebugData {
  float f1, f2, f3, f4;
  int i1, i2, i3, i4;
} debug;

layout(location = 0) out vec4 outColor;

void main() {
  vec3 col = vec3(0);
  if (length(fragUv - vec2(0.5)) < debug.f2) {
    col = vec3(1,0,0);
  }

  outColor = vec4(col, 1.0);
}
