#version 450

#include "structs.glsl"

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform DebugBlock {
  DebugData debug;
};

layout(location = 0) out vec4 outColor;

void main() {
  vec3 col = vec3(0);
  if (length(fragUv - vec2(0.5)) < debug.f1) {
    col = vec3(1,0,0);
  }

  outColor = vec4(col, 1.0);
}
