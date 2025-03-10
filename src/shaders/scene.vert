#version 450

#include "structs.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 0, binding = 1) readonly buffer ObjectBlock {
  ObjectData object[];
};

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUv;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec2 fragUv;
layout(location = 3) out vec3 fragPos;

void main() {
  mat4 to_view = global.view * object[gl_InstanceIndex].model;
  fragPos = vec3(to_view * vec4(inPosition, 1.0));
  gl_Position = global.proj * vec4(fragPos, 1.0);
  fragNormal = vec3(to_view * vec4(inNormal, 0));
  fragColor = inColor;
  fragUv = inUv;
}
