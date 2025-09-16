#version 450

#include "structs.glsl"

struct ShadowData {
  mat4 viewProj;
};

layout(set = 0, binding = 0) uniform ShadowBlock {
  ShadowData shadow;
};
layout(set = 0, binding = 1) readonly buffer ObjectBlock {
  ObjectData objects[]; // Ordered by instance id
};
layout(set = 0, binding = 2) readonly buffer TransformBlock {
  mat4 models[];
};

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUv;

void main() {
  ObjectData obj = objects[gl_InstanceIndex];
  gl_Position = shadow.viewProj * models[obj.index] * vec4(inPosition, 1.0);
}
