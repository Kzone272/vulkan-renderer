#version 450

#include "structs.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
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

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec2 fragUv;
layout(location = 3) out vec3 fragPos;
layout(location = 4) flat out uint matIndex;
layout(location = 5) out vec4 shadowPos;

void main() {
  ObjectData obj = objects[gl_InstanceIndex];
  mat4 model = models[obj.index];
  vec4 worldPos = model * vec4(inPosition, 1.0);
  vec4 viewPos = global.view * worldPos;
  
  fragPos = viewPos.xyz;
  gl_Position = global.proj * viewPos;
  shadowPos = global.shadowViewProj * worldPos;

  fragNormal = vec3(global.view * model * vec4(inNormal, 0));
  fragColor = inColor;
  fragUv = inUv;
  matIndex = obj.matIndex;
}
