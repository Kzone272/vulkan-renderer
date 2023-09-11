#version 450

struct Light {
  uint type;
  vec3 vec;
  vec3 color;
  float falloff;
};

layout(set = 0, binding = 0) uniform UniformBufferObject {
  mat4 proj_view;
  Light lights[8];
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUv;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec2 fragUv;
layout(location = 3) out vec3 fragPos;

layout(push_constant) uniform PushData {
	mat4 model;
} push;

void main() {
  fragPos = vec3(push.model * vec4(inPosition, 1.0));
  gl_Position = ubo.proj_view * push.model * vec4(inPosition, 1.0);
  fragNormal = vec3(push.model * vec4(inNormal, 0));
  fragColor = inColor;
  fragUv = inUv;
}
