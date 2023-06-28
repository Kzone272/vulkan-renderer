#version 450

layout(binding = 0) uniform UniformBufferObject {
  mat4 proj_view;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inUv;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragUv;

layout(push_constant) uniform PushData {
	mat4 model;
} push;

void main() {
  gl_Position = ubo.proj_view * push.model * vec4(inPosition, 1.0);
  fragColor = inColor;
  fragUv = inUv;
}
