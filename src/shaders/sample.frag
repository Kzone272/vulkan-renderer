#version 450

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform sampler2D image;

layout(location = 0) out vec4 outColor;

void main() {
  vec4 color = texture(image, fragUv);

  outColor = vec4(color.rgb, 1.0);
}
