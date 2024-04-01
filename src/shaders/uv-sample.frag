#version 450

layout(set = 0, binding = 0) uniform sampler2D uvImage;
layout(set = 1, binding = 0) uniform sampler2D image;

layout(location = 0) in vec2 fragUv;
layout(location = 0) out vec4 outColor;

void main() {
  vec2 uv = texture(uvImage, fragUv).xy;
  vec2 size = vec2(textureSize(uvImage, 0));
  vec2 aspect = size / vec2(max(size.x, size.y));
  vec2 delta = fragUv - uv;
  float dist = length(aspect * delta);
  float scale = 0.2 * pow(dist / 20.0, 4.0);
  outColor = texture(image, uv + (delta * scale));
}
