#version 450

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform sampler2DMS image;

layout(location = 0) out vec4 outColor;

void main() {
  const int nSamples = textureSamples(image);
  const vec2 size = textureSize(image);
  const ivec2 uv = ivec2(size * fragUv);

  vec4 color = vec4(0);
  for (int i = 0; i < nSamples; i++) {
    color += texelFetch(image, uv, i);
  }
  outColor = color / nSamples;
}
