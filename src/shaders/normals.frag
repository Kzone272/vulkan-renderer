#version 450

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform sampler2DMS image;

layout(location = 0) out vec4 outColor;

void main() {
  int nSamples = textureSamples(image);
  ivec2 iUv = ivec2(textureSize(image) * fragUv);

  vec3 color = vec3(0);
  for (int i = 0; i < nSamples; i++) {
    color += texelFetch(image, iUv, i).rgb;
  }
  color /= nSamples;
  outColor = vec4(color * vec3(1, 1, -1), 1);
}
