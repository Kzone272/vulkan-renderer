#version 450

layout(location = 0) in vec2 fragUv;
layout(set = 0, binding = 0) uniform sampler2D image;
layout(push_constant) uniform PushBlock {
  int step_size;
};

layout(location = 0) out vec2 outColor;

void main() {
  ivec2 size = textureSize(image, 0);
  ivec2 iUv = ivec2(size * fragUv);

  vec2 minSample = vec2(-1, -1);
  float minD2 = 1000000000;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      ivec2 nbPos = iUv + step_size * ivec2(i, j);
      vec2 samp = texelFetch(image, nbPos, 0).rg;
      vec2 delta = size * (fragUv - samp);
      float d2 = dot(delta, delta);
      if (d2 < minD2) {
        minD2 = d2;
        minSample = samp;
      }
    }
  }
  outColor = minSample;
}
