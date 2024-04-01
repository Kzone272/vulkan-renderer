#version 450

layout(push_constant) uniform PushBlock {
  float tweak;
};

layout(set = 0, binding = 0) uniform sampler2D image;

layout(location = 0) in vec2 fragUv;
layout(location = 0) out vec2 outColor;

// https://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
float rand(vec2 pos){
  return fract(sin(dot(pos, vec2(12.9898, 78.233))) * 43758.5453);
}

void main() {
  ivec2 size = textureSize(image, 0);
  vec2 samp = texture(image, fragUv).xy;
  if (samp == vec2(-1.0)) {
    outColor = vec2(-1.0);
  } else {
    float dist = length(vec2(size) * (samp - fragUv)) / length(vec2(size));
    float minProb = .0001;
    float maxProb = 0.02;
    float x = clamp(maxProb - (0.01 * pow(dist * 100.0, 0.8)), minProb, maxProb);
    vec2 param = fragUv;
    double prob = rand(param);
    if (double(x) > prob) {
      outColor = fragUv;
    } else {
      outColor = vec2(-1.0);
    }
  }
}
