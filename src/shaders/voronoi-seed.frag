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

// Hash functions from: https://www.shadertoy.com/view/ldB3zc
float hash1( float n ) {
  return fract(sin(n)*43758.5453);
}
vec2 hash2( vec2  p ) {
  p = vec2( dot(p,vec2(127.1,311.7)), dot(p,vec2(269.5,183.3)) );
  return fract(sin(p)*43758.5453);
}

void main() {
  ivec2 size = textureSize(image, 0);
  vec2 samp = texture(image, fragUv).xy;
  if (samp == vec2(-1.0)) {
    outColor = vec2(-1.0);
  } else {
    float dist = length(vec2(size) * (samp - fragUv)) / length(vec2(size));
    const vec2 block = vec2(2);
    vec2 scaled = fragUv * size / block;
    vec2 within = fract(scaled);
    vec2 point = hash2(floor(scaled));

    if (length(within - point) < 0.1 && pow(dist, 0.1) * tweak < rand(point)) {
      outColor = fragUv;
    } else {
      outColor = vec2(-1.0);
    }
  }
}
