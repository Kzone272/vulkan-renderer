#version 450

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform PostFxData {
  float v1;
  float v2;
  float v3;
  float v4;
  bool b1;
  bool b2;
  bool b3;
  bool b4;
} post;
layout(set = 0, binding = 1) uniform sampler2D colorSampler;

layout(location = 0) out vec4 outColor;

void main() {
  vec4 tx = texture(colorSampler, fragUv);
  float grey = (tx.r + tx.g + tx.b) / 3;

  float desat = post.b1 ? post.v1 : 1;
  vec3 col = mix(vec3(grey), tx.rgb, desat);

  outColor = vec4(col, 1.0);
}
