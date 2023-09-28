#version 450

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform PostFxData {
  float v1, v2, v3, v4;
  bool b1, b2, b3, b4;
} post;

layout(location = 0) out vec4 outColor;

void main() {
  vec3 col = vec3(0);
  if (length(fragUv - vec2(0.5)) < post.v2) {
    col = vec3(1,0,0);
  }

  outColor = vec4(col, 1.0);
}
