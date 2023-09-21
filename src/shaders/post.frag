#version 450

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform sampler2D colorSampler;

layout(location = 0) out vec4 outColor;

void main() {
  vec4 tx = texture(colorSampler, fragUv);
  float grey = (tx.r + tx.g + tx.b) / 3;
  vec3 col = mix(vec3(grey), tx.rgb, 0.5); // desaturate 50%

  outColor = vec4(col, 1.0);
}
