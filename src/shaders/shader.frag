#version 450

layout(set = 1, binding = 0) uniform sampler2D texSampler;
layout(set = 1, binding = 1) uniform Material {
  vec3 color;
} material;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragColor;
layout(location = 2) in vec2 fragUv;

layout(location = 0) out vec4 outColor;

void main() {
  outColor = texture(texSampler, fragUv)
      * vec4(fragColor, 1.0)
      * vec4(material.color, 1.0);
}
