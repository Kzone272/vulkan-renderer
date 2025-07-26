#version 450

#include "structs.glsl"
#include "maths.glsl"
#include "colors.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 0, binding = 3) readonly buffer MaterialBlock {
  MaterialData materials[];
};

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragColor;
layout(location = 2) in vec2 fragUv;
layout(location = 3) in vec3 fragPos;
layout(location = 4) flat in uint matIndex;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormalDepth;

void main() {
  vec3 vnorm = normalize(fragNormal);
  float z = gl_FragCoord.z;
  outNormalDepth = vec4(vnorm.xyz, z);

  MaterialData material = materials[matIndex];

  float t = gl_FragCoord.y / global.height;
  vec3 color = oklabMix(material.color1, material.color2, t);

  outColor = vec4(color, 1);
}
