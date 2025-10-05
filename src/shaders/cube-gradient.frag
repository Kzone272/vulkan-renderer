#version 450

#include "structs.glsl"
#include "colors.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};

layout(location = 0) in vec3 fragUvw;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormalDepth;

void main() {
  float t = dot(normalize(fragUvw), vec3(0, 1, 0));
  t = 0.5 * t + 0.5;

  vec3 color = oklabMix(global.colorDown.rgb, global.colorUp.rgb, t);

  outColor = vec4(color, 1);
}
