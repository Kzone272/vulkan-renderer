#version 450

#include "structs.glsl"
#include "maths.glsl"
#include "edge-maths.glsl"

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 0, binding = 1) uniform DebugBlock {
  DebugData debug;
};
layout(set = 1, binding = 0) uniform sampler2D normDepthSampler;

layout(location = 0) out uint outColor;

void main() {
  ivec2 iSize = textureSize(normDepthSampler, 0);
  vec2 pixel = 1 / vec2(iSize);

  vec2 uvM = fragUv; // Middle
  vec2 uvN = fragUv + vec2(0, -pixel.y); // North
  vec2 uvW = fragUv + vec2(-pixel.x, 0); // West

  vec4 sampM = texture(normDepthSampler, uvM);
  vec3 worldM = getViewPos(getClipPos(uvM, sampM.w), global.inv_proj).xyz;

  vec4 sampN = texture(normDepthSampler, uvN);
  vec3 worldN = getViewPos(getClipPos(uvN, sampN.w), global.inv_proj).xyz;

  vec4 sampW = texture(normDepthSampler, uvW);
  vec3 worldW = getViewPos(getClipPos(uvW, sampW.w), global.inv_proj).xyz;

  const float depth_thresh = debug.f3;
  const float angle_thresh = abs(cos(radians(debug.f4)));

  outColor = 0;

  if (edgeBetween(
        sampM.xyz, worldM, sampN.xyz, worldN, depth_thresh, angle_thresh)) {
    outColor |= EDGE_UP;
  }
  if (edgeBetween(
        sampM.xyz, worldM, sampW.xyz, worldW, depth_thresh, angle_thresh)) {
    outColor |= EDGE_LEFT;
  }
}
