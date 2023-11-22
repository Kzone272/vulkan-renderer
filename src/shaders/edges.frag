#version 450

#include "structs.glsl"
#include "maths.glsl"

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 0, binding = 1) uniform DebugBlock {
  DebugData debug;
};
layout(set = 1, binding = 0) uniform sampler2DMS normDepthSampler;

layout(location = 0) out vec4 outColor;


bool edgeBetween(vec3 aNorm, vec3 aPos, vec3 bNorm, vec3 bPos) {
  float cosang = abs(dot(aNorm, bNorm));
  float plane_d = abs(dot(aNorm, bPos - aPos));

  const float depth_thresh = debug.f3;
  const float angle_thresh = abs(cos(radians(debug.f4)));

  return cosang < angle_thresh || plane_d > depth_thresh;
}

vec4 getEdgeColor(int msaaSample) {
  bool b3 = bool(debug.i4 & 0xFF0000);
  
  vec2 size = textureSize(normDepthSampler);
  vec2 pixel = 1 / size;

  ivec2 iUv = ivec2(size * fragUv);
  vec4 normDepth = texelFetch(normDepthSampler, iUv, msaaSample);
  vec3 norm = normDepth.xyz;
  float z = normDepth.w;

  vec2 clipXy = vec2(fragUv * 2) - 1;
  vec4 vpos = getViewPos(vec3(clipXy, z), global.inv_proj);

  vec3 edgeCol = vec3(0);
  vec4 color = vec4(edgeCol, 0);

  // Larger grid size allows for wider outlines, but it's slower
  const int search = 1;
  const int grid = 2 * search + 1;
  const int num = grid * grid - 1;
  int ind = 0;
  ivec2 nUvs[num];
  for (int i = -search; i <= search; i++) {
    for (int j = -search; j <= search; j++) {
      if (i == 0 && j == 0) {
        continue;
      }
      nUvs[ind] = ivec2(i, j);
      ind++;
    }
  }

  bool is_edge = false;
  float edge_d = length(nUvs[0]) + sqrt(2)/2;
  for (int i = 0; i < nUvs.length(); i++) {
    vec4 samp = texelFetch(normDepthSampler, iUv + nUvs[i], msaaSample);
    vec4 sampVpos = getViewPos(
        vec3(clipXy + 2 * pixel * nUvs[i], samp.w),
        global.inv_proj);

    if (edgeBetween(norm, vpos.xyz, samp.xyz, sampVpos.xyz)) {
      if (b3) {
        is_edge = true;
        break;
      } else {
        vec2 ndist = nUvs[i] / 2.f;
        edge_d = min(edge_d, length(ndist));
      }
    }
  }
  
  float softw = 0.25;
  float alpha = 1 - smoothstep(debug.f1 - softw, debug.f1 + softw, edge_d);
  if (b3) {
    alpha = is_edge ? 1 : 0;
  }
  
  color.a = alpha;

  return color;
}

void main() {
  vec4 edgeAcc = vec4(0);
  const int nSamples = textureSamples(normDepthSampler);
  for (int i = 0; i < nSamples; i++) {
    edgeAcc += getEdgeColor(i);
  }
  edgeAcc /= nSamples;

  outColor = edgeAcc;
}
