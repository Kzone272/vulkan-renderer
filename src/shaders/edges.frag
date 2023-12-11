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
layout(set = 2, binding = 0) uniform sampler2DMS samplePoints;

layout(location = 0) out vec2 outColor;


bool edgeBetween(vec3 aNorm, vec3 aPos, vec3 bNorm, vec3 bPos) {
  float cosang = abs(dot(aNorm, bNorm));
  float plane_d = abs(dot(aNorm, bPos - aPos));

  const float depth_thresh = debug.f3;
  const float angle_thresh = abs(cos(radians(debug.f4)));

  return cosang < angle_thresh || plane_d > depth_thresh;
}

void main() {
  ivec2 iSize = textureSize(normDepthSampler);
  vec2 size = vec2(iSize);
  vec2 fPx = size * fragUv;

  const int nSamples = 4; // TODO: Make this configurable
  vec2 subsamps[nSamples];
  for (int i = 0; i < nSamples; i++) {
    subsamps[i] = texelFetch(samplePoints, ivec2(0), i).xy;
  }

  // Neighbours compared.
  ivec2 nbs[] = {
    ivec2(0, 0),
    ivec2(0, -1),
    ivec2(-1, 0),
  };

  const int numNbs = nbs.length();
  const int num = numNbs * nSamples;
  bool valid[num];
  vec4 norms[num];
  vec3 worlds[num];
  vec2 uvs[num];
  ivec2 iUv = ivec2(fPx);
  for (int i = 0; i < numNbs; i++) {
    vec2 clipXy = vec2(fragUv * 2) - 1;
    for (int j = 0; j < nSamples; j++) {
      const int ind = i * nSamples + j;
      ivec2 nbUv = iUv + nbs[i];
      if (nbUv.x < 0 || nbUv.x >= iSize.x ||
          nbUv.y < 0 || nbUv.y >= iSize.y) {
        valid[ind] = false;
        uvs[ind] = vec2(1, 0); // signal error
        continue;
      }

      vec4 samp = texelFetch(normDepthSampler, nbUv, j);
      valid[ind] = true;
      norms[ind] = samp;
      uvs[ind] = (vec2(iUv) + subsamps[j]);
      worlds[ind] = getViewPos(
          vec3(clipXy + 2.f * nbs[i] / size, samp.w), global.inv_proj).xyz;
    }
  }

  int nEdges = 0;
  vec2 edge_p_acc = vec2(0);
  for (int i = 0; i < nSamples; i++) {
    bool is_edge = false;
    float min_d2 = 1000000000;
    vec2 min_edge_p;

    for (int j = 0; j < num; j++) {
      if (j == i) {
        continue;
      }
      if (!valid[i] || !valid[j]) {
        // One sample is out of bounds.
        continue;
      }
      if (norms[i] == norms[j]) {
        // Exact same value from a different subsample of the same pixel.
        continue;
      }
      if (edgeBetween(norms[i].xyz, worlds[i], norms[j].xyz, worlds[j])) {
        vec2 delta = uvs[j] - uvs[i];
        float d2 = dot(delta, delta);
        if (d2 < min_d2) {
          is_edge = true;
          min_d2 = d2;
          min_edge_p = (uvs[i] + uvs[j]) / 2.f;
        }
      }
    }
    if (is_edge) {
      nEdges++;
      edge_p_acc += min_edge_p;
    }
  }
  edge_p_acc /= nEdges;

  if (nEdges > 0) {
    outColor = edge_p_acc / size;
  } else {
    outColor = vec2(-1);
  }
}
