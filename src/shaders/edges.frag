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
layout(set = 1, binding = 0) uniform sampler2DMS normDepthSampler;
layout(set = 2, binding = 0) uniform sampler2DMS samplePoints;

layout(location = 0) out vec2 outColor;


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
          getClipPos(fragUv + nbs[i] / size, samp.w), global.inv_proj).xyz;
    }
  }

  const float depth_thresh = debug.f3;
  const float angle_thresh = abs(cos(radians(debug.f4)));

  int nEdges = 0;
  vec2 edgeAcc = vec2(0);
  for (int i = 0; i < nSamples; i++) {
    vec2 subEdgeAcc = vec2(0);
    int nSubEdges = 0;

    for (int j = i + nSamples; j < num; j += nSamples) {
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
      if (edgeBetween(
            norms[i].xyz, worlds[i], norms[j].xyz, worlds[j],
            depth_thresh, angle_thresh)) {
        nSubEdges++;
        subEdgeAcc += (uvs[i] + uvs[j]) / 2.f;
      }
    }

    if (nSubEdges > 0) {
      nEdges++;
      edgeAcc += subEdgeAcc / nSubEdges;
    }
  }
  edgeAcc /= nEdges;

  if (nEdges > 0) {
    outColor = edgeAcc / size;
  } else {
    outColor = vec2(-1);
  }
}
