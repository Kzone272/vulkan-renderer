#version 450

#include "structs.glsl"
#include "edge-maths.glsl"

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 0, binding = 1) uniform DebugBlock {
  DebugData debug;
};
layout(set = 1, binding = 0) uniform usampler2D prepassSampler;

layout(location = 0) out vec2 outColor;

const int ITERS = 12;
const float Q[ITERS] = float[](
  1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 4.0, 8.0);

void main() {
  ivec2 iSize = textureSize(prepassSampler, 0);
  vec2 size = vec2(iSize);
  vec2 pixel = 1 / size;

  uint samp = texture(prepassSampler, fragUv).r;
  bool edgeLeft = bool(samp & EDGE_LEFT);
  bool edgeUp = bool(samp & EDGE_UP);
  if (!bool(samp)) {
    outColor = vec2(-1);
    return;
  }

  bool vert = edgeLeft;
  if (edgeLeft && edgeUp) {
    uint sampN = texture(prepassSampler, fragUv + vec2(0, -pixel.y)).r;
    uint sampS = texture(prepassSampler, fragUv + vec2(0, pixel.y)).r;
    uint sampW = texture(prepassSampler, fragUv + vec2(-pixel.x, 0)).r;
    uint sampE = texture(prepassSampler, fragUv + vec2(pixel.x, 0)).r;
    uint numVert = ((sampN & EDGE_LEFT) + (sampS & EDGE_LEFT)) / EDGE_LEFT;
    uint numHor = ((sampW & EDGE_UP) + (sampE & EDGE_UP)) / EDGE_UP;
    vert = numVert > numHor;
  }
  uint dirMask = vert ? EDGE_LEFT : EDGE_UP;

  vec2 dir = pixel * (vert ? vec2(0, 1) : vec2(1, 0));
  vec2 orthoDir = pixel * (vert ? vec2(1, 0) : vec2(0, 1));
  vec2 uvPos = fragUv + dir;
  vec2 uvNeg = fragUv - dir;

  // Basically the FXAA algorithm, but ignoring luma.
  bool donePos = false;
  bool doneNeg = false;
  for (int i = 0; i < ITERS; i++) {
    if (!donePos) {
      uint sampPos = texture(prepassSampler, uvPos).r;
      donePos = !bool(sampPos & dirMask);
    }
    if (!doneNeg) {
      uint sampNeg = texture(prepassSampler, uvNeg).r;
      doneNeg = !bool(sampNeg & dirMask);
    }

    if (donePos && doneNeg) {
      break;
    }

    if (!donePos) {
      uvPos += dir * Q[i];
    }
    if (!doneNeg) {
      uvNeg -= dir * Q[i];
    }
  }

  float distPos = length(size * (uvPos - fragUv));
  float distNeg = length(size * (uvNeg - fragUv));
  float total = distPos + distNeg;
  float closer = min(distPos, distNeg);
  float edgeOffset = 0.5 - (closer / total);

  bool posCloser = distPos < distNeg;
  vec2 uvEnd = posCloser ? uvPos : uvNeg;
  uint sampOrthoPos = texture(prepassSampler, uvEnd + orthoDir).r;
  uint sampOrthoNeg = texture(prepassSampler, uvEnd - orthoDir).r;
  bool edgePos = bool(sampOrthoPos & dirMask);
  bool edgeNeg = bool(sampOrthoNeg & dirMask);
  float offsetDir = 0;
  if (edgePos != edgeNeg) {
    offsetDir = edgePos ? 1 : -1;
  }
  vec2 pixelOffset = edgeOffset * offsetDir * orthoDir;

  vec2 baseOffset = 0.5 * -orthoDir;
  outColor = fragUv + baseOffset + pixelOffset;
}
