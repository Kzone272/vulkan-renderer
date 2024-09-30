#version 450

#include "maths.glsl"

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform sampler2DMS image;

layout(push_constant) uniform PushBlock {
  mat4 inv_proj;
};

layout(location = 0) out vec4 outColor;

void main() {
  vec4 normDepth = texelFetch(image, ivec2(textureSize(image) * fragUv), 0);
  vec3 clipPos = getClipPos(fragUv, normDepth.w);
  vec4 vpos = getViewPos(clipPos, inv_proj);

  outColor = vec4(vec3(fract(vpos.z / 500)), 1);
}
