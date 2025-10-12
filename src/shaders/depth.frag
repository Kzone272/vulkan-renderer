#version 450

#include "maths.glsl"

layout(location = 0) in vec2 fragUv;

layout(set = 0, binding = 0) uniform sampler2D image;

layout(push_constant) uniform PushBlock {
  mat4 invProj;
};

layout(location = 0) out vec4 outColor;

void main() {
  vec4 normDepth = texture(image, fragUv);
  vec3 clipPos = getClipPos(fragUv, normDepth.w);
  vec4 vpos = getViewPos(clipPos, invProj);

  outColor = vec4(vec3(fract(vpos.z / 500)), 1);
}
