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
layout(set = 0, binding = 4) uniform sampler shadowSampler;
layout(set = 0, binding = 5) uniform texture2D shadowTex;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragColor;
layout(location = 2) in vec2 fragUv;
layout(location = 3) in vec3 fragPos;
layout(location = 4) flat in uint matIndex;
layout(location = 5) in vec4 shadowPos;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormalDepth;

#include "shadow.glsl"

float toonDirLight(vec3 lightVec, vec3 norm) {
  vec3 l = -normalize(vec3(global.view * vec4(normalize(lightVec), 0)));
  // 0 on dark side, 1 on light side.
  return step(0, dot(l, norm));
}

void main() {
  vec3 vnorm = normalize(fragNormal);
  float z = gl_FragCoord.z;
  outNormalDepth = vec4(vnorm.xyz, z);

  MaterialData material = materials[matIndex];

  vec2 uv = gl_FragCoord.xy / vec2(global.width, global.height);
  float t = pctAlong(uv, material.data1.xy, material.data1.zw);

  float sun = shadowPct() * toonDirLight(global.lights[0].vec.xyz, vnorm);
  vec3 color;
  if (sun > 0.5) {
    color = oklabMix(material.color1, material.color2, t);
  } else {
    color = oklabMix(material.color3.rgb, material.color4.rgb, t);
  }

  outColor = vec4(color, 1);
}
