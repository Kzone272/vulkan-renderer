#version 450

#include "structs.glsl"
#include "maths.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 1, binding = 0) uniform sampler2D texSampler;
layout(set = 1, binding = 1) uniform MaterialBlock {
  MaterialData material;
};

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragColor;
layout(location = 2) in vec2 fragUv;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormalDepth;


float dirLight(vec3 lightVec, vec3 norm) {
  vec3 l = -normalize(vec3(global.view * vec4(lightVec, 0)));
  return max(0, dot(l, norm));
}

float pointLight(vec3 lightVec, float falloff, vec3 norm) {
  vec3 lightPos = vec3(global.view * vec4(lightVec, 1));
  vec3 toLight = lightPos - fragPos;
  vec3 l = normalize(toLight);
  float intensity = clamp(map(length(toLight), 0, falloff, 1, 0), 0, 1);
  return max(0, intensity * dot(l, norm));
}

void main() {
  vec3 vnorm = normalize(fragNormal);
  float z = gl_FragCoord.z;
  outNormalDepth = vec4(vnorm.xyz, z);

  vec3 color;
  if (material.type == kPhongMaterial) {
    vec3 diffuse = texture(texSampler, fragUv).rgb
        * fragColor
        * material.color1;

    vec3 lambert = vec3(0);
    for (int i = 0; i < global.lights.length(); i++) {
      Light light = global.lights[i];

      float intensity = 0;
      if (light.type == kDirectionalLightType) {
        intensity = dirLight(light.vec, vnorm);
      } else if (light.type == kPointLightType) {
        intensity = pointLight(light.vec, light.falloff, vnorm);
      }

      lambert += intensity * light.color * diffuse;
    }

    // Global illumination
    // TODO: Make this a light type.
    lambert += vec3(0.2) * diffuse;
    color = clamp(lambert, 0, 1);
  } else if (material.type == kGoochMaterial) {
    vec3 L = normalize(vec3(1, 1, 1));
    float gooch = (1 + dot(vnorm, L)) / 2;
    color = mix(material.color2, material.color1, gooch);
  }

  outColor = vec4(color, 1);
}
