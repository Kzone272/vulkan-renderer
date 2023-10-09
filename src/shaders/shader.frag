#version 450

struct Light {
  uint type;
  vec3 vec;
  vec3 color;
  float falloff;
};
const uint kDirectionalLightType = 1;
const uint kPointLightType = 2;

layout(set = 0, binding = 0) uniform Global {
  mat4 view;
  mat4 proj;
  Light lights[8];
} global;

layout(set = 1, binding = 0) uniform sampler2D texSampler;
layout(set = 1, binding = 1) uniform Material {
  vec3 color;
} material;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragColor;
layout(location = 2) in vec2 fragUv;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormalDepth;


// TODO: Put this in some shared place?
float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}


float dirLight(vec3 l, vec3 norm) {
  return max(0, dot(l, norm));
}

float pointLight(vec3 p, float falloff, vec3 norm) {
  vec3 toLight = p - fragPos;
  vec3 l = normalize(toLight);
  float intensity = clamp(map(length(toLight), 0, falloff, 1, 0), 0, 1);
  return max(0, intensity * dot(l, norm));
}

void main() {
  vec4 diffuse = texture(texSampler, fragUv)
      * vec4(fragColor, 1.0)
      * vec4(material.color, 1.0);

  vec3 wnorm = normalize(fragNormal);
  vec4 vpos = global.view * vec4(fragPos, 1);
  vec4 vnorm = global.view * vec4(wnorm, 0);
  float z = gl_FragCoord.z;
  outNormalDepth = vec4(vnorm.xyz, z);

  vec3 lambert = vec3(0);
  for (int i = 0; i < global.lights.length(); i++) {
    Light light = global.lights[i];

    float intensity = 0;
    if (light.type == kDirectionalLightType) {
      intensity = dirLight(normalize(-light.vec), wnorm);
    } else if (light.type == kPointLightType) {
      intensity = pointLight(light.vec, light.falloff, wnorm);
    }

    lambert += intensity * light.color * diffuse.rgb;
  }

  // Global illumination
  // TODO: Make this a light type.
  lambert += vec3(0.2) * diffuse.rgb;

  lambert = clamp(lambert, 0, 1);
  outColor = vec4(lambert, diffuse.a);
}
