#version 450

layout(location = 0) in vec2 fragUv;

// TODO: Move this into shared glsl code.
layout(set = 0, binding = 0) uniform DebugData {
  float f1, f2, f3, f4;
  int i1, i2, i3, i4;
} debug;
layout(set = 0, binding = 1) uniform sampler2D sceneSampler;
layout(set = 0, binding = 2) uniform sampler2D normDepthSampler;

layout(location = 0) out vec4 outColor;

void main() {
  vec4 scene = texture(sceneSampler, fragUv);
  vec4 normDepth = texture(normDepthSampler, fragUv);

  bool desat = bool(debug.i4 & 0xFF);
  
  vec3 color = scene.rgb;
  if (desat) {
    float grey = dot(scene.rgb, vec3(.21, .72, .07));
    color = mix(vec3(grey), scene.rgb, debug.f1);
  } else if (debug.i1 == 1) {
    color = normDepth.rgb;
  } else if (debug.i1 == 2) {
    color = vec3(normDepth.w);
  }

  outColor = vec4(color, 1.0);
}
