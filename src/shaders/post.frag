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
  vec2 size = textureSize(normDepthSampler, 0);
  vec2 pixel = 1 / size;
  
  bool desat = bool(debug.i4 & 0xFF);
  bool draw_edges = bool(debug.i4 & 0xFF00);

  vec4 scene = texture(sceneSampler, fragUv);
  vec4 normDepth = texture(normDepthSampler, fragUv);

  vec4 neigh[8];
  if (draw_edges) {
    neigh[0] = texture(normDepthSampler, fragUv + (0, pixel.y));
    neigh[1] = texture(normDepthSampler, fragUv + (0, -pixel.y));
    neigh[2] = texture(normDepthSampler, fragUv + (-pixel.x, 0));
    neigh[3] = texture(normDepthSampler, fragUv + (pixel.x, 0));
    neigh[4] = texture(normDepthSampler, fragUv + (pixel.x, pixel.y));
    neigh[5] = texture(normDepthSampler, fragUv + (-pixel.x, pixel.y));
    neigh[6] = texture(normDepthSampler, fragUv + (pixel.x, -pixel.y));
    neigh[7] = texture(normDepthSampler, fragUv + (-pixel.x, -pixel.y));
  }
  
  vec3 color = scene.rgb;
  if (desat) {
    float grey = dot(scene.rgb, vec3(.21, .72, .07));
    color = mix(vec3(grey), scene.rgb, debug.f1);
  } else if (debug.i1 == 1) {
    color = normDepth.rgb;
  } else if (debug.i1 == 2) {
    color = vec3(normDepth.w);
  }

  if (draw_edges) {
    bool edge = false;
    for (int i = 0; i < neigh.length(); i++) {
      float cosang = abs(dot(normDepth.xyz, neigh[i].xyz));
      float diff = abs(neigh[i].w - normDepth.w);

      if (cosang < debug.f4 || diff > debug.f3) {
        edge = true;
        break;
      }
    }

    if (edge) {
      color = vec3(0);
    }
  }

  outColor = vec4(color, 1.0);
}
