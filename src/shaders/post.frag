#version 450

layout(location = 0) in vec2 fragUv;
layout(origin_upper_left) in vec4 gl_FragCoord;

// TODO: Move this into shared glsl code.
layout(set = 0, binding = 0) uniform Global {
  mat4 view;
  mat4 proj;
} global;
layout(set = 0, binding = 1) uniform DebugData {
  float f1, f2, f3, f4;
  int i1, i2, i3, i4;
} debug;
layout(set = 0, binding = 2) uniform sampler2D sceneSampler;
layout(set = 0, binding = 3) uniform sampler2D normDepthSampler;

layout(location = 0) out vec4 outColor;

void main() {
  vec2 size = textureSize(normDepthSampler, 0);
  vec2 pixel = 1 / size;
  
  bool desat = bool(debug.i4 & 0xFF);
  bool draw_edges = bool(debug.i4 & 0xFF00);
  bool b3 = bool(debug.i4 & 0xFF0000);
  bool b4 = bool(debug.i4 & 0xFF000000);

  vec4 scene = texture(sceneSampler, fragUv);
  vec4 normDepth = texture(normDepthSampler, fragUv);
  vec3 norm = normDepth.xyz;
  float z = normDepth.w;

  mat4 ctow = inverse(global.proj * global.view);
  vec4 ndc = vec4((gl_FragCoord.xy / size - 0.5) * 2, z, 1);
  vec4 wpos = ctow * ndc;
  
  vec3 color = scene.rgb;
  if (desat) {
    float grey = dot(scene.rgb, vec3(.21, .72, .07));
    color = mix(vec3(grey), scene.rgb, debug.f1);
  } else if (debug.i1 == 1) {
    color = norm;
  } else if (debug.i1 == 2) {
    color = vec3(z);
  }


  float depth_thresh = debug.f3;

  // TODO: Modify this code for SDF edges.
  vec2 nUvs[8] = {
    vec2(0, pixel.y),
    vec2(0, -pixel.y),
    vec2(-pixel.x, 0),
    vec2(pixel.x, 0),
    vec2(pixel.x, pixel.y),
    vec2(-pixel.x, pixel.y),
    vec2(pixel.x, -pixel.y),
    vec2(-pixel.x, -pixel.y),
  };
  if (draw_edges) {
    bool edge = false;
    for (int i = 0; i < nUvs.length(); i++) {
      vec4 samp = texture(normDepthSampler, fragUv + nUvs[i]);
      vec4 samp_cpos = vec4(ndc.xy + 2 * nUvs[i], samp.w, 1);
      vec4 samp_wpos = ctow * samp_cpos;

      float cosang = abs(dot(norm, samp.xyz));
      float plane_d = abs(dot(norm, samp_wpos.xyz - wpos.xyz));

      if (cosang < cos(debug.f4) || plane_d > depth_thresh) {
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
