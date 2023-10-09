#version 450

layout(location = 0) in vec2 fragUv;

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
  float z = normDepth.w;
  
  vec4 vnorm = vec4(normDepth.xyz, 0);
  vec4 cvec = vec4(gl_FragCoord.xz, 1, 0);
  vec4 vvec = inverse(global.proj) * cvec;
  vec3 eye = normalize(-vvec.xyz);

  
  vec3 color = scene.rgb;
  if (desat) {
    float grey = dot(scene.rgb, vec3(.21, .72, .07));
    color = mix(vec3(grey), scene.rgb, debug.f1);
  } else if (debug.i1 == 1) {
    color = normDepth.rgb;
  } else if (debug.i1 == 2) {
    color = vec3(z);
  }
  // TODO: Modify this code for SDF edges.
  // vec4 neigh[8];
  // if (draw_edges) {
  //   neigh[0] = texture(normDepthSampler, fragUv + (0, pixel.y));
  //   neigh[1] = texture(normDepthSampler, fragUv + (0, -pixel.y));
  //   neigh[2] = texture(normDepthSampler, fragUv + (-pixel.x, 0));
  //   neigh[3] = texture(normDepthSampler, fragUv + (pixel.x, 0));
  //   neigh[4] = texture(normDepthSampler, fragUv + (pixel.x, pixel.y));
  //   neigh[5] = texture(normDepthSampler, fragUv + (-pixel.x, pixel.y));
  //   neigh[6] = texture(normDepthSampler, fragUv + (pixel.x, -pixel.y));
  //   neigh[7] = texture(normDepthSampler, fragUv + (-pixel.x, -pixel.y));
  // }

  // if (draw_edges) {
  //   bool edge = false;
  //   for (int i = 0; i < neigh.length(); i++) {
  //     float cosang = abs(dot(normDepth.xyz, neigh[i].xyz));
  //     float diff = abs(neigh[i].w - z);

  //     if (cosang < cos(debug.f4) || diff > depth_thresh) {
  //       edge = true;
  //       break;
  //     }
  //   }

  //   if (edge) {
  //     color = vec3(0);
  //   }
  // }


  float fresnel = 1 - dot(eye, vnorm.xyz);
  float depth_scale = 0.6;
  float fresnel_fact = 1 +
      clamp((fresnel - depth_scale) / (1 - depth_scale), 0, 1);

  float depth_thresh = debug.f3 * fresnel_fact;
  
  if (draw_edges) {
    float halfFloor = floor(debug.f1 * 0.5);
    float halfCeil = ceil(debug.f1 * 0.5);

    vec2 dlUv = fragUv - pixel * halfFloor;
    vec2 urUv = fragUv + pixel * halfCeil;  
    vec2 drUv = fragUv + vec2(pixel.x * halfCeil, -pixel.y * halfFloor);
    vec2 ulUv = fragUv + vec2(-pixel.x * halfFloor, pixel.y * halfCeil);

    vec4 dr = texture(normDepthSampler, drUv);
    vec4 ul = texture(normDepthSampler, ulUv);
    vec4 dl = texture(normDepthSampler, dlUv);
    vec4 ur = texture(normDepthSampler, urUv);
    float d0 = dr.w - ul.w;
    float d1 = dl.w - ur.w;
    float depth_diff = sqrt(d0 * d0 + d1 * d1);

    float n0 = abs(dot(dr.xyz, normDepth.xyz));
    float n1 = abs(dot(ul.xyz, normDepth.xyz));
    float n2 = abs(dot(dl.xyz, normDepth.xyz));
    float n3 = abs(dot(ur.xyz, normDepth.xyz));
    float norm_diff = min(min(n0, n1), min(n2, n3));

    if (depth_diff > depth_thresh || norm_diff < cos(debug.f4)) {
      color = vec3(0);
    }
  }

  outColor = vec4(color, 1.0);
}
