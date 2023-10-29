#version 450

#include "structs.glsl"

layout(location = 0) in vec2 fragUv;
layout(origin_upper_left) in vec4 gl_FragCoord;

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(set = 0, binding = 1) uniform DebugBlock {
  DebugData debug;
};
layout(set = 0, binding = 2) uniform sampler2D sceneSampler;
layout(set = 0, binding = 3) uniform sampler2D normDepthSampler;

layout(location = 0) out vec4 outColor;

vec4 getViewPos(vec3 cpos) {
  vec4 ndc = vec4(cpos, 1);
  vec4 vpos = global.inv_proj * ndc;
  vpos /= vpos.w;
  return vpos;
}

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

  vec2 clipXy = vec2(gl_FragCoord.xy / size * 2) - 1;
  vec4 vpos = getViewPos(vec3(clipXy, z));
  
  vec3 color = scene.rgb;
  if (desat) {
    float grey = dot(scene.rgb, vec3(.21, .72, .07));
    color = mix(vec3(grey), scene.rgb, debug.f1);
  } else if (debug.i1 == 1) {
    color = vec3(norm.x, norm.y, -norm.z);
  } else if (debug.i1 == 2) {
    color = vec3(fract(vpos.z / 500));
  }


  float depth_thresh = debug.f3;

  // Larger grid size allows for wider outlines, but it's slower
  const int grid = 3; 
  const int num = grid * grid - 1;
  int cent = grid / 2;
  int ind = 0;
  vec2 nUvs[num];
  for (int i = 0; i < grid; i++) {
    for (int j = 0; j < grid; j++) {
      if (i == cent && j == cent) {
        continue;
      }
      nUvs[ind] = vec2(pixel.x * (i - cent), pixel.y * (j - cent));
      ind++;
    }
  }

  if (draw_edges) {
    bool is_edge = false;
    float edge_d = length(size * nUvs[0]) + sqrt(2)/2;
    for (int i = 0; i < nUvs.length(); i++) {
      vec4 samp = texture(normDepthSampler, fragUv + nUvs[i]);
      vec4 sampVpos = getViewPos(vec3(clipXy + 2 * nUvs[i], samp.w));

      float cosang = abs(dot(norm, samp.xyz));
      float plane_d = abs(dot(norm, sampVpos.xyz - vpos.xyz));

      if (cosang < abs(cos(radians(debug.f4))) || plane_d > depth_thresh) {
        if (b3) {
          is_edge = true;
          break;
        } else {
          vec2 ndist = nUvs[i] * size;
          // Clip the distance vector to the edge of the pixel.
          ndist -= 0.5 * ndist / max(abs(ndist.x), abs(ndist.y));
          edge_d = min(edge_d, length(ndist));
        }
      }
    }
    
    float softw = 0.25;
    float alpha = smoothstep(debug.f1 - softw, debug.f1 + softw, edge_d);

    if (b3) {
      alpha = is_edge ? 0 : 1;
    }
    
    vec3 edgecol = vec3(0);
    color = mix(edgecol, color, alpha);
  }

  outColor = vec4(color, 1.0);
}
