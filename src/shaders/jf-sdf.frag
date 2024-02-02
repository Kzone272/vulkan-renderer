#version 450

layout(location = 0) in vec2 fragUv;
layout(set = 0, binding = 0) uniform sampler2D image;
layout(push_constant) uniform PushBlock {
  float sdf_width;
};

layout(location = 0) out vec4 outColor;

void main() {
  const ivec2 size = textureSize(image, 0);
  vec2 samp = texture(image, fragUv).rg;
  if (samp == vec2(-1, -1)) {
    outColor = vec4(0,0,0,1);
  } else {
    float dist = length(size * (samp - fragUv));
    
    outColor = vec4(samp, 0, 1);
    return;

    float alpha = clamp(sdf_width - dist, 0, 1);
    // Cool repeated outlines effect.
    // alpha *= smoothstep(-0.2, 0.2, sin(dist / 3));
    vec4 col = vec4(0, 0, 0, alpha);
    // Display jump flood beneath edges:
    // col = mix(col, vec4(samp, 0, 1), 1 - col.a);
    outColor = col;
  }
}
