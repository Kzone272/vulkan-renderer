#version 450

layout(location = 0) in vec2 cellCentroid;
layout(location = 1) in vec3 cellColor;
layout(location = 2) in vec2 cellPos;

layout(location = 0) out vec4 outColor;

void main() {
  float dist = length(cellPos - cellCentroid);
  gl_FragDepth = 1.0 - dist;

  // vec3 col = vec3(gl_FragDepth);
  vec3 col = vec3(cellColor);
  // Draw centers
  if (dist < 0.02) {
    float alpha = smoothstep(0.019, 0.02, dist);
    col = mix(col, vec3(0), 1 - alpha);
  }

  outColor = vec4(col, 1.0);
}
