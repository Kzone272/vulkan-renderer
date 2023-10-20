#version 450

layout(location = 0) in vec2 inPos;
layout(location = 1) in vec3 inColor;

layout(location = 0) out vec2 cellCentroid;
layout(location = 1) out vec3 cellColor;
layout(location = 2) out vec2 cellPos;

const float R = 2;

// Square
vec2 positions[6] = vec2[](
  vec2(-1, -1),
  vec2( 1, -1),
  vec2(-1,  1),
  vec2(-1,  1),
  vec2( 1, -1),
  vec2( 1, 1)
);

void main() {
  cellCentroid = inPos;
  cellColor = inColor;
  cellPos = positions[gl_VertexIndex] + inPos;
  gl_Position = vec4(R * positions[gl_VertexIndex] + inPos, 0, 1);
}
