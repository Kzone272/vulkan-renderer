#version 450

#include "structs.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};

layout(location = 0) out vec3 fragUvw;

vec3 positions[8] = vec3[](
  vec3(-1,  1,  1), // 0: left up forward
  vec3( 1,  1,  1), // 1: right up forward
  vec3(-1, -1,  1), // 2: left down forward
  vec3( 1, -1,  1), // 3: right down forward
  vec3(-1,  1, -1), // 4: left up backward
  vec3( 1,  1, -1), // 5: right up backward
  vec3(-1, -1, -1), // 6: left down backward
  vec3( 1, -1, -1) // 7: right down backward
);

uint indices[36] = uint[](
  0, 1, 2, 2, 1, 3, // forward
  5, 4, 7, 7, 4, 6, // backward
  4, 0, 6, 6, 0, 2, // left
  1, 5, 3, 3, 5, 7, // right
  4, 5, 0, 0, 5, 1, // up
  2, 3, 6, 6, 3, 7 // down
);

void main() {
  fragUvw = 2 * global.near * positions[indices[gl_VertexIndex]];
  
  // Ignore view matrix's translation with w = 0.
  vec4 viewPos = global.view * vec4(fragUvw, 0);
  viewPos.w = 1; // Set back to a position.
  gl_Position = global.proj * viewPos;
}
