#version 450

#include "structs.glsl"
#include "draw-2d.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(push_constant) uniform PushBlock {
  Draw2d draw;
};

layout(location = 0) out vec2 position;

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
  vec2 size = vec2(global.width, global.height);
  vec2 aspect = vec2(size.y / size.x, 1);
  vec2 pixel = 2 / size;

  float halfW = 0.5 * draw.values.x + max(pixel.x, pixel.y);

  position = vec2(0);

  if (draw.type == k2dLine) {
    vec2 dir = (draw.pos2 - draw.pos1) / aspect;
    vec2 orthoDir = aspect * normalize(vec2(dir.y, -dir.x));

    vec2 startL = draw.pos1 - halfW * orthoDir;
    vec2 startR = draw.pos1 + halfW * orthoDir;
    vec2 endL = draw.pos2 - halfW * orthoDir;
    vec2 endR = draw.pos2 + halfW * orthoDir;

    if (gl_VertexIndex == 0 || gl_VertexIndex == 3) {
      position = startL;
    } else if (gl_VertexIndex == 1 || gl_VertexIndex == 5) {
      position = endR;
    } else if (gl_VertexIndex == 2) {
      position = startR;
    } else if (gl_VertexIndex == 4) {
      position = endL;
    }
  } else if (draw.type == k2dCircle) {
    float r = draw.values.y + halfW + max(pixel.x, pixel.y);
    position = draw.pos1 + r * aspect * positions[gl_VertexIndex];
  }

  gl_Position = vec4(position, 0, 1);
}
