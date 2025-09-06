#version 450

#include "structs.glsl"
#include "draw-2d.glsl"

layout(set = 0, binding = 0) uniform GlobalBlock {
  GlobalData global;
};
layout(push_constant) uniform PushBlock {
  Draw2d draw;
};

layout(location = 0) in vec2 inPosition;

layout(location = 0) out vec4 outColor;

void main() {
  vec2 size = vec2(global.width, global.height);

  float dist = 0;
  float alpha = 1;

  float halfW = 0.5 * draw.values.x * 0.5 * size.y;

  if (draw.type == k2dLine) {
    vec2 dir = normalize(draw.pos2 - draw.pos1);
    vec2 toPos = inPosition.xy - draw.pos1;
    vec2 proj = dot(toPos, dir) * dir;
    vec2 rej = toPos - proj;
    
    float lineDist = length(0.5 * size * rej);
    dist = halfW - lineDist;
  } else if (draw.type == k2dCircle) {
    float circleDist = length(0.5 * size * (inPosition - draw.pos1));
    float r = draw.values.y * 0.5 * size.y;
    dist = halfW - abs(circleDist - r);
  }

  alpha = smoothstep(-0.5, 0.5, dist);
  outColor = vec4(draw.color.rgb, draw.color.a * alpha);
}
