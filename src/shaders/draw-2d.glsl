// Common definitions used for 2d draws.

struct Draw2d {
  vec2 pos1;
  vec2 pos2;
  vec4 color;
  vec4 values;
  uint type;
};

uint k2dLine = 0;
uint k2dCircle = 1;
