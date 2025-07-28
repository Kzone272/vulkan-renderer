// Common math functions that are useful in various shaders.

float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

float greyscale(vec3 color) {
  return dot(color, vec3(.21, .72, .07));
}

vec4 getViewPos(vec3 cpos, mat4 inv_proj) {
  vec4 ndc = vec4(cpos, 1);
  vec4 vpos = inv_proj * ndc;
  vpos /= vpos.w;
  return vpos;
}

vec3 getClipPos(vec2 uvPos, float z) {
  return vec3(2 * uvPos - 1, z);
}

// Project point onto the line (start->end) and determine how far it is
// along that line is as a percentage. 0 = start, 1 = end. 0.5 = halfway.
// Value clamped to [0, 1].
float pctAlong(vec2 point, vec2 start, vec2 end) {
  vec2 dir = end - start;
  vec2 toPoint = point - start;
  float dirLenSq = dot(dir, dir);
  if (dirLenSq == 0) {
    return 0; // Bad input
  }

  float t = dot(toPoint, dir) / dirLenSq;
  return clamp(t, 0.f, 1.f);
}
