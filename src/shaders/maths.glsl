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
