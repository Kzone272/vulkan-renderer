// Common math functions for determining edges based on normal and depth buffers

bool edgeBetween(
    vec3 aNorm, vec3 aPos, vec3 bNorm, vec3 bPos,
    float depth_thresh, float angle_thresh) {
  float cosang = abs(dot(aNorm, bNorm));
  float plane_d = abs(dot(aNorm, bPos - aPos));

  return cosang < angle_thresh || plane_d > depth_thresh;
}
